module udp_cam_ctrl_tf(
      input               clk,
      input               rst_n,

      // ===== SDRAM read side (接口已修正) =====
      output reg          read_req,
      input               read_req_ack,
      input               read_en,          // [!! 修复 !!] 改为 INPUT
      input      [31:0]   read_data,

      // ===== UDP stack interface =====
      input               udp_tx_ready,
      input               app_tx_ack,
      output reg          app_tx_data_request,
      output reg          app_tx_data_valid,
      output reg  [7:0]   app_tx_data,
      output reg [15:0]   udp_data_length
  );

  // -----------------------------------------------------------------------------
  // Constant configuration
  // -----------------------------------------------------------------------------
  localparam IMG_HEADER       = 32'hAA00_55FF;
  localparam IMG_WIDTH        = 32'd640;
  localparam IMG_HEIGHT       = 32'd480;
  localparam IMG_TOTAL        = IMG_WIDTH * IMG_HEIGHT * 3;   // RGB888
  localparam IMG_FRAMSIZE     = 32'd636;                      // packet payload (except last)
  localparam IMG_FRAMTOTAL    = 32'd1450;                     // ceil(921600 / 636)
  localparam LAST_FRAMSIZE    = 32'd36;                       // final packet payload
  localparam IMG_HEADER_LEN   = 9'd256;                       // 32 bytes header
  localparam HEADER_BYTES     = 16'd32;

  // ============================================================================
  // State machine encoding
  // ============================================================================
  // [!! 修复: 增加新状态 REQUEST_DATA !!]
  localparam START_UDP       = 4'd0;
  localparam WAIT_FIFO_RDY   = 4'd1;
  localparam WAIT_UDP_DATA   = 4'd2;
  localparam WAIT_ACK        = 4'd3;
  localparam SEND_UDP_HEADER = 4'd4;
  localparam REQUEST_DATA    = 4'd5; // [!! 新增状态 !!]
  localparam SEND_UDP_DATA   = 4'd6;
  localparam DELAY           = 4'd7;

  reg [3:0] state; // [!! 修复: 位宽改为 4 !!]

  // ============================================================================
  // Registers
  // ============================================================================
  reg [31:0] img_framseq;       
  reg [31:0] img_picseq;        
  reg [31:0] img_offset;        
  reg  [8:0] header_cnt;        
  reg [15:0] data_cnt;          
  reg [21:0] delay_cnt;         
  reg [31:0] data_reg;          
  reg  [1:0] byte_sel;          

  wire [31:0] curr_payload_bytes =
      (img_framseq == IMG_FRAMTOTAL - 1) ? LAST_FRAMSIZE : IMG_FRAMSIZE;
  wire [15:0] payload_limit      = curr_payload_bytes[15:0];
  wire        last_payload_byte  = (data_cnt + 16'd1 >= payload_limit);

  wire [255:0] udp_header = {
      curr_payload_bytes,
      img_framseq,
      img_picseq,
      img_offset,
      IMG_TOTAL,
      IMG_HEIGHT,
      IMG_WIDTH,
      IMG_HEADER
  };

  // ============================================================================
  // Main FSM
  // ============================================================================
  always @(posedge clk or negedge rst_n) begin
      if(!rst_n) begin
          state               <= START_UDP;
          app_tx_data_request <= 1'b0;
          app_tx_data_valid   <= 1'b0;
          app_tx_data         <= 8'd0;
          udp_data_length     <= HEADER_BYTES + IMG_FRAMSIZE[15:0];
          img_framseq         <= 32'd0;
          img_picseq          <= 32'd0;
          img_offset          <= 32'd0;
          header_cnt          <= 9'd0;
          data_cnt            <= 16'd0;
          delay_cnt           <= 22'd0;
          read_req            <= 1'b0;
          data_reg            <= 32'd0;
          byte_sel            <= 2'd0;
      end else begin
          case(state)
              START_UDP: begin
                  app_tx_data_request <= 1'b0;
                  app_tx_data_valid   <= 1'b0;
                  data_cnt            <= 16'd0;
                  img_framseq         <= 32'd0;
                  img_offset          <= 32'd0;
                  read_req            <= 1'b0;
                  img_picseq          <= img_picseq + 1'd1;
                  delay_cnt           <= 22'd0;
                  state               <= WAIT_FIFO_RDY;
              end

              // [!! 修复: 不在此处请求数据 !!]
              WAIT_FIFO_RDY: begin
                  if(delay_cnt >= 22'd2000) begin
                      delay_cnt <= 22'd0;
                      state     <= WAIT_UDP_DATA;
                  end else begin
                      delay_cnt <= delay_cnt + 1'd1;
                  end
                  
                  // [!! 修复: 删除 read_req 逻辑 !!]
                  // if(delay_cnt == 22'd10)
                  //     read_req <= 1'b1;
                  // else if(read_req_ack)
                  //     read_req <= 1'b0;
              end

              WAIT_UDP_DATA: begin
                  app_tx_data_request <= 1'b1;
                  app_tx_data_valid   <= 1'b0;

                  if(udp_tx_ready)
                      state <= WAIT_ACK;
              end

              WAIT_ACK: begin
                  app_tx_data_request <= app_tx_ack ? 1'b0 : 1'b1;
                  app_tx_data_valid   <= 1'b0;
                  read_req            <= 1'b0; // 确保 read_req 为低

                  if(app_tx_ack) begin
                      header_cnt        <= 9'd8;
                      data_cnt          <= 16'd0;
                      byte_sel          <= 2'd0;
                      app_tx_data_valid <= 1'b1;
                      app_tx_data       <= udp_header[7:0];
                      udp_data_length   <= HEADER_BYTES + payload_limit;
                      state             <= SEND_UDP_HEADER;
                  end
              end

              SEND_UDP_HEADER: begin
                  app_tx_data_valid <= 1'b1;
                  app_tx_data       <= udp_header[header_cnt +: 8];

                  if(header_cnt == IMG_HEADER_LEN - 9'd8) begin
                      header_cnt <= 9'd0;
                      app_tx_data_valid <= 1'b0;
                      state      <= REQUEST_DATA; // [!! 修复: 跳转到新状态 !!]
                  end else begin
                      header_cnt <= header_cnt + 9'd8;
                  end
              end

              // [!! 修复: 新增状态，在发送数据前请求 !!]
              REQUEST_DATA: begin
                  read_req <= 1'b1;
                  app_tx_data_valid <= 1'b0; // 保持无效

                  if (read_req_ack) begin
                      read_req <= 1'b0;
                      state <= SEND_UDP_DATA; // SDRAM 已确认，准备接收
                  end
              end

              // [!! 修复: 此状态逻辑现在可以正常工作了 !!]
              SEND_UDP_DATA: begin
                  app_tx_data_valid <= 1'b0; // 默认无效
                  read_req <= 1'b0; // 保持请求为低

                  if(byte_sel == 2'd0) begin // 状态 0: 等待 SDRAM 数据
                      if(read_en) begin // [!!] 等待 SDRAM 控制器发来 'read_en'
                          data_reg <= read_data; // 锁存有效数据
                          app_tx_data <= read_data[15:8]; // B
                          app_tx_data_valid <= 1'b1;
                          
                          if (last_payload_byte) begin
                              data_cnt <= 16'd0;
                              byte_sel <= 2'd0;
                              state <= DELAY;
                          end else begin
                              data_cnt <= data_cnt + 1'b1;
                              byte_sel <= 2'd1;
                          end
                      end
                  end
                  else if(byte_sel == 2'd1) begin // 状态 1: 发送 G
                      app_tx_data <= data_reg[23:16]; // G (来自上一拍锁存的 data_reg)
                      app_tx_data_valid <= 1'b1;
                      
                      if (last_payload_byte) begin
                          data_cnt <= 16'd0;
                          byte_sel <= 2'd0;
                          state <= DELAY;
                      end else begin
                          data_cnt <= data_cnt + 1'b1;
                          byte_sel <= 2'd2;
                      end
                  end
                  else begin // 状态 2: 发送 R
                      app_tx_data <= data_reg[31:24]; // R (来自 data_reg)
                      app_tx_data_valid <= 1'b1;
                      
                      if (last_payload_byte) begin
                          data_cnt <= 16'd0;
                          byte_sel <= 2'd0;
                          state <= DELAY;
                      end else begin
                          data_cnt <= data_cnt + 1'b1;
                          byte_sel <= 2'd0; // 回去等下一个 read_en
                      end
                  end
              end

              DELAY: begin
                  read_req <= 1'b0; // 确保为低

                  if(delay_cnt >= 22'd800) begin
                      delay_cnt   <= 22'd0;
                      img_framseq <= img_framseq + 1'd1;
                      img_offset  <= img_offset + curr_payload_bytes;

                      if(img_framseq >= (IMG_FRAMTOTAL - 1))
                          state <= START_UDP;
                      else
                          state <= WAIT_UDP_DATA;
                  end else begin
                      delay_cnt <= delay_cnt + 1'd1;
                  end
              end

              default: state <= START_UDP;
          endcase
      end
  end

  endmodule