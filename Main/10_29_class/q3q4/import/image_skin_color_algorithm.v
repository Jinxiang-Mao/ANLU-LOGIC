`timescale 1ns / 1ps
//********************************************************************** 
// -------------------------------------------------------------------
// Copyright Notice
// ------------------------------------------------------------------- 
// Author: Geeker_FPGA 
// Email:geeker_fpga@163.com 
// Email:geeker_fpga@uisrc.com v
// Date:2021/03/14
// Description: 
// skin_color_algorithm
//
// 
// Web:http://www.uisrc.com
//------------------------------------------------------------------- 
//*********************************************************************/

//肤色识别模块
module skin_color_algorithm
(
	input   wire				i_clk,				//输入时钟信号
	input   wire				i_rst_n,			//输入复位信号

	input	wire				i_hsyn,				//输入行同步信号
	input	wire				i_vsyn,				//输入场同步信号
	input	wire				i_de,				//输入数据有效信号
	input	wire [7:0]			i_r,				//输入图像R通道的值
	input	wire [7:0] 			i_g,				//输入图像G通道的值
	input	wire [7:0] 			i_b,				//输入图像B通道的值
	
	output	wire				o_hs,				//输出行同步信号
	output	wire				o_vs,				//输出场同步信号
	output	wire				o_de,				//输出数据有效信号
	output  wire [7:0]			o_r,				//输出R通道分量的值
	output  wire [7:0]			o_g,				//输出G通道分量的值
	output  wire [7:0]			o_b					//输出B通道分量的值
);

// ycbcr0(:,:,1)  =  0.2568*image_in_r + 0.5041*image_in_g + 0.0979*image_in_b + 16; 
// ycbcr0(:,:,2)  = -0.1482*image_in_r - 0.2910*image_in_g + 0.4392*image_in_b + 128;
// ycbcr0(:,:,3)  =  0.4392*image_in_r - 0.3678*image_in_g - 0.0714*image_in_b + 128;

// ycbcr0(:,:,1)  = 256*( 0.2568*image_in_r + 0.5041*image_in_g + 0.0979*image_in_b + 16 )>>8; 
// ycbcr0(:,:,2)  = 256*(-0.1482*image_in_r - 0.2910*image_in_g + 0.4392*image_in_b + 128)>>8;
// ycbcr0(:,:,3)  = 256*( 0.4392*image_in_r - 0.3678*image_in_g - 0.0714*image_in_b + 128)>>8;

// ycbcr0(:,:,1)  = (66*image_in_r + 129*image_in_g + 25*image_in_b + 4096  )>>8; 
// ycbcr0(:,:,2)  = (-38*image_in_r - 74*image_in_g + 112*image_in_b + 32768)>>8;
// ycbcr0(:,:,3)  = (112*image_in_r - 94*image_in_g - 18*image_in_b + 32768 )>>8;

reg [15:0] r_d0;		//寄存R通道分量中间计算结果
reg [15:0] g_d0;		//寄存G通道分量中间计算结果
reg [15:0] b_d0;		//寄存B通道分量中间计算结果
	
reg [15:0] r_d1;		//寄存R通道分量中间计算结果
reg [15:0] g_d1;		//寄存G通道分量中间计算结果
reg [15:0] b_d1;        //寄存B通道分量中间计算结果

reg [15:0] r_d2;		//寄存R通道分量中间计算结果
reg [15:0] g_d2;        //寄存G通道分量中间计算结果
reg [15:0] b_d2;        //寄存B通道分量中间计算结果

reg [15:0] y_d0;		//寄存Y通道分量中间计算结果
reg [15:0] cb_d0;		//寄存cb通道分量中间计算结果
reg [15:0] cr_d0;		//寄存cr通道分量中间计算结果

reg [7:0] y_d1;			//寄存Y通道分量中间计算结果
reg [7:0] cb_d1;        //寄存cb通道分量中间计算结果
reg [7:0] cr_d1;        //寄存cr通道分量中间计算结果

reg [7:0] skin_color_r;	//经过肤色识别后得到的R通道分量的值
reg [7:0] skin_color_g;	//经过肤色识别后得到的G通道分量的值
reg [7:0] skin_color_b; //经过肤色识别后得到的B通道分量的值

reg [3:0] hsyn;			//行同步信号寄存器
reg [3:0] vsyn;			//场同步信号寄存器
reg [3:0] de;			//数据有效信号同步寄存器


//根据RGB转YCbCr公式进行颜色空间的转换
always@(posedge i_clk or negedge i_rst_n) 
begin
    if(!i_rst_n)
	begin
        r_d0 <= 16'd0;
        g_d0 <= 16'd0;
        b_d0 <= 16'd0;
		r_d1 <= 16'd0;
		g_d1 <= 16'd0;
		b_d1 <= 16'd0;
		r_d2 <= 16'd0;
		g_d2 <= 16'd0;
		b_d2 <= 16'd0;	
    end
    else 
	begin
        r_d0 <= 66  * i_r;
        g_d0 <= 129 * i_g;
        b_d0 <= 25  * i_b;
		r_d1 <= 38  * i_r;
		g_d1 <= 74  * i_g;
		b_d1 <= 112 * i_b;
		r_d2 <= 112 * i_r;
		g_d2 <= 94  * i_g;
		b_d2 <= 18  * i_b;	        	
    end
end

//根据RGB转YCbCr公式进行颜色空间的转换
always@(posedge i_clk or negedge i_rst_n) 
begin
    if(!i_rst_n)
	begin
		y_d0  <= 16'd0;
		cb_d0 <= 16'd0;
		cr_d0 <= 16'd0;
    end
    else 
	begin
		y_d0  <= r_d0 + g_d0 + b_d0 + 4096 ;
		cb_d0 <= b_d1 - r_d1 - g_d1 + 32768;
		cr_d0 <= r_d2 - g_d2 - b_d2 + 32768;      	
    end
end

//根据RGB转YCbCr公式进行颜色空间的转换
always@(posedge i_clk or negedge i_rst_n) 
begin
    if(!i_rst_n)
	begin
		y_d1  <= 16'd0;
		cb_d1 <= 16'd0;
		cr_d1 <= 16'd0;
    end
    else 
	begin
		y_d1  <= y_d0[15:8];
		cb_d1 <= cb_d0[15:8];
		cr_d1 <= cr_d0[15:8];      	
    end
end

//通过判断cb和cr分量来进行图像肤色识别
always@(posedge i_clk or negedge i_rst_n) 
begin
    if(!i_rst_n)
	begin
		skin_color_r <= 8'd0;
		skin_color_g <= 8'd0;
		skin_color_b <= 8'd0;
    end
    else if(cb_d1 > 77 && cb_d1 < 127 && cr_d1 > 133 && cr_d1 < 173)
	begin
		skin_color_r <= 8'd255;
		skin_color_g <= 8'd255;
		skin_color_b <= 8'd255;     	
    end
    else 
	begin
		skin_color_r <= 8'd0;
		skin_color_g <= 8'd0;
		skin_color_b <= 8'd0;     	
    end	
end

//通过移位寄存器来进行行、场同步信号和数据有效信号的打拍
always@(posedge i_clk ) 
begin
	hsyn	<= {hsyn[2:0],i_hsyn};
	vsyn	<= {vsyn[2:0],i_vsyn};
	de		<= {de[2:0],i_de};
end

//输出打拍之后的行、场同步信号，数据有效信号和经过肤色识别算法处理的RGB三个通道的值
assign 	o_hs	= hsyn[3];
assign 	o_vs	= vsyn[3];
assign 	o_r		= skin_color_r;
assign 	o_g		= skin_color_g;
assign 	o_b 	= skin_color_b;
assign 	o_de	= de[3];

endmodule