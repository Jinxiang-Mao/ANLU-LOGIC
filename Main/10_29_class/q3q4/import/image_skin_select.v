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

//对识别到的图像肤色进行框选
module image_skin_select
(
	input   wire				i_clk,				//输入时钟信号	
	input   wire				i_rst_n,			//输入复位信号

	input	wire				i_hsyn,				//输入行同步信号
	input	wire				i_vsyn,				//输入场同步信号
	input	wire				i_de,				//输入数据有效信号
	input	wire [7:0]			i_r,				//输入经过肤色识别处理之后R通道分量的值
	input	wire [7:0] 			i_g,				//输入经过肤色识别处理之后G通道分量的值
	input	wire [7:0] 			i_b,				//输入经过肤色识别处理之后B通道分量的值
	input	wire [7:0]			i_r_original,		//输入原始图像R通道分量的值
	input	wire [7:0] 			i_g_original,		//输入原始图像G通道分量的值
	input	wire [7:0] 			i_b_original,		//输入原始图像B通道分量的值
	input	wire [10:0]			i_x_pos,			//输入图像的X坐标
	input	wire [10:0] 			i_y_pos,			//输入图像的Y坐标
	output  reg  [7:0]			o_r,				//输出经过处理之后的R通道分量的值
	output  reg  [7:0]			o_g,				//输出经过处理之后的G通道分量的值
	output  reg  [7:0]			o_b				//输出经过处理之后的B通道分量的值
);
parameter  H_ACTIVE 		= 1920; 				//显示区域宽度                              
parameter  V_ACTIVE 		= 1080;  				//显示区域高度
parameter  R_VALUE			= 8'd255;				//显示选框颜色
parameter  G_VALUE			= 8'h00;				//显示选框颜色
parameter  B_VALUE			= 8'h00;				//显示选框颜色

reg  [10:0] 	    h_cnt;							//显示区域行计数器
reg  [10:0] 	    v_cnt;							//显示区域场计数器
reg 				i_vsyn_d;						//场同步信号打拍寄存器

reg  [10:0] 	    skin_h_min;						//肤色图像行位置最小值临时结果
reg  [10:0] 	    skin_h_max;						//肤色图像行位置最大值临时结果
reg  [10:0] 	    skin_v_min;						//肤色图像场位置最小值临时结果
reg  [10:0] 	    skin_v_max;						//肤色图像场位置最大值临时结果

reg  [10:0] 	    skin_h_min_d;					//肤色图像行位置最小值最终结果
reg  [10:0] 	    skin_h_max_d;                   //肤色图像行位置最大值最终结果
reg  [10:0] 	    skin_v_min_d;                   //肤色图像场位置最小值最终结果
reg  [10:0] 	    skin_v_max_d;                   //肤色图像场位置最大值最终结果

//寄存一次场同步信号，用于判断场同步信号的边沿信息
always@(posedge i_clk ) 
begin
	i_vsyn_d <= i_vsyn;
end

//显示区域行计数
always@(posedge i_clk or negedge i_rst_n) 
begin
    if(!i_rst_n)
	begin
        h_cnt <= 11'd0;
    end
    else if(i_de)
	begin
		if(h_cnt == H_ACTIVE - 1'b1)
			h_cnt <= 11'd0;
		else 
			h_cnt <= h_cnt + 11'd1;
    end
end

//显示区域场计数
always@(posedge i_clk or negedge i_rst_n) 
begin
    if(!i_rst_n)
	begin
        v_cnt <= 11'd0;
    end
    else if(h_cnt == H_ACTIVE - 1'b1)
	begin
		if(v_cnt == V_ACTIVE - 1'b1)
			v_cnt <= 11'd0;
		else 
			v_cnt <= v_cnt + 11'd1;
    end
end

//查找肤色行位置最小值
always@(posedge i_clk or negedge i_rst_n) 
begin
    if(!i_rst_n)
	begin
		skin_h_min  <= H_ACTIVE;
    end
    else if(~i_vsyn_d & i_vsyn)
	begin
    	skin_h_min  <= H_ACTIVE;
    end	
    else if(i_r == 8'hff && i_g == 8'hff && i_b == 8'hff && skin_h_min > h_cnt) 
	begin
    	skin_h_min  <= h_cnt;
    end
end

//查找肤色行位置最大值
always@(posedge i_clk or negedge i_rst_n) 
begin
    if(!i_rst_n)
	begin
		skin_h_max  <= 8'h00;
    end
    else if(~i_vsyn_d & i_vsyn)
	begin
    	skin_h_max  <= 8'h00;
    end	
    else if(i_r == 8'hff && i_g == 8'hff && i_b == 8'hff && skin_h_max < h_cnt) 
	begin
    	skin_h_max  <= h_cnt;
    end
end

//查找肤色场位置最小值
always@(posedge i_clk or negedge i_rst_n) 
begin
    if(!i_rst_n)
	begin
		skin_v_min  <= V_ACTIVE;
    end
    else if(~i_vsyn_d & i_vsyn)
	begin
    	skin_v_min  <= V_ACTIVE;
    end	
    else if(i_r == 8'hff && i_g == 8'hff && i_b == 8'hff && skin_v_min > v_cnt) 
	begin
    	skin_v_min  <= v_cnt;
    end
end

//查找肤色场位置最大值
always@(posedge i_clk or negedge i_rst_n) 
begin
    if(!i_rst_n)
	begin
		skin_v_max  <= 8'h00;
    end
    else if(~i_vsyn_d & i_vsyn)
	begin
    	skin_v_max  <= 8'h00;
    end	
    else if(i_r == 8'hff && i_g == 8'hff && i_b == 8'hff && skin_v_max < v_cnt) 
	begin
    	skin_v_max  <= v_cnt;
    end
end

//在场同步信号的下降沿，也就是一帧图像结束的时候，输出肤色的位置信息
always@(posedge i_clk or negedge i_rst_n) 
begin
    if(!i_rst_n)
	begin
		skin_h_min_d <= 'd0;
		skin_h_max_d <= 'd0;
		skin_v_min_d <= 'd0;
		skin_v_max_d <= 'd0;
    end
    else if(~i_vsyn & i_vsyn_d)
	begin
		skin_h_min_d <= skin_h_min;
		skin_h_max_d <= skin_h_max;
		skin_v_min_d <= skin_v_min;
		skin_v_max_d <= skin_v_max;   	
    end
end

//对肤色的位置信息进行框选
always@(posedge i_clk or negedge i_rst_n) 
begin
    if(!i_rst_n)
	begin
		o_r <= 'd0;
		o_g <= 'd0;
		o_b <= 'd0;
    end
    // else if((i_x_pos == skin_h_min_d && i_y_pos >= skin_v_min_d && i_y_pos <= skin_v_max_d)||
			// (i_x_pos == skin_h_max_d && i_y_pos >= skin_v_min_d && i_y_pos <= skin_v_max_d)||
			// (i_y_pos == skin_v_min_d && i_x_pos >= skin_h_min_d && i_x_pos <= skin_h_max_d)||
			// (i_y_pos == skin_v_max_d && i_x_pos >= skin_h_min_d && i_x_pos <= skin_h_max_d))
    else if(((i_x_pos == skin_h_min_d || i_x_pos == skin_h_max_d ) && i_y_pos >= skin_v_min_d && i_y_pos <= skin_v_max_d)||
			((i_y_pos == skin_v_min_d || i_y_pos == skin_v_max_d ) && i_x_pos >= skin_h_min_d && i_x_pos <= skin_h_max_d))			
	begin
		o_r <= R_VALUE;
		o_g <= G_VALUE;
		o_b <= B_VALUE;	
    end
    else 
	begin
		o_r <= i_r_original;
		o_g <= i_g_original;
		o_b <= i_b_original;	
    end	
end

endmodule