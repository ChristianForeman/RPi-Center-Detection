`timescale 1ns / 1ps

module projector(output VGA_SYNC_N, output VGA_BLANK_N, input [4:0]GPIO, input CLOCK_50, output reg VGA_HS, output reg VGA_VS,	output VGA_CLK, output [7:0]VGA_R, output [7:0]VGA_B, output [7:0]VGA_G);
	//GPIO[0] is ab22, GPIO[2] is ab21, GPIO[4] is ac21

	//1) Divide Clock from 50 MHz to 25 MHz
	reg reset = 0;  // for PLL
	wire clk25MHz;
	ip ip1(
		.areset(reset),
		.inclk0(CLOCK_50),
		.c0(clk25MHz),
		.locked()
		);  
		
	//2) counter and sync generation
	reg [9:0] counter_x = 0;  // horizontal counter
	reg [9:0] counter_y = 0;  // vertical counter
	always @(posedge clk25MHz)  // horizontal counter
		begin 
			if (counter_x < 799)
				counter_x <= counter_x + 1;  // horizontal counter (including off-screen horizontal 160 pixels) total of 800 pixels 
			else
				counter_x <= 0;              
		end  // always 
	
	always @ (posedge clk25MHz)  // vertical counter
		begin 
			if (counter_x == 799)  // only counts up 1 count after horizontal finishes 800 counts
				begin
					if (counter_y < 525)  // vertical counter (including off-screen vertical 45 pixels) total of 525 pixels
						counter_y <= counter_y + 1;
					else
						counter_y <= 0;              
				end 
		end 
		
		always @(posedge clk25MHz)
			begin
				VGA_HS <= (counter_x >= 0 && counter_x < 96) ? 1:0;  // hsync high for 96 counts of counterx                                               
				VGA_VS <= (counter_y >= 0 && counter_y < 2) ? 1:0;   // vsync high for 2 counts of countery
			end
			
	assign VGA_CLK = clk25MHz; //four our DAC
		
	//3) Run designs based on GPIOs
	reg [7:0] r_red;
	reg [7:0] r_blue;
	reg [7:0] r_green;

	always @ (posedge clk25MHz) begin
		if (GPIO[0] == 0 && GPIO[2] == 0 && GPIO[4] == 0) begin //VERTICAL
			if (counter_x < 244)
				begin              
					r_red <= 8'hFF;    // yellow
					r_blue <= 8'h00;
					r_green <= 8'hFF;
				end 
			else if (counter_x >= 244 && counter_x < 344)
				begin              
					r_red <= 8'hFF;    // red
					r_blue <= 8'h00;
					r_green <= 8'h00;
				end 
			else if (counter_x >= 344 && counter_x < 444)
				begin              
					r_red <= 8'h00;    // blue
					r_blue <= 8'hFF;
					r_green <= 8'h00;
				end 
			else if (counter_x >= 444 && counter_x < 544)
				begin              
					r_red <= 8'h00;    // green
					r_blue <= 8'h00;
					r_green <= 8'hFF;
				end 
			else if(counter_x >= 544 && counter_x < 644)
				begin              
					r_red <= 8'hFF;    // white
					r_blue <= 8'hFF;
					r_green <= 8'hFF;
				end 
				else if(counter_x >= 644)
					begin              
						r_red <= 8'hFF;    // pink
						r_blue <= 8'hFF;
						r_green <= 8'h00;
					end		
			end
			
			else if (GPIO[0] == 1 && GPIO[2] == 1 && GPIO[4] == 1) begin //HORIZONTAL
				if (counter_y < 135)
					begin              
						r_red <= 8'hFF;    // yellow
						r_blue <= 8'h00;
						r_green <= 8'hFF;
					end 
				else if (counter_y >= 135 && counter_y < 235)
					begin              
						r_red <= 8'hFF;    // red
						r_blue <= 8'h00;
						r_green <= 8'h00;
					end 
				else if (counter_y >= 235 && counter_y < 335)
				begin              
					r_red <= 8'h00;    // blue
					r_blue <= 8'hFF;
					r_green <= 8'h00;
				end 
				else if (counter_y >= 335 && counter_y <= 435)
					begin              
						r_red <= 8'h00;    // green
						r_blue <= 8'h00;
						r_green <= 8'hFF;
					end 
				else if(counter_y >= 435)
					begin              
						r_red <= 8'hFF;    // white
						r_blue <= 8'hFF;
						r_green <= 8'hFF;
					end
			end
			if (GPIO[0] == 0 && GPIO[2] == 1 && GPIO[4] == 0) begin //SMILEY
				if (counter_y < 135)
				begin              
					r_red <= 8'hFF;    // white
					r_blue <= 8'hFF;
					r_green <= 8'hFF;
				end  // if (counter_y < 135)

			else if (counter_y >= 135 && counter_y < 205)
				begin 
					if (counter_x < 324)
						begin 
							r_red <= 8'hFF;    // white
							r_blue <= 8'hFF;
							r_green <= 8'hFF;
						end  // if (counter_x < 324)
					else if (counter_x >= 324 && counter_x < 604)
						begin 
							r_red <= 8'hFF;    // yellow
							r_blue <= 8'h00;
							r_green <= 8'hFF;
						end  // else if (counter_x >= 324 && counter_x < 604)
					else if (counter_x >= 604)
						begin 
							r_red <= 8'hFF;    // white
							r_blue <= 8'hFF;
							r_green <= 8'hFF;
						end  // else if (counter_x >= 604)
					end  // else if (counter_y >= 135 && counter_y < 205)
			else if (counter_y >= 205 && counter_y < 217)
				begin 
					if (counter_x < 324)
						begin 
							r_red <= 8'hFF;    // white
							r_blue <= 8'hFF;
							r_green <= 8'hFF;
						end  // if (counter_x < 324)
					else if (counter_x >= 324 && counter_x < 371)
						begin 
							r_red <= 8'hFF;    // yellow
							r_blue <= 8'h00;
							r_green <= 8'hFF;
						end  // else if (counter_x >= 324 && counter_x < 371)
					else if (counter_x >= 371 && counter_x < 383)
						begin 
							r_red <= 8'h00;    // black
							r_blue <= 8'h00;
							r_green <= 8'h00;
						end  // else if (counter_x >= 371 && counter_x < 383)
					else if (counter_x >= 383 && counter_x < 545)
						begin 
							r_red <= 8'hFF;    // yellow
							r_blue <= 8'h00;
							r_green <= 8'hFF;
						end  // else if (counter_x >= 383 && counter_x < 545)
					else if (counter_x >= 545 && counter_x < 557)
						begin 
							r_red <= 8'h00;    // black
							r_blue <= 8'h00;
							r_green <= 8'h00;
						end  // else if (counter_x >= 545 && counter_x < 557)
					else if (counter_x >= 557 && counter_x < 604)
						begin 
							r_red <= 8'hFF;    // yellow
							r_blue <= 8'h00;
							r_green <= 8'hFF;
						end  // else if (counter_x >= 557 && counter_x < 604)
					else if (counter_x >= 604)
						begin 
							r_red <= 8'hFF;    // white
							r_blue <= 8'hFF;
							r_green <= 8'hFF;
						end  // else if (counter_x >= 604)
				end  // else if (counter_y >= 205 && counter_y < 217)
			else if (counter_y >= 217 && counter_y < 305)
				begin
					if (counter_x < 324)
						begin 
							r_red <= 8'hFF;    // white
							r_blue <= 8'hFF;
							r_green <= 8'hFF;
						end  // if (counter_x < 324)
					else if (counter_x >= 324 && counter_x < 604)
						begin 
							r_red <= 8'hFF;    // yellow
							r_blue <= 8'h00;
							r_green <= 8'hFF;
						end  // else if (counter_x >= 324 && counter_x < 604)
					else if (counter_x >= 604)
						begin 
							r_red <= 8'hFF;    // white
							r_blue <= 8'hFF;
							r_green <= 8'hFF;
						end  // else if (counter_x >= 604)	
				end  // else if (counter_y >= 217 && counter_y < 305)
			else if (counter_y >= 305 && counter_y < 310)
				begin
					if (counter_x < 324)
						begin 
							r_red <= 8'hFF;    // white
							r_blue <= 8'hFF;
							r_green <= 8'hFF;
						end  // if (counter_x < 324)
					else if (counter_x >= 324 && counter_x < 371)
						begin 
							r_red <= 8'hFF;    // yellow
							r_blue <= 8'h00;
							r_green <= 8'hFF;
						end  // else if (counter_x >= 324 && counter_x < 371)
					else if (counter_x >= 371 && counter_x < 557)
						begin 
							r_red <= 8'h00;    // black
							r_blue <= 8'h00;
							r_green <= 8'h00;
						end  // else if (counter_x >= 371 && counter_x < 557)
					else if (counter_x >= 557 && counter_x < 604)
						begin 
							r_red <= 8'hFF;    // yellow
							r_blue <= 8'h00;
							r_green <= 8'hFF;
						end  // else if (counter_x >= 557 && counter_x < 604)
					else if (counter_x >= 604)
						begin 
							r_red <= 8'hFF;    // white
							r_blue <= 8'hFF;
							r_green <= 8'hFF;
						end  // else if (counter_x >= 604)	
				end  // else if (counter_y >= 217 && counter_y < 305)
			else if (counter_y >= 305 && counter_y < 414)
				begin
					if (counter_x < 324)
						begin 
							r_red <= 8'hFF;    // white
							r_blue <= 8'hFF;
							r_green <= 8'hFF;
						end  // if (counter_x < 324)
					else if (counter_x >= 324 && counter_x < 604)
						begin 
							r_red <= 8'hFF;    // yellow
							r_blue <= 8'h00;
							r_green <= 8'hFF;
						end  // else if (counter_x >= 324 && counter_x < 604)
					else if (counter_x >= 604)
						begin 
							r_red <= 8'hFF;    // white
							r_blue <= 8'hFF;
							r_green <= 8'hFF;
						end  // else if (counter_x >= 604)	
				end  // else if (counter_y >= 305 && counter_y < 414)
			else if (counter_y <= 414)
				begin              
					r_red <= 8'hFF;    // white
					r_blue <= 8'hFF;
					r_green <= 8'hFF;
				end  // if (counter_y >= 414)
			end
		end

	assign VGA_R = (counter_x > 144 && counter_x <= 783 && counter_y > 35 && counter_y <= 514) ? r_red : 8'h00;
	assign VGA_B = (counter_x > 144 && counter_x <= 783 && counter_y > 35 && counter_y <= 514) ? r_blue : 8'h00;
	assign VGA_G = (counter_x > 144 && counter_x <= 783 && counter_y > 35 && counter_y <= 514) ? r_green : 8'h00;
	assign VGA_SYNC_N = 1'b0;
	assign VGA_BLANK_N = (counter_x > 144 && counter_x <= 783 && counter_y > 35 && counter_y <= 514) ? 1: 0;
endmodule