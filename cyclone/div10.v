
`ifndef _div10_h
`define _div10_h

module div10 (
	input clk,
	input rst,
	output q);
	
	reg [9:0] count;
	initial count = 10'b0;
	
	always @ (posedge clk or negedge rst) begin
		if (~rst) begin
			count = 10'b0;
			end
		else begin
			count = count + 1;
			end
		end
		
	assign q = count[9];
	
	endmodule
	
module div10_norst (
	input clk,
	output q);
	
	reg [9:0] count;
	initial count = 10'b0;
	
	always @ (posedge clk) begin
		count = count + 1;
		end
		
	assign q = count[9];
	
	endmodule
	
module div4 (
	input clk,
	input rst,
	output q);
	
	reg [3:0] count;
	initial count = 4'b0;
	
	always @ (posedge clk or negedge rst) begin
		if (~rst) begin
			count = 4'b0;
			end
		else begin
			count = count + 1;
			end
		end
		
	assign q = count[3];
endmodule
	
`endif
	
