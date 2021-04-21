
`ifndef _counter_vh
`define _counter_vh

/*Assorted counters. 
Naming convention is [counter type]_counter[number of bits].*/

module counter3 (
	input inc_n,
	input rst,
	output [2:0] count);
	
	reg [2:0] count_n;
	initial count_n = 3'b0;
	assign count = count_n;
	
	always @ (posedge inc_n or negedge rst) begin
		if(~rst) begin
			count_n = 3'b0;
			end
		else begin
			count_n = count_n + 1;
			end
		end
endmodule


module counter4 (
	input inc_n,
	input rst,
	output [3:0] count);
	
	reg [3:0] count_n;
	initial count_n = 4'b0;
	assign count = count_n;
	
	always @ (posedge inc_n or negedge rst) begin
		if(~rst) begin
			count_n = 4'b0;
			end
		else begin
			count_n = count_n + 1;
			end
		end
endmodule


module counter8 (
	input inc_n,
	input rst,
	output [7:0] count);
	
	reg [7:0] count_n;
	initial count_n = 8'b0;
	assign count = count_n;
	
	always @ (posedge inc_n or negedge rst) begin
		if(~rst) begin
			count_n = 8'b0;
			end
		else begin
			count_n = count_n + 1;
			end
		end
endmodule

`endif
