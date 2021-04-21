
`ifndef _7_seg_v
`define _7_seg_v


//converts a 4-bit binary value to display on a 7-segment display.
module _7_seg_driver (
	output [7:0] segments,
	input [3:0] value,
	input decimal);
	
	assign segments[7] = ~decimal; //a separate input to drive the decimal point high.
	
	assign segments[6:0] = 
		(value == 4'b0000) ? ~(7'b011_1111): //swapped
		(value == 4'b0001) ? ~(7'b000_0110): //swapped
		(value == 4'b0010) ? ~(7'b101_1011): //110_1101
		(value == 4'b0011) ? ~(7'b100_1111): //111_1001
		(value == 4'b0100) ? ~(7'b110_0110): //011_0011
		(value == 4'b0101) ? ~(7'b110_1101): //101_1011
		(value == 4'b0110) ? ~(7'b111_1101): //101_1111
		(value == 4'b0111) ? ~(7'b000_0111): //111_0000
		(value == 4'b1000) ? ~(7'b111_11111): //111_1111
		(value == 4'b1001) ? ~(7'b110_0111): //111_0011
		(value == 4'b1010) ? ~(7'b111_0111): //111_0111
		(value == 4'b1011) ? ~(7'b111_1100): //001_1111
		(value == 4'b1100) ? ~(7'b011_1001): //100_1110
		(value == 4'b1101) ? ~(7'b101_1110): //011_1101
		(value == 4'b1110) ? ~(7'b111_1001): //100_1111
		(value == 4'b1111) ? ~(7'b111_0001): //100_0111
		7'bXXX_XXXX;
		
endmodule


//This should only update value when en is pulsed.
module _7_seg_w_en (
	output [7:0] segments,
	input [3:0] value,
	input decimal,
	input en);
	
	reg [6:0] seg_buf;
	assign segments[7] = ~decimal; //a separate input to drive the decimal point high.
	assign segments[6:0] = seg_buf;
	
	always @ (posedge en) begin
		seg_buf = 
			(value == 4'b0000) ? ~(7'b011_1111): //swapped
			(value == 4'b0001) ? ~(7'b000_0110): //swapped
			(value == 4'b0010) ? ~(7'b101_1011): //110_1101
			(value == 4'b0011) ? ~(7'b100_1111): //111_1001
			(value == 4'b0100) ? ~(7'b110_0110): //011_0011
			(value == 4'b0101) ? ~(7'b110_1101): //101_1011
			(value == 4'b0110) ? ~(7'b111_1101): //101_1111
			(value == 4'b0111) ? ~(7'b000_0111): //111_0000
			(value == 4'b1000) ? ~(7'b111_11111): //111_1111
			(value == 4'b1001) ? ~(7'b110_0111): //111_0011
			(value == 4'b1010) ? ~(7'b111_0111): //111_0111
			(value == 4'b1011) ? ~(7'b111_1100): //001_1111
			(value == 4'b1100) ? ~(7'b011_1001): //100_1110
			(value == 4'b1101) ? ~(7'b101_1110): //011_1101
			(value == 4'b1110) ? ~(7'b111_1001): //100_1111
			(value == 4'b1111) ? ~(7'b111_0001): //100_0111
			7'bXXX_XXXX;
		end
		
endmodule

/*
Module prints backwards, also the 0 and 6 positions are swapped.

module _7_seg_driver (
	output [7:0] segments,
	input [3:0] value,
	input decimal);
	
	assign segments[7] = ~decimal; //a separate input to drive the decimal point high.
	
	assign segments[6:0] = 
		(value == 4'b0000) ? ~(7'b111_1110):
		(value == 4'b0001) ? ~(7'b011_0000):
		(value == 4'b0010) ? ~(7'b110_1101):
		(value == 4'b0011) ? ~(7'b111_1001):
		(value == 4'b0100) ? ~(7'b011_0011):
		(value == 4'b0101) ? ~(7'b101_1011):
		(value == 4'b0110) ? ~(7'b101_1111):
		(value == 4'b0111) ? ~(7'b111_0000):
		(value == 4'b1000) ? ~(7'b111_1111):
		(value == 4'b1001) ? ~(7'b111_0011):
		(value == 4'b1010) ? ~(7'b111_0111):
		(value == 4'b1011) ? ~(7'b001_1111):
		(value == 4'b1100) ? ~(7'b100_1110):
		(value == 4'b1101) ? ~(7'b011_1101):
		(value == 4'b1110) ? ~(7'b100_1111):
		(value == 4'b1111) ? ~(7'b100_0111):
		7'bXXX_XXXX;
		
endmodule
*/
	
	
	`endif
	