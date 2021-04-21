
`ifndef _onehot_v
`define _onehot_v


module onehot_3_8 (
	output [7:0] decoded,
	input [2:0] encoded);
	
	assign decoded[0] = (~encoded[0] & ~encoded[1] & ~encoded[2]);
	assign decoded[1] = (encoded[0] & ~encoded[1] & ~encoded[2]);
	assign decoded[2] = (~encoded[0] & encoded[1] & ~encoded[2]);
	assign decoded[3] = (encoded[0] & encoded[1] & ~encoded[2]);
	assign decoded[4] = (~encoded[0] & ~encoded[1] & encoded[2]);
	assign decoded[5] = (encoded[0] & ~encoded[1] & encoded[2]);
	assign decoded[6] = (~encoded[0] & encoded[1] & encoded[2]);
	assign decoded[7] = (encoded[0] & encoded[1] & encoded[2]);
endmodule


module onehot_2to4 (
	input [1:0] encoded,
	output [3:0] decoded);
	
	assign decoded[0] = (~encoded[0] & ~encoded[1]);
	assign decoded[1] = (encoded[0] & ~encoded[1]);
	assign decoded[2] = (~encoded[0] & encoded[1]);
	assign decoded[3] = (encoded[0] & encoded[1]);
endmodule

	
	
`endif