
`ifndef _DATA_REGISTERS_V
`define _DATA_REGISTERS_V

/*
addressable data registers for peripheral logic output -> I2C controller module
*/

module data_registers (
	output [7:0] data_out,
	//input clk,
	input [3:0] addr,
	input [7:0] adc1,
	input [7:0] adc2,
	input [7:0] adc3);
	
	wire [7:0] holding_registers [0:3];
	reg [7:0] ID = 8'hAA; //meaningless value.
	//reg [7:0] data_n;
	//assign data_out = data_n;
	
	
	assign holding_registers[4'h0] = ID;
	assign holding_registers[4'h1] = adc1;
	assign holding_registers[4'h2] = adc2;
	assign holding_registers[4'h3] = adc3;
	
	assign data_out = holding_registers[addr];
	/*
	always @ (posedge clk) begin
		data_n = holding_registers[addr];
		end
		*/
endmodule


`endif
