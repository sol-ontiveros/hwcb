
`ifndef _UART_V
`define _UART_V

/*
Module for controlling the TL16C55 UART ACE IC.
Clock input should match the clock to the UART bridge device.
*/


//Register addresses, from datasheet page 32.
`define RXTX_REG 			3'b000
`define FIFO_CTRL_REG 	3'b010
`define LINE_CTRL_REG	3'b011
`define MDM_CTRL_REG		3'b100
`define LINE_STS_REG		3'b101


module uart (
	inout [7:0] parallel_port,
	output write_enable, //active low
	output [3:0] address, //bit 3 is DLAB
	output ADS, //address strobe, active low.
	
	input clk,
	//input read_ready, //active low
	input EN, //active high
	input data_in);
	
	reg [7:0] data_n;
	reg writen_n;
	
	assign write_enable = ~writen_n;
	assign address[3] = 1'b0; //drive DLAB low - we won't use it.
	
	//The timing of the enable vs data out may need to be adjusted.
	always @ (posedge clk) begin
		if (write_n==1) begin
			write_n = 0;
			end
		if (EN==1 && ADS==0) begin
			//shift data to the output port
			data_n = data_in;
			//select transmit/recieve reg and strobe ADS
			address[2:0] = RXTX_REG;
			ADS = 1;
			end
		else if (EN==1) begin
			ADS <= 0;
			write_n <= 1; //drive write_enable low to start data load tx FIFO
			end
		end
endmodule
			
`endif	