

`ifndef _PCA9564_V
`define _PCA9564_V

	

/*
Device interface module for the NXP pca9564 parralel to i2c IC.

This module handles the basic hardware interface, and is intended to be used in combination
with an data flow control module.
The module will automatically configure the device to the settings in INIT_REG.
*/





module pca9564 (
	output [7:0] data_out,
	output [1:0] addr,
	output write_strobe,
	output read_strobe,
	output busy,
	output byte_strobe,
	input [7:0] data_in,
	input [7:0] data_len,
	input write_enable, //active low
	input interrupt_req,
	input clk );
	
	
	//PCA9564 device register addresses
	parameter STATUS_REG =	2'b00; //read-only
	parameter TIMEOUT_REG =	2'b00; //write-only
	parameter DATA_REG =		2'b01;
	parameter OWN_ADDR_REG= 2'b10;
	parameter CTRL_REG = 	2'b11;

	parameter START_BIT = 8'b00100000;
	parameter STOP_BIT =  8'b00010000;

	// device configuration for the pca9564
	parameter INIT_REG =	 8'b11000100;
	
	
	reg periph_config_flag = 1'b0; //flag that indicates if I2C device has been configured.
	wire device_init_flag; //flag goes high once device initialization period is complete.
	reg [9:0] device_init_counter = 10'b0;
	
	//reg busy_n = 1'b0;
	reg tx_started_n = 1'b1;
	reg ws_n = 1'b1; //active low output.
	reg rs_n = 1'b1; //active low output.
	reg bs_n = 1'b0;
	reg [7:0] data_n;
	reg [7:0] data_len_n;
	reg [1:0] addr_n;
	
	//transmit flags.
	reg start_sent = 1'b0;
	reg data_sent = 1'b0;
	
	//assign busy = (tx_started_n && busy_n);
	assign busy = tx_started_n;
	//TODO: Adjust this to the appropriate timing.
	assign device_init_flag = device_init_counter[2]; //flag goes high after counter 
	assign write_strobe = ws_n;
	assign read_strobe = rs_n;
	assign byte_strobe = bs_n;
	assign data_out = data_n;
	assign addr = addr_n;
	
	

	always @ (posedge clk) begin
		//initialization/configuration loop
		//tx_started_n = ~(device_init_flag); //set busy flag until init is done.
		if (!device_init_flag) begin
			ws_n = 1'b1;
			//send configuration data
			if (!periph_config_flag) begin
				addr_n = CTRL_REG;
				data_n = INIT_REG;
				ws_n = 1'b0; // pulse strobe low to write data to specified reg.
				periph_config_flag = 1'b1;
				end
			//wait for device init period (500us)
			//once device_init_counter[MSb] goes high, this loop will break.
			else begin
				device_init_counter = (device_init_counter + 1'b1);
				tx_started_n = ~device_init_flag;
				end
			end
		
		//with initialization done, begin main tx function.
		else if (device_init_flag) begin // only allow tx after device has been initialized.
			//clear strobes if they have been used.
			ws_n = 1'b1;
			bs_n = 1'b0;
			
			//begin a new write if requested and last write is complete.
			if ((!write_enable) && (!tx_started_n)) begin //write_enable is active low.
				data_len_n = data_len;
				start_sent = 1'b0;
				data_sent = 1'b0;
				tx_started_n = 1'b1;
				end
				
			//once start has been requested, begin write routine.
			else if (tx_started_n) begin
				//ws_n = 1'b1;
				//send a start condition first
				if(!start_sent) begin
					addr_n = CTRL_REG;
					data_n = (INIT_REG | START_BIT);
					ws_n = 1'b0;
					start_sent = 1'b1;
					end
					
				//send data from data array.
				else if (!data_sent) begin
					addr_n = DATA_REG;
					if(data_len_n > 0) begin
						data_n = data_in; //set output to current byte.
						//bs_n = 1'b1; //strobe bs to advance to next byte (read on next loop)
						ws_n = 1'b0; //strobe ws to write byte to data reg.
						data_len_n = (data_len_n - 1'b1); //countdown indicated message length
						end
					//once all data is sent, set proper flag.
					else begin
						data_sent = 1'b1;
						end
					end
				
				//send stop condition to finish transmission.
				else if (start_sent && data_sent) begin
					addr_n = CTRL_REG;
					data_n = (INIT_REG | STOP_BIT);
					ws_n = 1'b0;
					tx_started_n = 1'b0;
					end	
				end
			end
		end
endmodule


`endif