
`ifndef _adc_data_v
`define _adc_data_v

//Data read module for ADS7808 ADC.

/*TODO:
This module will need extra testing to verify the validity of the data being read.
As of yet, I haven't tested for noise, and I'm not quite happy with the dumb way it reads data in (i.e.
just read the next 12 bits regardless of what is happening on the data line).
*/

module adc_data (
	output reg [11:0] data_out,
	output read_complete, //active high
	output RC, //active low
	input clk,
	input enable,
	input data_in);
	
	reg [3:0] bit_count;
	reg read_started;
	reg [11:0] data_buf;
		
	//output registers
	assign read_complete = ~read_started;
	reg rc_n;
	assign RC = rc_n;
	//assign data_out = data_buf;
	
	initial begin
		bit_count = 0;
		read_started = 0;
		data_buf = 0;
		rc_n = 1'b0;
		end
	
	//always update data output ever time a conversion starts and finishes, but not during.
	//this should avoid output of partial/corrupted data.
	always @ (read_started) begin
		data_out = data_buf;
		end
	
	always @ (posedge clk) begin
		//start a read by driving RC low, if one is not already in progress. 
		if ((enable) && (!read_started)) begin
			rc_n <= 1'b1; //drive R/C low to start conversion
			read_started <= 1'b1; //set read_started flag high so we know we're in the middle of a conversion.
			end
		//if a read has started, monitor inputs for new data.
		else if (read_started==1'b1) begin
			//release R/C to initialize the data transfer from the ADC
			if (rc_n==1'b1) begin
				rc_n <= 1'b0;
				end
			//delay two clock pulse to wait for sync to happen
			//TODO: this may or may not be timed exactly right.
			else if (bit_count < 2) begin
				bit_count <= bit_count + 1;
				end
			//begin to read data into buffer
			else if ((bit_count < 4'd14) && (bit_count > 4'd1)) begin
				data_buf[4'd11 - (bit_count - 4'd2)] <= ~data_in; //data inverted by txvr/buffer - invert it back.
				bit_count <= bit_count +1;
				end
			//if the read is complete, reset flag and counter.
			else begin
				read_started <= 1'b0;
				bit_count <= 4'd0;
				end
			end
		end
endmodule


/*
This module performs all the signal ANDing and syncing operations to run the 3 adcs in parallel.
It also truncates the reads to the 8 MSB for each channel.
*/
module adc_sync(
	output [7:0] adc0_data, //truncated adc data output channels
	output [7:0] adc1_data,
	output [7:0] adc2_data,
	output RC, //adc_data uses this to initialize a read, assuming it is not busy.
	output adc_clk, //adc ICs use an external clock signal.
	output read_complete, //used to indicate the module is ready to perform another read.
	input clk,
	input adc0_in,
	input adc1_in,
	input adc2_in,
	input enable); //active high
	
	wire [11:0] data0;
	wire [11:0] data1;
	wire [11:0] data2;
	assign adc0_data = data0[11:4];
	assign adc1_data = data1[11:4];
	assign adc2_data = data2[11:4];
	
	//synchronize adc_read_complete signals
	wire [2:0] adc_read_complete;
	assign read_complete = (adc_read_complete[0] & adc_read_complete[1] & adc_read_complete[2]);
	
	//synchonize RC signals so we don't get multiple enables per read.
	//TODO: could remove this functionality from adc_data and implement here.
	wire [2:0] rc_bus;
	assign RC = (rc_bus[0] & rc_bus[1] & rc_bus[2]);
	
	//pass clock input through to adc clock pin.
	assign adc_clk = clk;
	
	//all adc enable lines are synchronized. Read completes will be ANDed together.
	adc_data ADC1(data0, adc_read_complete[0], rc_bus[0], clk, enable, adc0_in);
	adc_data ADC2(data1, adc_read_complete[1], rc_bus[1], clk, enable, adc1_in);
	adc_data ADC3(data2, adc_read_complete[2], rc_bus[2], clk, enable, adc2_in);
endmodule

`endif
		