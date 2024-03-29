module testbench3;

logic clk, rst, out_clk;

timeunit 1ns;
timeprecision 1ps;

cd3 dut(clk,rst,out_clk);

initial
	begin
		rst = 0;
		clk = 0;
		#10 rst = 1;
	end
	
always

	begin
		#5 clk = 1;
		#5 clk = 0;
	end
	
endmodule
