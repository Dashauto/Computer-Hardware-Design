module testbench;

logic clk, reset,left, right, la, lb, lc, ra, rb, rc;

timeunit 1ns;
timeprecision 1ps;

lab3_b dut(clk, reset,left, right, la, lb, lc, ra, rb, rc);
 
initial
	begin
		reset = 0;
		clk = 0;
		left = 0;
		right = 0;
		#10 reset = 1;
		#10 left = 1;
		#35 left = 0;
		#10 right = 1;
		#35 right = 0;
		#10 left = 1;
		#35 left = 0;
		#10 right = 1;
		#35 right = 0;
		
	end
	
always

	begin
		#5 clk = 1;
		#5 clk = 0;
	end
	
endmodule
