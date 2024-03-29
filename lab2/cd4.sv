module cd4 (input logic clk , input logic rst, output logic out_clk );

logic D, Q, d, q = 0;

always @(posedge clk)

begin
if (~rst)
     Q <= 1'b0;
else
     Q <= D;	
end

assign D = ~Q;

always @(posedge Q)

begin
if (~rst)
	q <= 1'b0;
else
	q <= d;
	
end

assign d = ~q;
assign out_clk = q;

endmodule
