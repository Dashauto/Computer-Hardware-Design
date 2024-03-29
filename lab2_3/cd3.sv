/*
module cd3(input logic clk, input logic rst, output logic out_clk);

logic clock_P=0,clock_N=0;
int P=0,N=0; //Positive and Negative edge trigger
logic result; //results stored varibale

//count positive edge
always_ff @(posedge clk) 
begin
if (~rst)
	P <= 0;
else
	if (P == 2)
		P <= 0;
	else
		P <= P + 1;
end
//count negative edge
always_ff @(negedge clk) 
begin
if (~rst)
	N <= 0;
else
	if (N == 2) 
		N <= 0;
	else
		N <= N + 1;
end   
//The next clk will be based on positive edge count
//if 0 or 1 change value; if 2 hold value
always_ff @(posedge clk)
begin
if (~rst)
	clock_P <= 0;
else
	if (P != 2)
		clock_P <= ~clock_P;
	else
		clock_P <= clock_P;
end    
//The next clk will be based on negative edge count
//if 0 or 1 change value; if 2 hold value
always_ff @(negedge clk)
begin
if (~rst)
	clock_N <= 0;
else
	if (N != 2)
		clock_N <= ~clock_N;
	else
		clock_N <= clock_N;
end

//clk goes to 1 only when either one or both of clock_P and clock_N is 1
or o1(result,clock_P,clock_N);
assign out_clk = result;
endmodule
*/

module cd3 (input logic clk , input logic rst, output logic out_clk );

logic QB = 0, DB, Q1 = 0, D1, Q2 = 0, D2;

always_ff @(posedge clk) 
begin
if(~rst)
    QB <= 1'b0;
   else
    QB <= DB;
end

always_ff @(posedge clk) 
begin
  if(~rst)
    Q1 <= 1'b0;
   else
    Q1 <= Q;
end

always_ff @(negedge clk) 
begin
  if(~rst)
    Q2 <= 1'b0;
   else
    Q2 <=  Q1;
end

//D = ~(Q and Q1);
and and1(DB,~Q, ~Q1);
//assign D = A;
//out_clk <= Q1 or Q2;
or or1(out_clk, Q1, Q2);
//assign out_clk = B;


endmodule

