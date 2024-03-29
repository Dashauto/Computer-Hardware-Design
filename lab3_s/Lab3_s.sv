
module Lab3_s(input logic clk,
                            input logic reset,
                            input logic left, right,
                            output logic la, lb, lc, ra, rb, rc
                            );

logic D1, D2, D3, D4, D5, D6;
logic Q1 = 0, Q2 = 0, Q3 = 0, Q4 = 0, Q5 = 0, Q6 = 0;
logic P1,P2;
logic A1,A2;
                            
always_ff @(posedge clk) 
begin

Q1 <= D1 & reset;
Q2 <= D2 & reset;
Q3 <= D3 & reset;
Q4 <= D4 & reset;
Q5 <= D5 & reset;
Q6 <= D6 & reset;
	
end
    
assign la = Q1;
assign lb = Q2;
assign lc = Q3;
assign ra = Q4;
assign rb = Q5;
assign rc = Q6;

or o1(P1,Q1,~Q2);
or o2(P2,Q4,~Q5);
and a1(A1,~Q4,~Q5,~Q6,~Q3);
and a2(A2,~Q1,~Q2,~Q3,~Q6);
and a3(D1,left,A1,P1);
and a4(D2,A1,Q1);
and a5(D3,A1,Q1,Q2);
and a6(D4,right,A2,P2);
and a7(D5,A2,Q4);
and a8(D6,A2,Q4,Q5);

endmodule

