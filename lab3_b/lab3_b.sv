module lab3_b(input logic clk,
 input logic reset,
 input logic left, right,
 output logic la, lb, lc, ra, rb, rc);

logic [2:0]current_state;
logic [2:0]next_state;

logic [2:0]SAT = 3'b000;
logic [2:0]LE0 = 3'b001;
logic [2:0]LE1 = 3'b010;
logic [2:0]LE2 = 3'b011;
logic [2:0]RE0 = 3'b100;
logic [2:0]RE1 = 3'b101;
logic [2:0]RE2 = 3'b110;

always_ff @ (posedge clk or negedge reset)
begin
	if(!reset)
		current_state <= SAT;
	else 
		current_state <= next_state;
end

always_comb
begin
	case(current_state)
		SAT: if (left == 1'b1)
				next_state = RE0;
			  else if (right == 1'b1)
			   next_state = LE0;
			  else
			  begin 
			   next_state = current_state;
			  end
				
		LE0: next_state = LE1;		
		LE1: next_state = LE2;		
		LE2: next_state = SAT;
		
		RE0: next_state = RE1;
		RE1: next_state = RE2;	
		RE2: next_state = SAT;
		
		default: next_state = SAT;
	endcase
end


always@(posedge clk or negedge reset) 
begin
	if (!reset)
	begin
		la <= 1'b0;
		lb <= 1'b0;
		lc <= 1'b0;
		ra <= 1'b0;
		rb <= 1'b0;
		rc <= 1'b0;
	end
	
	else
	begin
		case(current_state)
			SAT: begin
			la <= 1'b0;
			lb <= 1'b0;
			lc <= 1'b0;
			ra <= 1'b0;
			rb <= 1'b0;
			rc <= 1'b0;
			end
			
			LE0: begin
			la <= 1'b0;
			lb <= 1'b0;
			lc <= 1'b0;
			ra <= 1'b1;
			rb <= 1'b0;
			rc <= 1'b0;
			end
			
			LE1: begin
			la <= 1'b0;
			lb <= 1'b0;
			lc <= 1'b0;
			ra <= 1'b1;
			rb <= 1'b1;
			rc <= 1'b0;
			end
			
			LE2: begin
			la <= 1'b0;
			lb <= 1'b0;
			lc <= 1'b0;
			ra <= 1'b1;
			rb <= 1'b1;
			rc <= 1'b1;
			end
			
			RE0: begin
			la <= 1'b1;
			lb <= 1'b0;
			lc <= 1'b0;
			ra <= 1'b0;
			rb <= 1'b0;
			rc <= 1'b0;
			end
			
			RE1: begin
			la <= 1'b1;
			lb <= 1'b1;
			lc <= 1'b0;
			ra <= 1'b0;
			rb <= 1'b0;
			rc <= 1'b0;
			end
			
			RE2: begin
			la <= 1'b1;
			lb <= 1'b1;
			lc <= 1'b1;
			ra <= 1'b0;
			rb <= 1'b0;
			rc <= 1'b0;
			end
		endcase
	end
end
  
endmodule

 