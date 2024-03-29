module riscvmulti(input  logic        clk, reset, 
           output logic [31:0] WriteData, DataAdr, 
           output logic        MemWrite);

  logic Adrsrc;
  logic[31:0] ReadData;

  riscvmulti1 rvmulti(clk, reset, MemWrite, WriteData, Adrsrc, DataAdr, ReadData);
  idmem     mem(clk, MemWrite, Adrsrc, DataAdr, WriteData, ReadData);

endmodule

module riscvmulti1(input  logic       clk, reset,
                 output logic        MemWrite,
                 output logic [31:0] WriteData,
                 output logic Adrsrc,
                 output logic [31:0] Dataaddress,
                 input  logic [31:0] ReadData);

    logic       irwrite , pcwrite, regwrite, Zero;
    logic [1:0] ResultSrc, ImmSrc, alusrca, alusrcb;
    logic [2:0] ALUControl;
    logic [31:0] Instr;


    datapath   dp(clk, reset, pcwrite, Adrsrc, irwrite, ResultSrc, ALUControl, alusrca, alusrcb, ImmSrc, regwrite, ReadData, Zero, Dataaddress, WriteData, Instr);
    controller  c(clk, reset, Instr[6:0], Instr[14:12], Instr[30], Zero, ImmSrc, alusrca, alusrcb, ResultSrc, Adrsrc, ALUControl, irwrite, pcwrite, regwrite, MemWrite);

endmodule

module datapath(input logic clk,
                input logic reset,
                input logic pcwrite,            
                input logic adrsrc,    
                input logic irwrite,
                input logic[1:0] resultsrc,
                input logic[2:0] ALUControl,
                input logic[1:0] alusrca,
                input logic[1:0] alusrcb,
                input logic[1:0] immsrc,
                input logic regwrite,
                input logic [31:0] ReadData, 
                output logic zero,
                output logic [31:0] Dataaddress,
                output logic[31:0] WriteData,
                output logic[31:0] Instr
                );

    logic [31:0] PCNext, OldPC;  
    logic [31:0] Data; 
    logic [31:0] ImmExt;
    logic [31:0] rd1; 
    logic [31:0] rd2; 
    logic [31:0] A; 
    logic [31:0] SrcA, SrcB;
    logic [31:0] ALUout; 
    logic [31:0] Result; 
    logic[31:0] ALUResult;
    logic[31:0] PC;

    // next PC logic
    assign PCNext = Result;
    RegNext Regnext(clk, reset, pcwrite, PCNext, PC);
    mux2 #(32)  m(PC, Result, adrsrc, Dataaddress); 
    RegFetch Regfetch(clk, reset, irwrite, PC, ReadData, OldPC, Instr);
    Buffer  buffer1(clk, reset, ReadData, Data);

    // register file logic
    regfile rf(clk, regwrite, Instr[19:15], Instr[24:20], Instr[11:7], Result, rd1, rd2);
    extend  ext(Instr[31:7], immsrc, ImmExt);
    Buffer_2  buffer_2(clk, reset, rd1, rd2, A, WriteData);

    // ALU logic
    mux3 #(32)  MUX_1(PC, OldPC, A, alusrca, SrcA);
    mux3 #(32)  MUX_2(WriteData, ImmExt, 32'd4, alusrcb, SrcB);
    alu  alu(SrcA, SrcB, ALUControl, ALUResult, zero);

    // Result logic
    Buffer buffer2(clk, reset, ALUResult, ALUout);
    mux3 #(32)  MUX_3(ALUout, Data, ALUResult, resultsrc, Result);

endmodule

module regfile(input  logic        clk, 
               input  logic        we3, 
               input  logic [ 4:0] a1, a2, a3, 
               input  logic [31:0] wd3, 
               output logic [31:0] rd1, rd2);

  logic [31:0] rf[31:0];

  // three ported register file
  // read two ports combinationally (A1/RD1, A2/RD2)
  // write third port on rising edge of clock (A3/WD3/WE3)
  // register 0 hardwired to 0

  always_ff @(posedge clk)
    if (we3) rf[a3] <= wd3;	

  assign rd1 = (a1 != 0) ? rf[a1] : 0;
  assign rd2 = (a2 != 0) ? rf[a2] : 0;
endmodule

module extend(input  logic [31:7] instr,
              input  logic [1:0]  immsrc,
              output logic [31:0] immext);
 
  always_comb
    case(immsrc) 
               // I-type 
      2'b00:   immext = {{20{instr[31]}}, instr[31:20]};  
               // S-type (stores)
      2'b01:   immext = {{20{instr[31]}}, instr[31:25], instr[11:7]};  
               // B-type (branches)
      2'b10:   immext = {{19{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
               // J-type (jal)
      2'b11:   immext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};
              //  U-type
      default: immext = 32'bx; 
    endcase             
endmodule

//-----------------------------------------------------------
//------------------------Data Memory------------------------
//-----------------------------------------------------------
module idmem(input  logic clk, we,
             input  logic adrsrc,
             input  logic [31:0] a, wd,
             output logic [31:0] rd);

  logic [31:0] iRAM[63:0];
  logic [31:0] dRAM[63:0];

  initial
      $readmemh("riscvtest.txt",iRAM);

  assign rd = (adrsrc == 1'b0) ? iRAM[a[31:2]] : dRAM[a[31:2]];

    // if (adrsrc == 1'b0) begin
    //     rd = iRAM[a[31:2]];
    // end
    // else begin
    //     rd = dRAM[a[31:2]];
    // end

  always_ff @(posedge clk)
    if (we) dRAM[a[31:2]] <= wd;
endmodule

module mux2 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, 
              input  logic             s, 
              output logic [WIDTH-1:0] y);

  assign y = s ? d1 : d0; 
endmodule

module mux3 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, d2,
              input  logic [1:0]       s, 
              output logic [WIDTH-1:0] y);

  assign y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule

//--------------------------------------------------------
//------------------------Register------------------------
//--------------------------------------------------------
module RegNext(input logic clk,
             input logic reset,
             input logic tag,
             input logic[31:0] Next,
             output logic[31:0] OPC);

always_ff@(posedge clk or negedge reset)
begin
    if(!reset)
        OPC <= 32'b0;
    else 
    begin
        if(tag == 1'b1)
            OPC <= Next;
        else 
            OPC <= OPC;
    end
end

endmodule


module RegFetch(input logic clk,
             input logic reset,
             input logic tag,
             input logic[31:0] Pc, read,
             output logic[31:0] old, ins);

always_ff@(posedge clk or negedge reset)
begin
    if(!reset)
    begin
        old <= 32'b0;
        ins <= 32'b0;
    end
    else begin
        if(tag == 1'b1)
        begin
            old <= Pc;
            ins <= read;
        end
        else 
        begin
            old <= old;
            ins <= ins;
        end
    end
end
endmodule

//------------------------------------------------------
//------------------------Buffer------------------------
//------------------------------------------------------
module Buffer(input logic clk,
             input logic reset,
             input logic[31:0] in,
             output logic[31:0] out);

always_ff @(posedge clk or negedge reset)
begin
    if(!reset)
    begin
        out <= 32'b0;
    end
    else 
    begin
        out <= in;
    end

end
endmodule

module Buffer_2(input logic clk,
              input logic reset,
              input logic[31:0] in1, in3,
              output logic[31:0] a, wd);
				  
always_ff @(posedge clk or negedge reset) 
begin
    if(!reset)
    begin
        a <= 32'b0;
        wd <= 32'b0;
    end
    else 
    begin
        a <= in1;
        wd <= in3;
    end
end
endmodule

//----------------------------------------------------------
//------------------------controller------------------------
//----------------------------------------------------------
module controller(input  logic       clk,
                  input  logic       reset,
                  input  logic [6:0] op,
                  input  logic [2:0] funct3,
                  input  logic       funct7b5,
                  input  logic       Zero,
                  output logic [1:0] ImmSrc,
                  output logic [1:0] AluSrca, AluSrcb,
                  output logic [1:0] ResultSrc,
                  output logic       AdrSrc,
                  output logic [2:0] ALUControl,
                  output logic       IrWrite, PcWrite,
                  output logic       RegWrite, MemWrite
                  );

  logic [1:0] ALUOp;
  logic       Branch;
  logic       PcUpdate;

  FSM maind(op, reset, clk, ResultSrc, AluSrca, AluSrcb, ALUOp, MemWrite, Branch,
                RegWrite, PcUpdate, AdrSrc, IrWrite);

  aludec  alud(op[5], funct3, funct7b5, ALUOp, ALUControl);

  instrdec istrd(op, ImmSrc);

  assign PcWrite = Branch & Zero | PcUpdate;
endmodule


module FSM (input  logic [6:0] op,
            input  logic reset,
			input  logic clk,
			output logic [1:0] ResultSrc,
            output logic [1:0] AluSrca, AluSrcb,
            output logic [1:0] ALUOp,
            output logic       MemWrite,
			output logic       Branch,
            output logic       RegWrite, PcUpdate,
			output logic	   AdrSrc,
			output logic 	   IrWrite
            );
					
logic [3:0] current_state;
logic [13:0] all_control;
assign {ResultSrc, MemWrite, Branch, AluSrca, AluSrcb,
		RegWrite, PcUpdate, AdrSrc, IrWrite, ALUOp} = all_control;

parameter state_0 = 4'b0000;
parameter state_1 = 4'b0001;
parameter state_2 = 4'b0010;
parameter state_3 = 4'b0011;
parameter state_4 = 4'b0100;
parameter state_5 = 4'b0101;
parameter state_6 = 4'b0110;
parameter state_7 = 4'b0111;
parameter state_8 = 4'b1000;
parameter state_9 = 4'b1001;
parameter state_A = 4'b1010;

always_ff@(posedge clk) 
begin
	if (!reset) begin
		current_state = state_0;
	end
	else begin
		case (current_state)
            state_0: current_state = state_1;

            state_1: begin
                case(op)
                    7'b0000011: current_state = state_2; // lw
				    7'b0100011: current_state = state_2; // sw
					7'b0110011: current_state = state_6; // R-type 
					7'b0010011: current_state = state_8; // I-type ALU
					7'b1101111: current_state = state_9; // jal
					7'b1100011: current_state = state_A; // beq
					default:    current_state = state_1; // non-implemented instruction 

                endcase
            end

            state_2: begin
                case(op)
                    7'b0000011: current_state = state_3; // lw
					7'b0100011: current_state = state_5; // sw
					default:    current_state = state_2; // non-implemented instruction

                endcase
            end

            state_3: current_state = state_4;

            state_4: current_state = state_0;
            state_5: current_state = state_0;
            state_7: current_state = state_0;
            state_A: current_state = state_0;

            state_6: current_state = state_7;
            state_8: current_state = state_7;
            state_9: current_state = state_7;

        endcase
			
	end
		
end

always_comb
    case(current_state)
    // ResultSrc_MemWrite_Branch_2alusrca_2alusrcb_RegWrite_PCUpdate_adrsrc_irwrite_2ALUOp
        state_0: all_control <= 14'b10_0_0_00_10_0_1_0_1_00; //Fetch           
        state_1: all_control <= 14'b00_0_0_01_01_0_0_0_0_00; //Decode
        state_2: all_control <= 14'b00_0_0_10_01_0_0_0_0_00; //MemAdr
        state_3: all_control <= 14'b00_0_0_00_00_0_0_1_0_00; //MemRead
        state_4: all_control <= 14'b01_0_0_00_00_1_0_0_0_00; //MemWB
        state_5: all_control <= 14'b00_1_0_00_00_0_0_1_0_00; //MemWrite
	    state_6: all_control <= 14'b00_0_0_10_00_0_0_0_0_10; //ExecuteR
		state_7: all_control <= 14'b00_0_0_00_00_1_0_0_0_00; //ALUWB
		state_8: all_control <= 14'b00_0_0_10_01_0_0_0_0_10; //ExecuteI
		state_9: all_control <= 14'b00_0_0_01_10_0_1_0_0_00; //JAL
		state_A: all_control <= 14'b00_0_1_10_00_0_0_0_0_01; //BEQ
        default: all_control <= 14'b00_0_0_00_00_0_0_0_0_00; //???
    endcase
	
endmodule

module aludec(input  logic       opb5,
              input  logic [2:0] funct3,
              input  logic       funct7b5, 
              input  logic [1:0] ALUOp,
              output logic [2:0] ALUControl);

  logic  RtypeSub;
  assign RtypeSub = funct7b5 & opb5;  // TRUE for R-type subtract instruction

  always_comb
    case(ALUOp)
      2'b00:                ALUControl = 3'b000; // addition
      2'b01:                ALUControl = 3'b001; // subtraction
      default: case(funct3) // R-type or I-type ALU
                3'b000: if (RtypeSub)
							ALUControl = 3'b001; // sub
						else
							ALUControl = 3'b000; // add, addi
				3'b010:     ALUControl = 3'b101; // slt, slti
				3'b110:     ALUControl = 3'b011; // or, ori
				3'b111:     ALUControl = 3'b010; // and, andi
				default:    ALUControl = 3'bxxx; // ???
               endcase
    endcase
endmodule


	


module instrdec (input  logic [6:0] op,
				 output logic [1:0] ImmSrc
                 );
always_comb
	case(op)
		7'b0110011: ImmSrc = 2'bxx; // R-type
		7'b0010011: ImmSrc = 2'b00; // I-type ALU
		7'b0000011: ImmSrc = 2'b00; // lw
		7'b0100011: ImmSrc = 2'b01; // sw
		7'b1100011: ImmSrc = 2'b10; // beq
		7'b1101111: ImmSrc = 2'b11; // jal
		default:    ImmSrc = 2'bxx; // ???
	endcase
endmodule

module alu(input  logic [31:0] a, b,
           input  logic [2:0]  alucontrol,
           output logic [31:0] result,
           output logic        zero);

  logic [31:0] condinvb, sum;
  logic        v;              // overflow
  logic        isAddSub;       // true when is add or subtract operation

  assign condinvb = alucontrol[0] ? ~b : b;
  assign sum = a + condinvb + alucontrol[0];
  assign isAddSub = ~alucontrol[2] & ~alucontrol[1] |
                    ~alucontrol[1] & alucontrol[0];

  always_comb
    case (alucontrol)
      3'b000:  result = sum;                 // add
      3'b001:  result = sum;                 // subtract
      3'b010:  result = a & b;               // and
      3'b011:  result = a | b;               // or
      3'b100:  result = a ^ b;               // xor
      3'b101:  result = (a > b)? 0 : 1;      // slt
      3'b110:  result = a << b;              // sll
      3'b111:  result = a >> b;              // srl
      default: result = 32'bx;
    endcase

  assign zero = (result == 32'b0);
  assign v = ~(alucontrol[0] ^ a[31] ^ b[31]) & (a[31] ^ sum[31]) & isAddSub;
endmodule

