module top(input  logic        clk, rst_n, 
           output logic [31:0] WriteData, DataAdr, 
           output logic        MemWrite);

  logic Adrsrc;
  logic[31:0] ReadData;

  riscmulti rvmulti(clk, rst_n, MemWrite, WriteData, Adrsrc, DataAdr, ReadData);
  idmem     mem(clk, MemWrite, Adrsrc, DataAdr, WriteData, ReadData);


endmodule

module riscmulti(input  logic        clk, rst_n,
                 //output logic [31:0] PC,
                 //input  logic [31:0] Instr,
                 output logic        MemWrite,
                 output logic [31:0] WriteData,
                 output logic Adrsrc,
                 output logic [31:0] Adr,
                 input  logic [31:0] ReadData);

    logic       irwrite , pcwrite, regwrite, Zero;
    logic [1:0] ResultSrc, ImmSrc, alusrca, alusrcb;
    logic [2:0] ALUControl;
    logic [31:0] Instr;


    datapath   dp(clk, rst_n, pcwrite, Adrsrc, irwrite, ResultSrc, ALUControl, alusrca, alusrcb, ImmSrc, regwrite, ReadData, Zero, Adr, WriteData, Instr);
    controller  c(clk, rst_n, Instr[6:0], Instr[14:12], Instr[30], Zero, ImmSrc, alusrca, alusrcb, ResultSrc, Adrsrc, ALUControl, irwrite, pcwrite, regwrite, MemWrite);

endmodule

module datapath(input logic clk,
                input logic rst_n,
                input logic pcwrite,            
                input logic adrsrc,    //Write in the memory?
               // input logic memwrite,
                input logic irwrite,
                input logic[1:0] resultsrc,
                input logic[2:0] ALUControl,
                input logic[1:0] alusrca,
                input logic[1:0] alusrcb,
                input logic[1:0] immsrc,
                input logic regwrite,
                input logic [31:0] ReadData, 
                output logic zero,
                output logic [31:0] Adr,
                //output logic[31:0] ALUResult,
                output logic[31:0] WriteData,
                output logic[31:0] Instr
                //output logic[31:0] PC
                );

    logic [31:0] PCNext, OldPC; 
    //logic [31:0] Instr; 
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

    //PC&data logic
    assign PCNext = Result;
    regdp1 PCreg(clk, rst_n, pcwrite, PCNext, PC);
    mux2   PCMUX(PC, Result, adrsrc, Adr);  //Write in the memory?
    regdp2 Instreg(clk, rst_n, irwrite, PC, ReadData, OldPC, Instr);
    dffdp  readdata(clk, rst_n, ReadData, Data);

    //RF logic
    regfile rf(clk, regwrite, Instr[19:15], Instr[24:20], Instr[11:7], Result, rd1, rd2);
    extend  ext(Instr[31:7], immsrc, ImmExt);
    dffdp2  rfreg(clk, rst_n, rd1, rd2, A, WriteData);

    //ALU logic
    mux3 ALU_MUXa(PC, OldPC, A, alusrca, SrcA);
    mux3 ALU_MUXb(WriteData, ImmExt, 32'd4, alusrcb, SrcB);
    alu  alu(SrcA, SrcB, ALUControl, ALUResult, zero);

    //result logic
    dffdp aluoutreg(clk, rst_n, ALUResult, ALUout);
    mux3  resultmux(ALUout, Data, ALUResult, resultsrc, Result);

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
    //   3'b100:   immext = {instr[31], instr[30:20], instr[19:12], 12'b0};
      default: immext = 32'bx; // undefined
    endcase             
endmodule


////////////////////////////////////////////////////
//////////////Instrction and Data memory////////////
////////////////////////////////////////////////////
module idmem(input  logic clk, we,
             input  logic adrsrc,
             input  logic [31:0] a, wd,
             output logic [31:0] rd);

  logic [31:0] iRAM[63:0];
  logic [31:0] dRAM[63:0];

  initial
      $readmemh("riscvtest.txt",iRAM);

  assign rd = (adrsrc == 1'b0) ? iRAM[a[31:2]] : dRAM[a[31:2]]; // word aligned

  //assign rd = (adrsrc == 1'b1) ? dRAM[a[31:2]] : 32'bx; // word aligned

  always_ff @(posedge clk)
    if (we) dRAM[a[31:2]] <= wd;
endmodule

///////////////////////////////////////
/////////////////MUX///////////////////
///////////////////////////////////////
module mux2(input  logic [31:0] d0, d1, 
            input  logic             s, 
            output logic [31:0] y);

  assign y = s ? d1 : d0; 
endmodule

module mux3(input  logic [31:0] d0, d1, d2,
            input  logic [1:0]       s, 
            output logic [31:0] y);

  assign y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule

/////////////////////////////////////////////
/////////////register and dff////////////////
/////////////////////////////////////////////
module regdp1(input logic clk,
             input logic rst_n,
             input logic en,
             input logic[31:0] d1,
             output logic[31:0] d2);

always_ff@(posedge clk or negedge rst_n)
begin
    if(!rst_n)begin
        d2 <= 32'b0;
    end

    else begin
        if(en == 1'b1)begin
            d2 <= d1;
        end

        else begin
            d2 <= d2;
        end
    end
end

endmodule


module regdp2(input logic clk,
             input logic rst_n,
             input logic en,
             input logic[31:0] d1, d2,
             output logic[31:0] d3, d4);

always_ff@(posedge clk or negedge rst_n)
begin
    if(!rst_n)begin
        d3 <= 32'b0;
        d4 <= 32'b0;
    end
    else begin
        if(en == 1'b1)begin
            d3 <= d1;
            d4 <= d2;
        end

        else begin
            d3 <= d3;
            d4 <= d4;
        end
    end
end

endmodule

module dffdp(input logic clk,
             input logic rst_n,
             input logic[31:0] d1,
             output logic[31:0] d2);

always_ff @(posedge clk or negedge rst_n)
begin
    if(!rst_n)begin
        d2 <= 32'b0;
    end

    else begin
        d2 <= d1;
    end

end
endmodule

module dffdp2(input logic clk,
              input logic rst_n,
              input logic[31:0] d1, d3,
              output logic[31:0] d2, d4);
				  
always_ff @(posedge clk or negedge rst_n) 
begin
    if(!rst_n)begin
        d2 <= 32'b0;
        d4 <= 32'b0;
    end

    else begin
        d2 <= d1;
        d4 <= d3;
    end

end
endmodule


/////////////////////////////////////////////
/////////////Controller//////////////////////
/////////////////////////////////////////////
module controller(input logic clk,
                  input logic rst_n,
                  input logic[6:0] op,
                  input logic[2:0] funct3,
                  input logic funct7b5,
                  input logic zero,
                  output logic[1:0] immsrc,
                  output logic[1:0] alusrca, alusrcb,
                  output logic[1:0] resultsrc,
                  output logic adrsrc,
                  output logic[2:0] alucontrol,
                  output logic irwrite, pcwrite,
                  output logic regwrite, memwrite);

logic[1:0] aluop;
mainFSM m(clk, rst_n, op[6:0], zero, pcwrite, irwrite, regwrite, memwrite, resultsrc[1:0], alusrca[1:0], alusrcb[1:0], adrsrc, aluop[1:0]);

aludec alu(op[5], aluop[1:0], funct3[2:0], funct7b5, alucontrol[2:0]);

instrdec ins(op[6:0], immsrc[1:0]);

endmodule

module mainFSM(input logic clk,
               input logic rst_n,
               input logic[6:0] op,
               input logic zero,
               output logic pcwrite, irwrite,
               output logic regwrite, memwrite,
               output logic[1:0] resultsrc,
               output logic[1:0] alusrca, alusrcb,
               output logic adrsrc,
               output logic[1:0] aluop);

parameter S0 = 11'b00000000001;
parameter S1 = 11'b00000000010;
parameter S2 = 11'b00000000100;
parameter S3 = 11'b00000001000;
parameter S4 = 11'b00000010000;
parameter S5 = 11'b00000100000;
parameter S6 = 11'b00001000000;
parameter S7 = 11'b00010000000;
parameter S8 = 11'b00100000000;
parameter S9 = 11'b01000000000;
parameter S10 = 11'b10000000000;

logic[10:0] cs, ns;
logic pcupdate;
logic branch;

always_ff@(posedge clk or negedge rst_n)
begin
    if(!rst_n)begin
        cs <= S0;
    end
    else begin
        cs <= ns;
    end
end


assign pcwrite = (zero && branch) || pcupdate;


always_comb 
begin
    case (cs)
        S0 : begin
                branch = 1'b0;
                pcupdate = 1'b1;
                regwrite = 1'b0;
                memwrite = 1'b0;
                irwrite = 1'b1;
                resultsrc = 2'b10;
                alusrca = 2'b0;
                alusrcb = 2'b10;
                adrsrc = 1'b0;
                aluop = 2'b00;
                ns = S1;

        end
        
        S1 : begin
                branch = 1'b0;
                pcupdate = 1'b0;
                regwrite = 1'b0;
                memwrite = 1'b0;
                irwrite = 1'b0;
                resultsrc = 2'b0;
                alusrca = 2'b01;
                alusrcb = 2'b01;
                adrsrc = 1'b0;
                aluop = 2'b0;
                if((op == 7'b0000011) || (op == 7'b0100011))begin
                    ns = S2;
                end
                
                else if(op == 7'b0110011)begin
                    ns = S6;
                end

                else if(op == 7'b0010011)begin
                    ns = S8;
                end

                else if(op == 7'b1101111)begin
                    ns = S9;
                end

                else if(op == 7'b1100011)begin
                    ns = S10;
                end

                else begin
                    ns = S0;
                end
        end
        
        S2 : begin
              branch = 1'b0;
              pcupdate = 1'b0;
              regwrite = 1'b0;
              memwrite = 1'b0;
              irwrite = 1'b0;
              resultsrc = 2'b0;
              alusrca = 2'b10;
              alusrcb = 2'b01;
              adrsrc = 1'b0;
              aluop = 2'b0;
              if(op == 7'b0000011)begin
                ns = S3;
              end

              else if(op == 7'b0100011)begin
                ns = S5;
              end

              else begin
                ns = S0;
              end
        end

        S3 : begin
                branch = 1'b0;
                pcupdate = 1'b0;
                regwrite = 1'b0;
                memwrite = 1'b0;
                irwrite = 1'b0;
                resultsrc = 2'b0;
                alusrca = 2'b0;
                alusrcb = 2'b0;
                adrsrc = 1'b1;
                aluop = 2'b0;
                ns = S4;
        end

        S4 : begin
                branch = 1'b0;
                pcupdate = 1'b0;
                regwrite = 1'b1;
                memwrite = 1'b0;
                irwrite = 1'b0;
                resultsrc = 2'b01;
                alusrca = 2'b0;
                alusrcb = 2'b0;
                adrsrc = 1'b0;
                aluop = 2'b0;
                ns = S0;
        end

        S5 : begin
                branch = 1'b0;
                pcupdate = 1'b0;
                regwrite = 1'b0;
                memwrite = 1'b1;
                irwrite = 1'b0;
                resultsrc = 2'b00;
                alusrca = 2'b0;
                alusrcb = 2'b0;
                adrsrc = 1'b1;
                aluop = 2'b0;
                ns = S0;
        end

        S6 : begin
                branch = 1'b0;
                pcupdate = 1'b0;
                regwrite = 1'b0;
                memwrite = 1'b0;
                irwrite = 1'b0;
                resultsrc = 2'b00;
                alusrca = 2'b10;
                alusrcb = 2'b0;
                adrsrc = 1'b0;
                aluop = 2'b10;
                ns = S7;
        end

        S7 : begin
                branch = 1'b0;
                pcupdate = 1'b0;
                regwrite = 1'b1;
                memwrite = 1'b0;
                irwrite = 1'b0;
                resultsrc = 2'b00;
                alusrca = 2'b0;
                alusrcb = 2'b0;
                adrsrc = 1'b0;
                aluop = 2'b0;
                ns = S0;
        end

        S8 : begin
              branch = 1'b0;
              pcupdate = 1'b0;
              regwrite = 1'b0;
              memwrite = 1'b0;
              irwrite = 1'b0;
              resultsrc = 2'b00;
              alusrca = 2'b10;
              alusrcb = 2'b01;
              adrsrc = 1'b0;
              aluop = 2'b10;
              ns = S7;
        end

        S9 : begin
                branch = 1'b0;
                pcupdate = 1'b1;
                regwrite = 1'b0;
                memwrite = 1'b0;
                irwrite = 1'b0;
                resultsrc = 2'b00;
                alusrca = 2'b01;
                alusrcb = 2'b10;
                adrsrc = 1'b0;
                aluop = 2'b0;
                ns = S7;
        end

        S10 : begin
                branch = 1'b1;
                pcupdate = 1'b0;
                regwrite = 1'b0;
                memwrite = 1'b0;
                irwrite = 1'b0;
                resultsrc = 2'b00;
                alusrca = 2'b10;
                alusrcb = 2'b0;
                adrsrc = 1'b0;
                aluop = 2'b01;
                ns = S0;
        end

        default: begin
            ns = S0;
        end
    endcase
end

endmodule

module aludec(input logic opb5,
              input logic[1:0] aluop,
              input logic[2:0] funct3,
              input logic      funct7b5,
              output logic[2:0] alucontrol);

    logic  RtypeSub;
    assign RtypeSub = funct7b5 & opb5;  // TRUE for R-type subtract instruction

    always_comb
        case(aluop)
            2'b00:                alucontrol = 3'b0000; // addition
            2'b01:                alucontrol = 3'b0001; // subtraction
            default: case(funct3) // R-type or I-type ALU
                        2'b000:  if (RtypeSub) 
                                   alucontrol = 3'b001; // sub
                                 else          
                                   alucontrol = 3'b000; // add, addi
                        3'b010:    alucontrol = 3'b101; // slt, slti
                        3'b110:    alucontrol = 3'b011; // or, ori
                        3'b111:    alucontrol = 3'b010; // and, andi
                        default:   alucontrol = 3'b000; // ???
                     endcase
    endcase
endmodule


module instrdec(input logic[6:0] op,
                output logic[1:0] ImmSrc);
    always_comb
        case(op)
            7'b0110011: ImmSrc = 2'bXX; // R-type
            7'b0010011: ImmSrc = 2'b00; // I-type ALU
            7'b0000011: ImmSrc = 2'b00; // lw
            7'b0100011: ImmSrc = 2'b01; // sw
            7'b1100011: ImmSrc = 2'b10; // beq
            7'b1101111: ImmSrc = 2'b11; // jal
            default: ImmSrc = 2'bXX; // ???
        endcase
endmodule




////////////////////////////
///////////ALU//////////////
////////////////////////////
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

