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
        cs = S0;
    end
    else begin
        cs = ns;
    end
end


assign pcwrite = (zero & branch) | pcupdate;


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