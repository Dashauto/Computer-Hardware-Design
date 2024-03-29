`timescale 1ns / 1ps

module test;
parameter DATA_WIDTH = 8;
parameter FIFO_DEPTH = 16;
logic write_en_r;
logic read_en_r;
logic clk;
logic rst;
logic write_en;
logic read_en;
logic [DATA_WIDTH-1:0] wdata;
logic [DATA_WIDTH-1:0] rdata;
logic empty;
logic full;
logic [DATA_WIDTH-1:0] ref_data;
logic read_en_ff1;
logic error;
logic [4:0]write_ptr;
logic [4:0]read_ptr;
logic change = 1'b0;
//ref_data
always_ff@(posedge clk,posedge rst)
if(rst)
    ref_data<=0;
else if(read_en_ff1)
    ref_data<=ref_data+1;
//read_en_ff1
always_ff@(posedge clk)
    read_en_ff1<=read_en;
//write_en,read_en
assign write_en=(~full) ? write_en_r:1'b0;
assign read_en=(~empty) ? read_en_r:1'b0;
 

//clk
initial begin
    clk=0;
    forever begin
        #5 clk=~clk;
    end
end
//rst
initial
begin
    rst=1;
    #20
    rst=0;
end
//wdata
always_ff@(posedge clk,posedge rst)
if(rst)
    wdata<=0;
else if(write_en&&~full)              
    wdata<=wdata+1;
//wr_en
always_ff@(posedge clk,posedge rst)
if(rst)
    write_en_r<=0;
else if($random%100<90)               //有60%的几率写数据,衡量数据写入速率
    write_en_r<=1'b1;
else
    write_en_r<=1'b0;
//rd_en
always_ff@(posedge clk,posedge rst)
if(rst)
    read_en_r<=0;
else if($random%100<40)                //有40%的几率读数据，衡量数据读出速率
    read_en_r<=1'b1;
else 
    read_en_r<=1'b0;

always_comb
if(read_en_ff1&&ref_data!=rdata)
   error=1;
else
   error=0;
lab4_s
#(
.DATA_WIDTH(8),
.FIFO_DEPTH(16)
)
U
(.*);
endmodule

