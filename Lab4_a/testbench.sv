`timescale 1ns / 1ps


module test;
parameter DATA_WIDTH = 8;
parameter FIFO_DEPTH = 16;
parameter PERIOD_W =10;
parameter PERIOD_R=12;
logic rst;
logic wclk;
logic [7:0] wdata;
logic write_en;
logic full;
//
logic rclk;
logic [7:0] rdata;
logic read_en;
logic empty;
//
logic error;
logic write_en_r;
logic read_en_r;
logic read_en_d1;
logic [7:0] ref_data;
logic change = 1'b0;
logic [4:0]write_ptr;
logic [4:0]read_ptr;
//ref_data
always_ff@(posedge rclk,posedge rst)
if(rst)
    ref_data<=0;
else if(read_en_d1)
    ref_data<=ref_data+1;
//read_en_d1
always_ff@(posedge rclk)
    read_en_d1<=read_en;

//error
always_comb
if(read_en_d1&&rdata!=ref_data)
    error=1;
else
    error=0;
initial begin
	rdata = 8'b00000000;
end
//wclk
initial begin
    wclk=0;
    forever begin
        #(PERIOD_W/2) wclk=~wclk;
    end
end
//rclk
initial
begin
    rclk=0;
    forever
    begin
        #(PERIOD_R/2) rclk=~rclk;
    end
end
//rst
initial
begin
    rst=1;
    #50
    rst=0;
end
//wdata
always_ff@(posedge wclk,posedge rst)
if(rst)
    wdata<=0;
else if(write_en&&~full)
    wdata<=wdata+1;
//wr_en_r
always_ff@(posedge wclk,posedge rst)
if(rst)
    write_en_r<=0;
else
    write_en_r=$random%2;
//rd_en_r
always_ff@(posedge rclk,posedge rst)
if(rst)
    read_en_r<=0;
else
    read_en_r<=$random%2;
//read_en,write_en
assign read_en=(~empty)?read_en_r:1'b0;
assign write_en=(~full)?write_en_r:1'b0;
//inst
lab4_a
#(
.DATA_WIDTH(8),
.FIFO_DEPTH(16)
)
U
(.*);
endmodule



