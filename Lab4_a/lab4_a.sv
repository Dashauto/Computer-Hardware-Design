`timescale 1ns / 1ps


module lab4_a
#(parameter DATA_WIDTH = 8,
  parameter FIFO_DEPTH = 16)
(
input logic rst,
input logic wclk,
input logic [7:0] wdata,
input logic write_en,
input logic rclk,
input logic read_en,
output logic full,
output logic [7:0] rdata,
output logic empty,
output logic [4:0]write_ptr,
output logic [4:0]read_ptr
    );
logic [7:0] Data [15];
logic [3:0] write_position;                
logic [3:0] read_position;               
logic [4:0] write_next;              
logic [4:0] read_next;               
logic [4:0] write_next_gray;
logic [4:0] write_next_gray_d1;
logic [4:0] write_next_gray_d2;
logic [4:0] read_next_gray;
logic [4:0] read_next_gray_d1;
logic [4:0] read_next_gray_d2;
always_ff@(posedge wclk,posedge rst)
if(rst)
    write_next <= 0;
else if(write_en && ~full)      
    write_next <= write_next + 1;       
always_ff@(posedge rclk,posedge rst)
if(rst)
    read_next <= 0;
else if(read_en && ~empty)                   
    read_next <= read_next + 1;     
assign write_position = write_next[3:0];
assign read_position = read_next[3:0];   
always_ff@(posedge wclk)
if(write_en && ~full)
    Data[write_position] <= wdata;             
always_ff@(posedge rclk)
if(read_en && ~empty)
    rdata <= Data[read_position];           
assign write_next_gray=(write_next >> 1) ^ write_next;
assign read_next_gray=(read_next >> 1) ^ read_next; 

always_ff@(posedge wclk,posedge rst)
if(rst)
begin
    read_next_gray_d1 <= 0;
    read_next_gray_d2 <= 0;
end
else
begin
    read_next_gray_d1 <= read_next_gray;                            
    read_next_gray_d2 <= read_next_gray_d1;                         
end
always_ff@(posedge rclk,posedge rst)
if(rst)
begin
    write_next_gray_d1 <= 0;
    write_next_gray_d2 <= 0;
end
else
begin
    write_next_gray_d1 <= write_next_gray;
    write_next_gray_d2 <= write_next_gray_d1;                        
end

assign full=(write_next_gray=={~read_next_gray_d2[4:4-1],read_next_gray_d2[4-2:0]})?1'b1:1'b0;       
assign empty=(read_next_gray==write_next_gray_d2)?1'b1:1'b0;
assign write_ptr = write_next[4:0];
assign read_ptr = read_next[4:0];	 

endmodule










