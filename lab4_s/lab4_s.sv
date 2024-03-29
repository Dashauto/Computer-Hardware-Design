`timescale 1ns / 1ps

module lab4_s
#(parameter DATA_WIDTH = 8,
  parameter FIFO_DEPTH = 16)
(
input logic clk,
input logic rst,
input logic write_en,
input logic [7:0] wdata,
input logic read_en,
output logic [7:0] rdata,
output logic full,
output logic empty,
output logic [4:0]write_ptr,
output logic [4:0]read_ptr
    );
logic [3:0] write_next;             
logic [3:0] read_next;                
logic [4:0] write_position;                   
logic [4:0] read_position;                  

assign write_next = write_position[3:0];
assign read_next = read_position[3:0];

logic [7:0] Data [0:15];

always_ff@(posedge clk,posedge rst)
if(rst)
    write_position <= 0;                                    
else if(write_en && ~full)                          
    write_position <= write_position + 1;

always_ff@(posedge clk,posedge rst)
if(rst)
    read_position <= 0;
else if(read_en && ~empty)                           
    read_position <= read_position + 1;

assign full = (read_position=={~write_position[4],write_next})?1'b1:1'b0;
assign empty = (read_position==write_position)?1'b1:1'b0;

always_ff@(posedge clk)
if(write_en && ~full)
    Data[write_next] <= wdata;

always_ff@(posedge clk,posedge rst)       
if(rst)
    rdata <= 0;
else if(read_en && ~empty)
    rdata <= Data[read_next];

assign write_ptr = write_position[4:0];
assign read_ptr = read_position[4:0];	 

endmodule
































