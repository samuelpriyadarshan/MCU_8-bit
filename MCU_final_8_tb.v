`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/19/2021 11:18:21 AM
// Design Name: 
// Module Name: MCU_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module MCU_tb;
    reg clk;
    
    reg reset;
    MCU_main MCU_test(.clk(clk), .reset(reset));
    
    always #10 clk = ~ clk;
    initial begin
            clk = 0;
            reset = 0;
            #110;
            reset = 1;
            
            
    end
endmodule

