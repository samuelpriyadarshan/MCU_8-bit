`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/30/2021 08:12:12 PM
// Design Name: 
// Module Name: MCU_main
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
module program_memory(address, instruction);
    input [7:0] address;
    output [16:0] instruction;
    
    
    reg [16:0] memory [255:0];
    reg [16:0] instruction;
    integer i;
    initial begin
        
        for(i=0; i<256 ; i=i+1)begin
            case(i)
            /*0: memory[i] = 17'b00000011001010100; 
            6: memory[i] = 17'b00001011001010100; 
            8: memory[i] = 17'b00010011001000011; 
            10: memory[i] = 17'b00011011001000011; 
            12: memory[i] = 17'b00100011001010100;
            14: memory[i] = 17'b00101011001010100; 
            16: memory[i] = 17'b00110011001010100; 
            18: memory[i] = 17'b00111011001010100;
            20: memory[i] = 17'b01000011001000011; 
            22: memory[i] = 17'b01001011001010100;
            24: memory[i] = 17'b01010011001010100;
            25: memory[i] = 17'b01010011011010100; 
            28: memory[i] = 17'b01011011001010100; 
            30: memory[i] = 17'b01100011001010100; 
            32: memory[i] = 17'b01101011001010100; 
            34: memory[i] = 17'b01110011001010100;
            36: memory[i] = 17'b01111011001010100; 
            38: memory[i] = 17'b10000011001010100;
            40: memory[i] = 17'b10001011001010100; 
            42: memory[i] = 17'b10010011001010100; 
            44: memory[i] = 17'b10011011001010100; 
            46: memory[i] = 17'b10100011001010100;
            
            default: memory[i] = 17'b00000011001010100;*/
            //0: memory[i] = 17'b00000011001010100;
                            0: memory[i] = 17'b01011001000000000;
                            1: memory[i] = 17'b10001010001000100;
                            2: memory[i] = 17'b10101010010000100;
                            3: memory[i] = 17'b10101011001000100;
                            4: memory[i] = 17'b00101000010011000;
                            5: memory[i] = 17'b10100000000000000;
                            
                            default: memory[i] = 17'b00000011001010100;
            endcase
            #0.01;
        end
    end
     
    always@(address)begin
        instruction = memory[address];
    end
    
    
endmodule

module branch_seq(BS, zero, PS, PC, BrA, RAA, MC_out);
    
    input BS, zero, PS, PC, BrA, RAA;
    output [7:0] MC_out;
    
    
    
    wire [1:0] BS;
    wire zero, PS;
    wire [7:0] RAA, BrA, PC;
    
    
    reg [1:0] MC;
    reg [7:0] MC_out;
    
    always@(*)begin
        
        MC[0] <= BS[0] && (BS[1] || (PS ^ BS[0]));
        MC[1] <= BS[1];
        
        case(MC)
            2'b00: MC_out= PC + 1'b1; 
            2'b10: MC_out= RAA;
            2'b01: MC_out= BrA;
            2'b11: MC_out= BrA;
        endcase
        
        
    end
    
endmodule

module register_file(clk, A_addr, B_addr, D_addr, data_in, write_enable, A_data, B_data);
    input clk, write_enable;
    input [2:0] A_addr, B_addr, D_addr;
    input [7:0] data_in;
    output [7:0] A_data, B_data;
   
    reg [7:0] memory[7:0];
    reg [7:0] A_data;
    reg [7:0] B_data;
    integer i;
    
    initial begin
        for(i=1 ; i<8 ; i=i+1)begin
            memory[i] <= i;
            //memory_B[i] <= i;
            #0.01;
        end
    end
    always@(posedge clk)begin
        memory[0] <= 0;
    end
  
    always@(negedge clk)begin
        memory[0] <= 0;
        if (write_enable)begin
            memory [D_addr] <= data_in;
        end
    end
    
    always@(A_addr)begin
        A_data <= memory[A_addr];
    end
    
    always@(B_addr)begin
        B_data <= memory[B_addr];
    end
    
    
endmodule

module Inst_Decode(Inst, RW,DA, MD, BS, PS, MW, FS, MA, MB, MD, AA, BA, CS);
    input Inst;
    output RW,DA, MD, BS, PS, MW, FS, MA, MB, AA, BA, CS;
    
    wire [16:0] Inst;
    reg [2:0] DA, AA, BA;
    reg [1:0] MD, BS;
    reg [3:0] FS;
    reg RW, MW, MA,MB, CS, PS;
    reg [4:0]OPCODE;
    
    
    
    always@(*)begin
       OPCODE <= Inst[16:12];
       
       case(OPCODE)
            5'b00000:begin RW=0; DA=Inst[11:9]; MD=2'b00; BS=2'b00; PS=0; MW=0; FS=4'b0000; MA=0; MB=0; AA=Inst[8:6]; BA=Inst[5:3]; CS = 0; end
            
            5'b00001:begin RW=1; DA=Inst[11:9]; MD=2'b00; BS=2'b00; PS=0; MW=0; FS=4'b0010; MA=0; MB=0; AA=Inst[8:6]; BA=Inst[5:3]; CS = 0; end //SUB
            
            5'b00010:begin RW=1; DA=Inst[11:9]; MD=2'b00; BS=2'b11; PS=0; MW=0; FS=4'b1010; MA=1; MB=1; AA=Inst[8:6]; BA=Inst[5:3]; CS = 1; end //JML
            
            5'b00011:begin RW=0; DA=Inst[11:9]; MD=0; BS=2'b01; PS=0; MW=0; FS=4'b0000; MA=1; MB=1; AA=Inst[8:6]; BA=Inst[5:3]; CS=1 ; end //BZ
            
            5'b00100:begin RW=1; DA=Inst[11:9]; MD=2'b00; BS=2'b00; PS=0; MW=0; FS=4'b0001; MA=0; MB=1; AA=Inst[8:6]; BA=Inst[5:3]; CS = 1; end //ADI
            
            5'b00101:begin RW=0; DA=Inst[11:9]; MD=2'b10; BS=2'b00; PS=0; MW=1; FS=4'b0000; MA=0; MB=0; AA=Inst[8:6]; BA=Inst[5:3]; CS = 0; end //OUT
            
            5'b00110:begin RW=1; DA=Inst[11:9]; MD=2'b00; BS=2'b00; PS=0; MW=0; FS=4'b0111; MA=0; MB=0; AA=Inst[8:6]; BA=Inst[5:3]; CS = 0; end //SLT
            
            5'b00111:begin RW=1; DA=Inst[11:9]; MD=2'b01; BS=2'b00; PS=0; MW=0; FS=4'b0000; MA=0; MB=0; AA=Inst[8:6]; BA=Inst[5:3]; CS = 0; end //LD
            
            5'b01000:begin RW=0; DA=Inst[11:9]; MD=2'b00; BS=2'b11; PS=1'b0; MW=1'b0; FS=4'b0000; MA=1; MB=1; AA=Inst[8:6]; BA=Inst[5:3]; CS = 1'b1; end //JMP
            
            5'b01001:begin RW=1; DA=Inst[11:9]; MD=2'b00; BS=0; PS=0; MW=0; FS=4'b0001; MA=0; MB=1; AA=Inst[8:6]; BA=Inst[5:3]; CS = 1; end //AIU
            
            5'b01010:begin RW=1; DA=Inst[11:9]; MD=2'b00; BS=2'b00; PS=0; MW=0; FS=4'b0011; MA=0; MB=0; AA=Inst[8:6]; BA=Inst[5:3]; CS = 0; end //XOR
            
            5'b01011:begin RW=1; DA=Inst[11:9]; MD=2'b10; BS=2'b00; PS=0; MW=0; FS=4'd9; MA=0; MB=0; AA=Inst[8:6]; BA=Inst[5:3]; CS = 0; end //IN
            
            5'b01100:begin RW=1; DA=Inst[11:9]; MD=2'b00; BS=2'b00; PS=0; MW=0; FS=4'b1000; MA=0; MB=1; AA=Inst[8:6]; BA=Inst[5:3]; CS = 1; end //ANI
            
            5'b01101:begin RW=1; DA=Inst[11:9]; MD=2'b00; BS=2'b00; PS=0; MW=0; FS=4'b0101; MA=0; MB=0; AA=Inst[8:6]; BA=Inst[5:3]; CS = 0; end //NOT
            
            5'b01110:begin RW=0; DA=Inst[11:9]; MD=2'b00; BS=2'b00; PS=0; MW=1; FS=4'b0000; MA=0; MB=0; AA=Inst[8:6]; BA=Inst[5:3]; CS = 0; end //STR
            
            5'b01111:begin RW=0; DA=Inst[11:9]; MD=2'b00; BS=2'b01; PS=1; MW=0; FS=4'b0000; MA=1; MB=1; AA=Inst[8:6]; BA=Inst[5:3]; CS = 1; end //BNZ
            
            5'b10000:begin RW=1; DA=Inst[11:9]; MD=2'b00; BS=2'b00; PS=0; MW=0; FS=4'b0001; MA=0; MB=0; AA=Inst[8:6]; BA=0; CS = 0; end //MOV
            
            5'b10001:begin RW=1; DA=Inst[11:9]; MD=2'b00; BS=2'b00; PS=0; MW=0; FS=4'b0110; MA=0; MB=0; AA=Inst[8:6]; BA=0; CS = 0; end //LSL
            
            5'b10010:begin RW=1; DA=Inst[11:9]; MD=2'b00; BS=2'b00; PS=0; MW=0; FS=4'b0100; MA=0; MB=1; AA=Inst[8:6]; BA=0; CS = 1; end //ORI
            
            5'b10011:begin RW=1; DA=Inst[11:9]; MD=2'b00; BS=2'b00; PS=0; MW=0; FS=4'b0001; MA=0; MB=0; AA=Inst[8:6]; BA=Inst[5:3]; CS = 0; end //ADD
            
            5'b10100:begin RW=0; DA=0; MD=2'b00; BS=2'b10; PS=0; MW=0; FS=4'b0000; MA=0; MB=0; AA=Inst[8:6]; BA=0; CS = 1; end //JMR
            
            5'b10101:begin RW=1; DA=Inst[11:9]; MD=2'b00; BS=2'b00; PS=0; MW=0; FS=4'b0110; MA=0; MB=0; AA=Inst[8:6]; BA=0; CS = 0; end //LSR

            default: begin RW=0; DA=Inst[11:9]; MD=0; BS=0; PS=0; MW=0; FS=0; MA=0; MB=0; AA=Inst[8:6]; BA=Inst[5:3]; CS = 0; end
       endcase

    end
     
endmodule

module MUXs(PC, Inst, A_data, B_data, CS, MA, MB, MD, MA_out, MB_out, MD_out, F_out, Mem_out, IO_out);
    
    input PC, Inst, A_data, B_data, CS, MD, MA, MB, F_out, Mem_out, IO_out;
    output MA_out, MB_out, MD_out;
    
    reg [7:0] MA_out, MB_out, MD_out;
    wire [7:0] A_data, B_data, PC;
    wire CS;
    wire [16:0] Inst;
    wire MA, MB;
    wire [1:0] MD;
    wire [7:0] F_out, Mem_out, IO_out;
    
    always@(*)begin
        case(MA)
            1'b0: MA_out = A_data;
            1'b1: MA_out = PC;
        endcase
        
        case(MB)
            1'b0: MB_out = B_data;
            //1'b1: MB_out = {CS,Inst[5:0]};
            
            1'b1:begin
                if (CS == 1)begin
                    MB_out = {Inst[5], Inst[5], Inst[5:0]};
                    //MB_out =  Inst[5:0];
                end
                
                else 
                    MB_out = {1'b0, 1'b0, Inst[5:0]};
            end
        endcase
        
        case(MD)
            2'b00: MD_out = F_out;
            2'b01: MD_out = Mem_out;
            2'b10: MD_out = IO_out;
             
        endcase
    end
endmodule

module ALU(func_sel, Shift, A, B, F, zero, negative, carry, overflow);
    input [3:0] func_sel; 
    input [2:0] Shift;
    input [7:0]  A, B; 
    output [7:0] F; 
    output zero, negative, carry, overflow;
    
    wire [7:0] A,B;
    reg [7:0] F;
    reg overflow, zero;
    reg [8:0] F_w_carry;
    reg carry, negative;
    
    
    parameter ADD = 4'd1,
    SUB = 4'd2,
    XOR = 4'd3,
    OR = 4'd4,
    NOT = 4'd5,
    LSL = 4'd6,
    SLT = 4'd7,
    AND = 4'd8,
    PASSB = 4'd0,
    PASSA = 4'd9,
    PC1 = 4'b10,
    LSR = 4'd11;
    
    
   
    always@(func_sel)begin
        carry <= 0;
        negative <= 0;
        zero <= 0;
        overflow <= 0;
        
        case(func_sel)
            ADD: begin 
                assign F_w_carry = A+B;
            end
            
            SUB:begin
                assign F_w_carry = A + (~B) + 1;
                F = F_w_carry[7:0];
            end
            
            XOR:begin
                assign F_w_carry = A^B;
           end
            
            OR:begin
                assign F_w_carry = A||B;
            end
            
            NOT:begin
                assign F_w_carry = ~A[7:0];
                F_w_carry[8] <= 0;
            end
            
            LSL:begin
                assign F_w_carry = A << Shift;
                F = F_w_carry[7:0];
            end
            
            SLT:begin
                assign F = (A<B) ? 1 : 0 ;
                assign F_w_carry = (A<B) ? 1 : 0 ;
           end
            AND:begin
                assign F_w_carry= A && B ;
            end
            
            PASSA: assign F_w_carry = A;
            PASSB: assign F_w_carry = B;
            PC1: begin 
                assign F_w_carry = A ;
                F_w_carry <= F_w_carry + 1'b1;
            end
            
            LSR: begin
                assign F_w_carry = A >> Shift;
                F = F_w_carry[7:0];
            end
        
        endcase
        assign F = F_w_carry[7:0];
        assign zero = F ? 0 : 1;
        assign negative = F[7];
        assign carry = F_w_carry[8]; 
        assign overflow = F_w_carry[8] ^ F_w_carry[7];
    end
    
endmodule

module data_memory(clk, address, write_enable, data_input, data_output);
    input clk;
    input [7:0] address;
    input write_enable;
    input  [7:0] data_input;
    output [7:0] data_output;
    
    //wire write_enable;
    reg [7:0] memory [255:0];
    reg [7:0] data_output;
    integer i;
    
    initial begin
            for(i=0 ; i<256 ; i=i+1)begin
                memory[i] <= i;
                //memory_B[i] <= i;
                #0.01;
            end
    end
    
    always @(posedge clk)begin
       
        data_output <= memory[address];
          
    end
    
    always @(negedge clk) begin
        if(write_enable)begin
            memory[address] <= data_input ;
        end
    end
    
    
    
endmodule

module DHS_m(DA_O, MA, RW, Comp_A, Comp_B, MB, HA, HB, DHS);
    
    input DA_O, MA, RW, Comp_A, Comp_B, MB;
    output HA, HB, DHS;
    
    reg HA, HB;
    wire [2:0] DA_O;
    wire MA, MB, Comp_A, Comp_B, RW;
    wire DHS;
    
    always@(*)begin
        //DHS = () || ();    
        HA = (DA_O[2]||DA_O[1]||DA_O[0]) && RW && Comp_A && !MA;
        HB = (DA_O[2]||DA_O[1]||DA_O[0]) && RW && Comp_B && !MB;
    end
    
    assign DHS = HA||HB;
endmodule

module branch_detect(BS_O, RW_b, BS_b, MW_b);
    input BS_O;
    output RW_b,BS_b,MW_b;
    reg RW_b, MW_b, BS_b;
    wire [1:0] BS_O;
    

    always@(*)begin
        RW_b = !(BS_O[0] || BS_O[1]);
        BS_b = !(BS_O[0] || BS_O[1]);
        MW_b = !(BS_O[0] || BS_O[1]);
    end
    
endmodule

module clock_100to25(
    input clk_100M,
    output reg clk_25M = 0
    );
	reg count_bit = 0;
	always @(posedge clk_100M)
	begin
		count_bit = ~ count_bit;
		if (~count_bit)
			clk_25M = ~clk_25M;
	end
	
	

endmodule

module color_converter(
    input [7:0] color_code,
    output reg [7:0] color_out
    );
	
	always @(color_code)
	begin
		case(color_code)
			8'd00: color_out <= 8'b00000000; // Black
			8'd01: color_out <= 8'b11100000; // Red
			8'd02: color_out <= 8'b00011100; // Green
			8'd03: color_out <= 8'b00000011; // Blue
			8'd04: color_out <= 8'b11111100; // Yellow
			8'd05: color_out <= 8'b11100011; // Magenta
			8'd06: color_out <= 8'b00011111; // Cyan
			8'd07: color_out <= 8'b10010010; // Grey
			8'd08: color_out <= 8'b00100101; // Grey Blue
			8'd09: color_out <= 8'b01111010; // Seafoam
			8'd10: color_out <= 8'b01101010; // Purple
			8'd11: color_out <= 8'b11101010; // Pink
			8'd12: color_out <= 8'b11110110; // Peach
			8'd13: color_out <= 8'b10110111; // Light Blue
			8'd14: color_out <= 8'b11010101; // Dark purple
			8'd15: color_out <= 8'b11111111; // White
			default: color_out <= 8'b00000000; // Black
		endcase
	end

endmodule

module VGA_driver(
    input clk_25M,
    input [2:0] redIn,
    input [2:0] greenIn,
    input [1:0] blueIn,
    output reg [2:0] vgaRed = 0,
    output reg [2:0] vgaGreen = 0,
    output reg [2:1] vgaBlue = 0,
    output reg Hsync = 0,
    output reg Vsync = 0,
    output reg [9:0] x_pos = 0,
    output reg [9:0] y_pos = 0
    );

	reg [9:0] h_count = 0;
	reg [9:0] v_count = 0;
	
	always @(posedge clk_25M)
	begin
		// Control counter for h_sync and v_sync control
		if (h_count == 799)
		begin
			h_count <= 0;
			if (v_count == 524)
				v_count <= 0;
			else
				v_count <= v_count + 1'b1;
		end
		else
			h_count <= h_count + 1'b1;
		
		// control Hsync signal
		if ((h_count >= 0) && (h_count <= 95))
			Hsync <= 1;
		else
			Hsync <= 0;
		
		// control Vsync signal
		if ((v_count >= 0) && (v_count <= 1))
			Vsync <= 1;
		else
			Vsync <= 0;
			
		// Set color to black when outside of video bounds and output position
		if ((h_count >= 144) && (h_count <= 783) && (v_count >= 35) && (v_count <= 514))
		begin
			vgaRed <= redIn;
			vgaGreen <= greenIn;
			vgaBlue <= blueIn;
		end
		else
		begin
			vgaRed <= 0;
			vgaGreen <= 0;
			vgaBlue <= 0;
		end
		x_pos <= (h_count + 1'b1) - 9'd144; // X position being output is the next position. Will be between 0 and 639 for valid inputs
		y_pos <= v_count - 9'd35; // Y position is current position. Between 0 and 479 for valid inputs
	end


endmodule

module vga_grid16(
    input [9:0] x_pos,
    input [9:0] y_pos,
    output reg [1:0] grid_x,
    output reg [1:0] grid_y
    );
	
	// Set x_grid positon
	always @(x_pos)
	begin
		if (x_pos < 160)
			grid_x <= 2'b00;
		else if (x_pos < 320)
			grid_x <= 2'b01;
		else if (x_pos < 480)
			grid_x <= 2'b10;
		else //480-639
			grid_x <= 2'b11;
	end
	
	// Set y_grid position
	always @(y_pos)
	begin
		if (y_pos < 120)
			grid_y <= 2'b00;
		else if (y_pos < 240)
			grid_y <= 2'b01;
		else if (y_pos < 360)
			grid_y <= 2'b10;
		else //360-479
			grid_y <= 2'b11;
	end

endmodule

module vga_out(
    input clk,
	 input write_enable,
    input [7:0] position,
    input [7:0] value,
    output [2:0] vgaRed,
    output [2:0] vgaGreen,
    output [2:1] vgaBlue,
    output Hsync,
    output Vsync
    );
	 
	 wire clk_vga; // 25MHz clock for vga control
	 
	 // Pixel and color control
	 wire [9:0] x_pos;
	 wire [9:0] y_pos;
	 wire [2:0] redIn;
	 wire [2:0] greenIn;
	 wire [2:1] blueIn;
	 
	 wire [1:0] grid_x;
	 wire [1:0] grid_y;
	 
	 // Memory to store color of each segment
	 reg [7:0] color_mem [0:15];
	 
	 // Wire to carry converted color
	 wire [7:0] converted_color;
	 
	 // Set output color
	 assign {redIn, greenIn, blueIn[2:1]} = color_mem[{grid_y, grid_x}];
	 
	 clock_100to25 CLK_DIV(.clk_100M(clk), .clk_25M(clk_vga));
	 VGA_driver VGA_MOD(.clk_25M(clk_vga),
							 .redIn(redIn),
							 .greenIn(greenIn),
							 .blueIn(blueIn[2:1]),
							 .vgaRed(vgaRed),
							 .vgaGreen(vgaGreen),
							 .vgaBlue(vgaBlue),
							 .Hsync(Hsync),
							 .Vsync(Vsync),
							 .x_pos(x_pos),
							 .y_pos(y_pos)
							 );
	vga_grid16 VGA_GRID(.x_pos(x_pos),.y_pos(y_pos),.grid_x(grid_x),.grid_y(grid_y));
	color_converter COLOR_CONV(.color_code(value),.color_out(converted_color));
	
	always @(negedge clk)
	begin
		if (write_enable)
			color_mem[position] <= converted_color;
	end

endmodule

module mcu_io(
	 input clk,  // Clock
	 input reset,// Reset signal
	 input output_write_enable, // Write enable
	 input [7:0] output_data_in, // Data in
	 input [7:0] output_data_address, // Data address
	 output reg [7:0] input_data_out, // Data output
	 input [8:0] fpga_in, // Connection to FPGA input pins
	 output [9:0] fpga_out // Connection to FPGA output pins
    );
	 // Output
	vga_out VGA_OUT(.clk(clk),
			  .write_enable(output_write_enable),
			  .position(output_data_address),
			  .value(output_data_in),
			  .vgaRed(fpga_out[9:7]),
			  .vgaGreen(fpga_out[6:4]),
			  .vgaBlue(fpga_out[3:2]),
			  .Hsync(fpga_out[1]),
			  .Vsync(fpga_out[0])
    );
	 
//	 wire [7:0] sw;
//	 assign sw = fpga_in[7:0];
//	 
//	 always @(negedge clk)
//		if (!reset)
//			input_data_out <= 0;
//		else
//			input_data_out <= sw;
	 
	 // input
	 always @(fpga_in[8])
	 begin
		if (!reset)
			input_data_out <= 8'd0;
		else
			//if (!fpga_in[8])
				input_data_out <= fpga_in[7:0];
	 end

endmodule

module MCU_main(clk, reset);
    
    input clk;
    input reset;
    wire reset;
    wire clk;
    reg a;
    wire b;
    reg [7:0] address;
    wire [16:0] instruction;
    reg [16:0] IR;
    reg [7:0] ALU_out, Mem_out, IO_out;
    
    reg [7:0] Bus_A, Bus_B;
    
    //INSTRUCTION DECODER
    wire RW, PS, MW, CS, MA, MB;
    wire [2:0] DA, AA, BA;
    wire [1:0] BS, MD;
    wire [3:0] FS;
    
    reg RW_O, PS_O, MW_O, CS_O, MA_O, MB_O;
    reg [2:0] DA_O, AA_O, BA_O;
    reg [1:0] BS_O, MD_O;
    reg [3:0] FS_O;
    
    reg RW_O_1;
    reg [1:0] MD_O_1;
    reg [2:0] DA_O_1;
    
    //wire [16:0] Inst;
    
    //BRANCH SELECT
    wire zero;
    reg [7:0] BrA, RAA;
    wire [7:0] MC_out;
    
    
    //DECODE
    reg [5:0] IM ; 
    reg [2:0] SH ,SH_O;
    
    // REGISTER FILE
    wire [7:0] A_data, B_data;
    
    //MUXs
    wire [7:0] MA_out, MB_out, MD_out;
    
    //ALU
    wire [7:0] F;
    wire negative, carry, overflow;
    
    //DATA MEMORY
    wire [7:0] data_output;
    
    reg [7:0] PC, PC_1, PC_2, PC_3;
    reg [1:0] MC;
    
    //DHS
    wire HA, HB;
    wire DHS_m;
    reg DHS;
    
    //Comp
    reg Comp_A, Comp_B;
    
    //BRANCH DETECT
    wire RW_b, MW_b, BS_b;
    
    //IO_module
    reg output_wire_enable;
    reg [7:0] output_data_in, output_data_address; 
    reg [8:0] fpga_in;
    wire [7:0] input_data_out;
    wire [9:0] fpga_out;
    
    program_memory pm_main(.address(address), .instruction(instruction));
    Inst_Decode Instdecode(.Inst(IR), .RW(RW), .DA(DA), .MD(MD), .BS(BS), .PS(PS), .MW(MW), .FS(FS), .MA(MA), .MB(MB), .AA(AA), .BA(BA), .CS(CS));
    branch_seq Branch_sel (.BS(BS_O), .zero(zero), .PS(PS_O), .PC(PC), .BrA(BrA), .RAA(RAA), .MC_out(MC_out));
    register_file regfile(.clk(clk), .A_addr(AA), .B_addr(BA), .D_addr(DA_O_1), .data_in(MD_out), .write_enable(RW_O_1), .A_data(A_data), .B_data(B_data));
    MUXs MUX_inst (.MA_out(MA_out), .MB_out(MB_out), .A_data(A_data), .B_data(B_data), .PC(PC_1), .CS(CS), .Inst(IR), .MA(MA), .MB(MB), .MD_out(MD_out), .F_out(ALU_out), .Mem_out(Mem_out), .MD(MD_O_1), .IO_out(IO_out));
    ALU ALU_test (.func_sel(FS_O), .Shift(SH_O), .A(Bus_A), .B(Bus_B), .F(F), .zero(zero), .negative(negative), .carry(carry), .overflow(overflow));
    data_memory DM_TEST (.clk(clk), .address(address), .write_enable(MW_O), .data_input(Bus_B), .data_output(data_output) );
    DHS_m DHS_test(.DA_O(DA_O), .MA(MA), .RW(RW_O), .Comp_A(Comp_A), .Comp_B(Comp_B), .MB(MB), .HA(HA), .HB(HB), .DHS(DHS_m));
    branch_detect br_test(.BS_O(BS_O), .RW_b(RW_b), .BS_b(BS_b), .MW_b(MW_b));     
    mcu_io IO_MODULE(.clk(clk),.reset(reset),.output_write_enable(MW_O),.output_data_in(Bus_A),.output_data_address(Bus_B),.input_data_out(input_data_out),.fpga_in(fpga_in),.fpga_out(fpga_out) );  
    
    initial begin
            
            PC <= 8'b00000000;
            
            PC_1 <= 8'b00000000;
            PC_2 <= 8'b00000000;
            
            IR = 17'd0;
            
            #10;
            //repeat(5) @ (posedge clk);
            
            
        end
        always@(*)begin
            DHS = DHS_m;
            Comp_A = (DA_O == AA) ? 1 : 0;
            Comp_B = (DA_O == BA) ? 1 : 0;
        end
        
        
        always@(posedge clk)begin
        
                if(!reset) PC <= 8'd0;
            
            
                if(DHS) begin PC = PC  ; 
                             PC_1 = PC_1 ; 
                             PC_2 = PC_2 +1'b1; 
                             RW_O_1 = RW_O;
                             DA_O_1 = DA_O;
                             DA_O = 0;
                             RW_O = 0;
                             IR = 0;  
                             DHS = 0; 
                             BS_O = 0;
                       end
                    else begin  
                             PC_2 = PC_1 ;
                             PC_1 = PC;
                              
                             PC = MC_out;
                             RW_O_1 = RW_O; 
                             RW_O = RW; 
                             DA_O_1 = DA_O; 
                             DA_O = DA; BS_O = BS;end
                    
                
             
            if (BS_O)begin
                IR = 0;
                BS_O = 0;
                
            end
            
            else begin
                IR = instruction;
                BS_O = BS;
                
            end
            
            
            //PC
            address = PC;
            
            //PC_1
            IR = instruction ;
            SH = IR [2:0];
            IM = IR [5:0];
            
            //PC_2
            Bus_A = MA_out;
            Bus_B = MB_out;
            RAA = Bus_A;
                        
                        MD_O_1 = MD_O;
                        //RW_O_1 = RW_O;
                        //DA_O_1 = DA_O;
                        
                        
                        BS_O = BS;
                        PS_O = PS; 
                        MW_O = MW && !DHS;//(MW_b);
                        FS_O = FS; 
                        SH_O = SH;
                         
                        
                        
                        
                        //RW_O = RW ;//(RW_b);
                        //DA_O = DA ;
                        MD_O = MD;
                        
                        BrA = PC_2 + Bus_B;
                        
                        ALU_out = F;
                        Mem_out = data_output;
                        IO_out = input_data_out;
        end 
        
        
        always@(PC)begin
            
            
            //BSM = BS;
        end
        
        always@(PC_1)begin
            
            //PC = MC_out;
            
            
            
        end
        
        always@(PC_2)begin
            
            
        end
    
    
endmodule


