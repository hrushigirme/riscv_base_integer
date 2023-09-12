`timescale 1ns / 1ps

module riscv_tb();
reg clk_in;
integer i;

parameter reg0 = 5'b00000,reg1 = 5'b00001,reg2 = 5'b00010,reg3 = 5'b00011,reg4= 5'b00100,reg5 = 5'b00101,reg6 = 5'b00110,
          reg7 = 5'b00111,reg8 = 5'b01000,reg9 = 5'b01001,reg10 = 5'b01010,reg11= 5'b01011,reg12 = 5'b01100,reg13 = 5'b01101,
          reg14 = 5'b01110,reg15 = 5'b01111,reg16 = 5'b10000,reg17 = 5'b10001,reg18= 5'b10010,reg19 = 5'b10011,reg20 = 5'b10100,
          reg21 = 5'b10101,reg22 = 5'b10110,reg23 = 5'b10111,reg24 = 5'b11000,reg25= 5'b11001,reg26 = 5'b11010,reg27 = 5'b11011,
          reg28 = 5'b11100,reg29 = 5'b11101,reg30 = 5'b11110,reg31 = 5'b11111;
          
parameter u_lui = 7'b0110111,u_auipc = 7'b0010111,j_jal = 7'b1101111,i_jalr = 7'b1100111,b_beq = 7'b1100011,i_lb = 7'b0000011,
          s_sb = 7'b0100011, i_addi = 7'b0010011,i_slli = 7'b0010011,r_add = 7'b0110011;

riscv rv(clk_in);

initial begin
clk_in = 0;
forever begin
#5 clk_in = ~clk_in;
end
end

// register values
initial begin
for(i = 0; i<32; i =i +1) begin          // filling the reg with values
rv.DP.id_stage.regist.regi_32bit[i] = i;
end

rv.DP.id_stage.regist.regi_32bit[16] = 32'hf0001234; // reg16 value is changed
rv.DP.id_stage.regist.regi_32bit[17] = 32'h12345678; // reg17 value is changed
rv.DP.id_stage.regist.regi_32bit[20] = 32'hf0000000; // reg20 value is changed
rv.DP.id_stage.regist.regi_32bit[3]  = 32'h00000005; // reg3 value is changed
rv.DP.id_stage.regist.regi_32bit[22]  =32'hfffffffe; // reg22 value is changed
// program
rv.DP.if_stage.IM.mem[0] = {7'b0000000,reg2,reg1,3'b000,reg4,r_add}; //add
rv.DP.if_stage.IM.mem[1] = {7'b0100000,reg1,reg2,3'b000,reg4,r_add}; //sub
rv.DP.if_stage.IM.mem[2] = {7'b0000000,reg2,reg1,3'b000,reg4,r_add}; //add
rv.DP.if_stage.IM.mem[3] = {7'b0100000,reg1,reg2,3'b000,reg4,r_add}; //sub
rv.DP.if_stage.IM.mem[4] = {7'b0000000,reg2,reg1,3'b000,reg4,r_add}; //add
rv.DP.if_stage.IM.mem[5] = {7'b0000000,reg1,reg5,3'b011,reg4,r_add}; //slt reg4,reg2,reg1
rv.DP.if_stage.IM.mem[6] = {7'b0000000,reg5,reg1,3'b011,reg4,r_add}; //slt reg4,reg1,reg2
rv.DP.if_stage.IM.mem[7] = {7'b0000000,reg3,reg1,3'b110,reg4,r_add}; //or reg4,reg1,reg3
rv.DP.if_stage.IM.mem[8] = {12'b000000000111,reg1,3'b000,reg4,i_addi}; //addi reg4,reg1,7
rv.DP.if_stage.IM.mem[9] = {12'b000000000000,reg1,3'b010,reg4,i_addi}; //slti reg4,reg1,0
rv.DP.if_stage.IM.mem[10] ={12'b000000000111,reg1,3'b100,reg4,i_addi}; //xori reg4,reg1,7
rv.DP.if_stage.IM.mem[11] ={12'b000000000110,reg5,3'b110,reg4,i_addi}; //ori reg4,reg1,7
rv.DP.if_stage.IM.mem[12] ={12'b000000000110,reg5,3'b111,reg4,i_addi}; //andi reg4,reg1,7
rv.DP.if_stage.IM.mem[13] ={7'b0000000,reg5,reg3,3'b000,5'b00110,s_sb}; //sb reg5,reg3,6
rv.DP.if_stage.IM.mem[14] ={7'b0000000,reg16,reg11,3'b010,5'b00010,s_sb}; //sw reg5,reg3,6
rv.DP.if_stage.IM.mem[15] ={7'b0000000,reg16,reg11,3'b001,5'b00000,s_sb}; //sw reg5,reg3,6
rv.DP.if_stage.IM.mem[16] ={12'b000000000111,reg5,3'b000,reg4,i_addi}; //addi reg4,reg1,7
rv.DP.if_stage.IM.mem[17] ={7'b0000000,reg17,reg11,3'b010,5'b00010,s_sb}; //sw reg5,reg3,6
rv.DP.if_stage.IM.mem[18] ={12'b000000001011,reg2,3'b000,reg11,i_lb}; //lb reg5,reg3,6
rv.DP.if_stage.IM.mem[19] ={12'b000000001011,reg2,3'b001,reg11,i_lb}; //lh reg5,reg3,6
rv.DP.if_stage.IM.mem[20] ={12'b000000001011,reg2,3'b010,reg11,i_lb}; //lw reg5,reg3,6
rv.DP.if_stage.IM.mem[21] ={12'b000000001011,reg2,3'b010,reg12,i_lb}; //lw reg5,reg3,6
rv.DP.if_stage.IM.mem[22] ={20'h01256,reg13,u_lui}; //lui reg5,reg3,6
rv.DP.if_stage.IM.mem[23] ={20'h01256,reg13,u_auipc}; //auipc reg5,reg3,6
rv.DP.if_stage.IM.mem[24] ={20'h01256,reg18,u_auipc}; //auipc reg5,reg3,6
rv.DP.if_stage.IM.mem[25] ={12'b000000001011,reg2,3'b010,reg18,i_lb}; //lw reg18,reg3,6
rv.DP.if_stage.IM.mem[26] ={7'b0000000,5'b00100,reg2,3'b001,reg18,i_slli}; //shift   shft amt should be min 1
rv.DP.if_stage.IM.mem[27] ={7'b0100000,5'b00100,reg20,3'b101,reg18,i_slli}; //shift right arth imm

rv.DP.if_stage.IM.mem[28] ={7'b0000000,reg5,reg18,3'b001,reg17,r_add}; //shift left logical
rv.DP.if_stage.IM.mem[29] ={7'b0000000,reg3,reg17,3'b101,reg18,r_add}; //shift right logical 5 times
rv.DP.if_stage.IM.mem[30] ={12'b000000001011,reg2,3'b010,reg12,i_lb}; //lw reg5,reg3,6
rv.DP.if_stage.IM.mem[31] ={7'b0100000,reg3,reg18,3'b101,reg17,r_add}; //shift right arithmatic
rv.DP.if_stage.IM.mem[32] ={12'b000000001011,reg2,3'b001,reg12,i_lb}; //lw reg5,reg3,6
rv.DP.if_stage.IM.mem[33] ={7'b0000000,reg17,reg2,3'b010,5'b01100,s_sb}; //sw reg5,reg3,6
rv.DP.if_stage.IM.mem[34] ={7'b0100000,reg3,reg12,3'b101,reg17,r_add}; //shift right arithmatic

rv.DP.if_stage.IM.mem[35] = {7'b0000000,reg1,reg2,3'b111,5'b01001,b_beq}; //jal
rv.DP.if_stage.IM.mem[37] = {7'b0000000,reg2,reg2,3'b000,reg6,r_add}; //add
rv.DP.if_stage.IM.mem[38] = {7'b0100000,reg1,reg2,3'b000,reg6,r_add}; //sub
rv.DP.if_stage.IM.mem[39] = {7'b0000000,reg2,reg3,3'b000,reg6,r_add}; //add
rv.DP.if_stage.IM.mem[40] = {7'b0100000,reg1,reg4,3'b000,reg6,r_add}; //sub
rv.DP.if_stage.IM.mem[41] = {7'b0000000,reg1,reg5,3'b000,reg6,r_add}; //add
 
rv.DP.if_stage.IM.mem[42] = {7'b0000000,reg1,reg2,3'b111,5'b01001,b_beq}; //jal
rv.DP.if_stage.IM.mem[43] = {7'b0000000,reg2,reg2,3'b000,reg6,r_add}; //add
rv.DP.if_stage.IM.mem[44] = {7'b0100000,reg1,reg2,3'b000,reg6,r_add}; //sub
rv.DP.if_stage.IM.mem[45] = {7'b0000000,reg2,reg3,3'b000,reg6,r_add}; //add
rv.DP.if_stage.IM.mem[46] = {7'b0100000,reg1,reg4,3'b000,reg6,r_add}; //sub
rv.DP.if_stage.IM.mem[47] = {7'b0000000,reg1,reg5,3'b000,reg6,r_add}; //add

rv.DP.if_stage.IM.mem[48] = {7'b0000000,reg2,reg5,3'b000,reg4,r_add}; //add
rv.DP.if_stage.IM.mem[49] = {7'b0000000,reg2,reg1,3'b000,reg4,r_add}; //add
rv.DP.if_stage.IM.mem[50] = {12'b111111111111,reg22,3'b010,reg4,i_addi}; //slti  -2 < -1 is true
rv.DP.if_stage.IM.mem[51] = {7'b0000000,reg1,reg3,3'b000,reg2,r_add}; //add
rv.DP.if_stage.IM.mem[52] = {7'b0000000,reg2,reg1,3'b000,reg6,r_add}; //add


end
//rv.DP.if_stage.IM.mem


endmodule
