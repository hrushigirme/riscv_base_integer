`timescale 1ns / 1ps

module riscv(clk_in);
input clk_in;

wire if_inc_pc,if_inc_pc_new_noadd,if_inc_pc_new,if_reset,id_wr_en,id_sel_dm2,sel_shft,ex_sel_mux3,mem_cs,mem_rd_wrbar,mem_s_ubar,wb_sel_m6,sel_shamt_id;
wire mem_sel_mux5,clk,clk_ex_mem,ex_sel_m7;
wire [1:0] ex_sel_mux2,ex_sel_demux1;
wire [6:0] id_opcode,id_fun7;
wire [2:0] id_fun3;
wire [4:0] id_shamt;
wire [1:0]  mem_count_in,mem_count_mod;
wire [31:0] ex_mem_inc_npc_cu,wb_inc_npc_cu;
wire [4:0] id_cu_rd,id_cu_rs1,id_cu_rs2;
wire hold_data_haz;
wire [31:0] id_ex_pc_cu;

data_path DP(if_inc_pc,if_inc_pc_new_noadd,if_inc_pc_new,if_reset,
                 id_wr_en,id_sel_dm2,
                 id_opcode,id_fun3,
                 id_fun7,id_shamt,
                 sel_shft,ex_sel_mux3,
                 ex_sel_mux2,ex_sel_demux1,
                 mem_count_in,mem_count_mod,
                 mem_cs,mem_rd_wrbar,mem_s_ubar,
                 wb_sel_m6,
                 sel_shft,mem_sel_mux5,
                 clk,clk_ex_mem,ex_sel_m7,clk_in,sel_shamt_id,ex_mem_inc_npc_cu,wb_inc_npc_cu,
                 id_cu_rd,id_cu_rs1,id_cu_rs2,hold_data_haz,id_ex_pc_cu
                 );
                 
 control_unit cu(clk_in,if_inc_pc,if_inc_pc_new_noadd,if_inc_pc_new,if_reset,id_opcode,id_fun3,id_fun7,id_shamt,id_sel_dm2,ex_sel_mux3,ex_sel_mux2,ex_sel_demux1,sel_shft,
                    mem_count_in,mem_count_mod,mem_cs,mem_rd_wrbar,mem_s_ubar,wb_sel_m6,clk,clk_ex_mem,id_wr_en,mem_sel_mux5,ex_sel_m7,sel_shamt_id,ex_mem_inc_npc_cu,wb_inc_npc_cu,
                    id_cu_rd,id_cu_rs1,id_cu_rs2,hold_data_haz,id_ex_pc_cu);                
endmodule


// DATA PATH

module data_path(if_inc_pc,if_inc_pc_new_noadd,if_inc_pc_new,if_reset,
                 id_wr_en,id_sel_dm2,
                 id_opcode,id_fun3,
                 id_fun7,id_shamt,
                 ex_shft,ex_sel_mux3,
                 ex_sel_mux2,ex_sel_demux1,
                 mem_count_in,mem_count_mod,
                 mem_cs,mem_rd_wrbar,mem_s_ubar,
                 wb_sel_m6,
                 mem_sel_shft,mem_sel_mux5,
                 clk,clk_ex_mem,
                 ex_sel_m7,clk_in,sel_shamt_id,ex_mem_inc_npc_cu,wb_inc_npc_cu,
                 id_out_cu_rd,id_out_cu_rs1,id_out_cu_rs2,hold_data_haz,
                 id_ex_pc_cu
                 );
input clk,clk_ex_mem,clk_in;
// if
input if_inc_pc,if_inc_pc_new_noadd,if_inc_pc_new,if_reset;
wire [31:0] if_in_pc_new,if_in_pc_new_noadd;

//id
input id_wr_en,id_sel_dm2,sel_shamt_id,hold_data_haz;
output [6:0] id_opcode;
output [2:0] id_fun3;
output [6:0] id_fun7;
output [4:0] id_shamt;
output [4:0] id_out_cu_rd,id_out_cu_rs1,id_out_cu_rs2;
wire [31:0] if_id_ir,if_id_npc,if_id_pc;
wire [31:0] id_rd_data;
wire [4:0] id_ex_rd;

// exc
input ex_shft,ex_sel_mux3,ex_sel_m7;
input [1:0] ex_sel_mux2,ex_sel_demux1;
wire [31:0] id_ex_data_mem,id_ex_rs1_data,id_ex_rs2_data,id_ex_fun7_3_opcode,id_ex_npc,id_ex_imm_r_u,id_ex_imm_r_j,id_ex_imm_sx_i,id_ex_imm_sx_s,id_ex_imm_sx_b,id_ex_pc; 
wire [31:0] ex_shft_prev;
wire [4:0] ex_mem_rd;
output [31:0] ex_mem_inc_npc_cu,id_ex_pc_cu;
//mem
input mem_sel_shft,mem_sel_mux5;
input [1:0] mem_count_in,mem_count_mod;
input mem_cs,mem_rd_wrbar,mem_s_ubar;
wire [31:0] ex_mem_data_mem,ex_mem_rd_data,ex_mem_in_pc_new_noadd,ex_mem_ld_reg,ex_mem_npc,ex_mem_imm_sx_b;
wire [4:0] mem_wb_rd;
wire [31:0] mem_wb_inc_npc_cu;
//wb
input wb_sel_m6;
wire [31:0] mem_wb_rd_data,mem_wb_npc,mem_wb_imm_sx_b;
output [31:0] wb_inc_npc_cu;
wire [4:0] wb_rd;


inst_fetch if_stage(if_in_pc_new,if_in_pc_new_noadd,if_inc_pc,if_inc_pc_new_noadd,if_inc_pc_new,if_reset,if_id_ir,clk,if_id_npc,if_id_pc,hold_data_haz);

inst_decode id_stage(id_sel_dm2,clk_in,id_wr_en,id_rd_data,if_id_ir,if_id_npc,if_id_pc,id_ex_pc,id_ex_npc,id_ex_data_mem,id_ex_rs1_data,
                    id_ex_rs2_data,id_ex_fun7_3_opcode,id_shamt,id_ex_imm_r_u,id_ex_imm_r_j,id_ex_imm_sx_i,id_ex_imm_sx_b,id_fun3,id_fun7,id_opcode,wb_rd,id_ex_rd,id_ex_imm_sx_s,clk,sel_shamt_id,
                    id_out_cu_rd,id_out_cu_rs1,id_out_cu_rs2,hold_data_haz);

Execution ex_stage(id_ex_fun7_3_opcode,id_ex_data_mem,id_ex_rs1_data,id_ex_rs2_data,id_ex_npc,id_ex_imm_r_u,id_ex_pc,id_ex_imm_r_j,
                 id_ex_imm_sx_i,id_ex_imm_sx_b,ex_sel_demux1,ex_shft_prev,ex_mem_data_mem,ex_mem_rd_data,ex_mem_in_pc_new_noadd,ex_mem_ld_reg,
                 ex_mem_npc,ex_mem_inc_npc_cu,ex_mem_imm_sx_b,ex_shft,ex_sel_mux3,ex_sel_mux2,clk_ex_mem,id_ex_rd,ex_mem_rd,id_ex_imm_sx_s,ex_sel_m7,id_ex_pc_cu);
             

memory_stage mem_stage(ex_mem_data_mem,ex_mem_rd_data,ex_mem_ld_reg,ex_mem_npc,ex_mem_inc_npc_cu,ex_mem_imm_sx_b,mem_count_in,mem_count_mod,mem_cs,mem_rd_wrbar,mem_sel_shft,mem_s_ubar,mem_wb_rd_data,
              mem_wb_npc,mem_wb_inc_npc_cu,mem_wb_imm_sx_b,ex_shft_prev,clk_ex_mem,mem_sel_mux5,ex_mem_in_pc_new_noadd,if_in_pc_new_noadd,ex_mem_rd,mem_wb_rd);
              
wb wb_stage(mem_wb_rd_data,mem_wb_npc,mem_wb_inc_npc_cu,mem_wb_imm_sx_b,id_rd_data,wb_inc_npc_cu,if_in_pc_new,wb_sel_m6,mem_wb_rd,wb_rd);

endmodule

// if stage

module inst_fetch(if_in_pc_new,
                  if_in_pc_new_noadd,
                  if_inc_pc,
                  if_inc_pc_new_noadd,
                  if_inc_pc_new,
                  if_pc_reset,
                  if_id_ir,
                  clk,
                  if_id_npc,
                  if_id_pc,
                  hold_data_haz);

input [31:0] if_in_pc_new,if_in_pc_new_noadd;
input if_inc_pc,if_inc_pc_new_noadd,if_inc_pc_new,if_pc_reset, clk,hold_data_haz;
output reg [31:0] if_id_ir;
output [31:0] if_id_pc,if_id_npc ;
wire [31:0] if_id_pc_tem,if_id_ir_tem;
wire [31:0] pc;

assign if_id_pc = if_id_pc_tem; 
assign clk_haz = (clk&~hold_data_haz);

pc prog_count(if_id_npc,if_in_pc_new,if_in_pc_new_noadd,if_inc_pc,if_inc_pc_new,if_inc_pc_new_noadd,if_pc_reset,if_id_pc_tem,clk,pc,hold_data_haz);
inst_mem IM(pc,if_id_ir_tem);



always @(posedge clk_haz)
begin
if_id_ir <= if_id_ir_tem;
end

endmodule

module pc(npc_out,in_pc_new,in_pc_new_noadd,inc_pc,inc_pc_new,inc_pc_new_noadd,reset,pc_out,clk,pc,hold_data_haz);

input [31:0] in_pc_new,in_pc_new_noadd;
input inc_pc,inc_pc_new,reset,clk,inc_pc_new_noadd,hold_data_haz;
output reg [31:0] pc=32'b0;
reg [31:0] npc;
output reg [31:0] pc_out;
output [31:0] npc_out;
always @(*)
begin
case ({inc_pc,inc_pc_new,inc_pc_new_noadd,reset})
4'b1000: npc <= pc +1;
4'b0100: npc <= (pc -4) +in_pc_new; // here in_pc_new is added with pc-4 bcz by the time in_pc_new is updated in wb stage pc will inc by 4 so instead of taking pc from wb the pc in if satge is dec by 4 
4'b0010: npc <= in_pc_new_noadd;
4'b0001: npc <= 32'b0;
default : npc <= 32'b0;
endcase
end

assign clk_haz = clk&~hold_data_haz;
assign npc_out = pc;

always @(posedge clk_haz)
begin
pc<= npc;
pc_out<= pc;
end

endmodule

module inst_mem(addr,data);
input [31:0] addr;
output [31:0] data;
reg [31:0] data_tem;
reg [31:0] mem [1023:0];
 initial begin mem[0] = 8'h15; end
always @(*)
begin
 data_tem = mem[addr[9:0]];
end
assign data = data_tem;
endmodule


//ID stage


module inst_decode(sel_dm2,clk,wr_en_reg,rd_data,ir,npc,pc,out_pc,out_npc,data_mem,
                   rs1_data,rs2_data,fun7_3_opcode,shamt,imm_r_u,imm_r_j,imm_sx_i,imm_sx_b,fun3,fun7,opcode,rd_in,rd_out,imm_sx_s,clk_mod,sel_shamt,
                   out_cu_rd,out_cu_rs1,out_cu_rs2,hold_data_haz);

input [31:0] ir,npc,pc,rd_data;
input clk,clk_mod,wr_en_reg,sel_dm2,hold_data_haz;
output reg [31:0] out_pc,out_npc,data_mem,rs1_data,rs2_data,imm_r_u,imm_r_j,imm_sx_i,imm_sx_b,imm_sx_s;
output reg [16:0] fun7_3_opcode;
output [4:0] shamt;
output [2:0] fun3;
output [6:0] fun7;
output [6:0] opcode;
output reg [4:0] rd_out;
wire [4:0] rs1,rs2,rd;
wire [31:0] rs2_data_tem,rs1_data_tem,demux2_out1,demux2_out2; 
input [4:0] rd_in; 
wire [4:0] shamt_imm;
input sel_shamt;
output [4:0] out_cu_rd,out_cu_rs1,out_cu_rs2;
wire clk_mod_haz;

assign rs1 = ir[19:15];
assign rs2 = ir[24:20];
assign rd = ir[11:7];
assign out_cu_rd = rd;
assign out_cu_rs1 = rs1;
assign out_cu_rs2 = rs2;
assign opcode = ir[6:0];
assign fun3 = ir[14:12];
assign fun7 = ir[31:25];
assign shamt_imm = ir[24:20];

reg_32bit regist(rs1,rs2,rd_in,rs1_data_tem,rs2_data_tem,rd_data,wr_en_reg,clk);
demux_2op demux_2(rs2_data_tem,demux2_out1,demux2_out2,sel_dm2);
mux_2ip_5bit mux_2inp(shamt_imm,rs2_data_tem[4:0],sel_shamt,shamt);

assign clk_mod_haz = (clk_mod & ~hold_data_haz);

always @(posedge clk_mod_haz)
begin
rd_out <= rd;
data_mem <= demux2_out1;
rs1_data <= rs1_data_tem;
rs2_data <= demux2_out2;
fun7_3_opcode <= {ir[31:25],ir[14:12],ir[6:0]};
out_npc <= npc;
out_pc <= pc;
imm_r_u <= {ir[31:12],12'b0};
imm_r_j <= {{12{ir[31]}},ir[31:12]};
imm_sx_i <= {{20{ir[31]}},ir[31:20]};
imm_sx_b <= {{20{ir[31]}},ir[31:25],ir[11:7]};
imm_sx_s <= {{20{ir[31]}},ir[31:25],ir[11:7]};

end

endmodule

module reg_32bit (rs1,rs2,rd,rs1_data,rs2_data,rd_data,wr_en,clk);
input [4:0] rs1,rs2,rd;
output  [31:0] rs1_data,rs2_data,rd_data;
input wr_en,clk;
reg [31:0] regi_32bit [31:0];

assign rs1_data =  regi_32bit[rs1];
assign rs2_data =  regi_32bit[rs2];

always @(posedge clk)
begin
if(wr_en) regi_32bit[rd] = rd_data;
end
endmodule


module demux_2op(in,out1,out2,sel);
input [31:0] in;
output reg [31:0] out1,out2;
input  sel;
always @(*)
begin
case(sel)
0:begin out1 <= in;out2<=0; end
1:begin out1 <= 0;out2<=in; end
default :begin out1 <= in;out2<=0; end
endcase
end
endmodule



module mux_2ip_5bit(in1,in2,sel,out); // 2 input mux
input [4:0] in1,in2;
input  sel;
output reg [4:0] out; 

always @(*)
begin
case(sel)
0:out <= in1;
1:out<= in2;
default : out <= in1;
endcase
end
endmodule



// execution stage modules

module Execution(id_ex_fun7_3_opcode,
                 id_ex_data_mem,
                 id_ex_rs1_data,
                 id_ex_rs2_data,
                 id_ex_npc,
                 id_ex_imm_r_u,
                 id_ex_pc,
                 id_ex_imm_r_j,
                 id_ex_imm_sx_i,
                 id_ex_imm_sx_b,
                 sel_demux1,
                 shft_prev,
                 ex_mem_data_mem,
                 ex_mem_rd_data,
                 ex_mem_in_pc_new_noadd,
                 ex_mem_ld_reg,
                 ex_mem_npc,
                 ex_mem_inc_npc_cu,
                 ex_mem_imm_sx_b,
                 shft_cu,
                 sel_m3,
                 sel_m1_m2,
                 clk,rd_in,rd_out,
                 id_ex_imm_sx_s,sel_m7,
                 pc_cu);




input [31:0] shft_prev,id_ex_data_mem,id_ex_rs1_data,id_ex_rs2_data,id_ex_npc,id_ex_imm_r_u,id_ex_pc,id_ex_imm_r_j,id_ex_imm_sx_i,id_ex_imm_sx_s,id_ex_imm_sx_b;
input [16:0] id_ex_fun7_3_opcode;
input [1:0] sel_demux1,sel_m1_m2;
input shft_cu,sel_m3,clk,sel_m7;
output reg [31:0] ex_mem_data_mem,ex_mem_rd_data,ex_mem_in_pc_new_noadd,ex_mem_ld_reg,ex_mem_npc,ex_mem_inc_npc_cu,ex_mem_imm_sx_b;
input [4:0] rd_in;
output reg [4:0] rd_out;
wire [31:0] mux1_i_s,mux2_u_j,alu_in1,alu_in2,alu_out,demux1_out1,demux1_out2,demux1_out3,demux1_out4,mux7_out_is;
output [31:0] pc_cu;

assign pc_cu =  id_ex_pc;

mux_2ip m4(shft_prev,id_ex_rs1_data,shft_cu,mux1_i_s);
mux_2ip m3(id_ex_imm_r_u,id_ex_imm_r_j,sel_m3,mux2_u_j);
mux_2ip m7(id_ex_imm_sx_i,id_ex_imm_sx_s,sel_m7,mux7_out_is);
mux_3ip m1(id_ex_rs1_data,id_ex_pc,mux1_i_s,sel_m1_m2,alu_in1);
mux_3ip m2(id_ex_rs2_data,mux2_u_j,mux7_out_is,sel_m1_m2,alu_in2);
alu ALU_module(alu_in1,alu_in2,alu_out,id_ex_fun7_3_opcode);
demux_4op demux1 (alu_out,demux1_out1,demux1_out2,demux1_out3,demux1_out4,sel_demux1);

always @ (posedge clk)
begin
rd_out <= rd_in;
ex_mem_rd_data <= demux1_out1;
ex_mem_in_pc_new_noadd<=demux1_out2;
ex_mem_ld_reg<=demux1_out3;
ex_mem_inc_npc_cu <= demux1_out4;
ex_mem_data_mem <= id_ex_data_mem;
ex_mem_npc <= id_ex_npc;
ex_mem_imm_sx_b <= id_ex_imm_sx_b;
end
endmodule

module alu(in1,in2,out,sel_alu);
input [31:0] in1,in2;
output reg [31:0] out;
input [16:0] sel_alu; // {fun7,fun3,opcode} 7+3+7 = 17bits
wire [31:0] out_LTS,out_GTES;
parameter  lui =   20'bxxxxxxxxxx0110111,
           auipc = 20'bxxxxxxxxxx0010111,
           jal =   20'bxxxxxxxxxx1101111,
           jalr =  20'bxxxxxxx0001100111,
           beq  =  20'bxxxxxxx0001100011,
           bne  =  20'bxxxxxxx0011100011,
           blt =   20'bxxxxxxx1001100011,
           bge =   20'bxxxxxxx1011100011,
           bltu =  20'bxxxxxxx1101100011,
           bgeu =  20'bxxxxxxx1111100011,
           lb   =  20'bxxxxxxx0000000011,
           lh   =  20'bxxxxxxx0010000011,
           lw   =  20'bxxxxxxx0100000011,
           lbu  =  20'bxxxxxxx1000000011,
           lhu  =  20'bxxxxxxx1010000011,
           sb  =   20'bxxxxxxx0000100011,
           sh  =   20'bxxxxxxx0010100011,
           sw  =   20'bxxxxxxx0100100011,
           addi  = 20'bxxxxxxx0000010011,
           slti  = 20'bxxxxxxx0100010011,
           sltiu = 20'bxxxxxxx0110010011,
           xori  = 20'bxxxxxxx1000010011,
           ori   = 20'bxxxxxxx1100010011,
           andi  = 20'bxxxxxxx1110010011,
           slli  = 20'b00000000010010011,
           srli  = 20'b00000001010010011,
           srai  = 20'b01000001010010011,
           add  =  20'b00000000000110011,
           sub  =  20'b01000000000110011,
           sll  =  20'b00000000010110011,
           slt  =  20'b00000000100110011,
           sltu =  20'b00000000110110011,
           xor_  = 20'b00000001000110011,//name changed
           srl  =  20'b00000001010110011,
           sra  =  20'b01000001010110011,
           or_  =  20'b00000001100110011,//name changed
           and_ =  20'b00000001110110011;//name changed
           
         
           
           
lessthan_signed LTS(in1,in2,out_LTS); //LTS less than signed
greaterthanequal_sign GTES(in1,in2,out_GTES); // greater than equal signed

always @(*)
casex({3'b000,sel_alu}) // made to 20 bits so that easily written inn hex dec 
           lui   : out <= in2;
           auipc,jal,jalr,lb,lh,lw,lbu,lhu,sb,sh,sw,addi,add : out <= in1 + in2;
           beq   : out <= (in1 == in2) ? 32'h00001 : 32'h00000;
           bne   : out <= (in1 != in2) ? 32'h00001 : 32'h00000;
           blt,slti,slt   : out <= out_LTS;//
           bge   : out <= out_GTES;
           bltu  : out <= (in1 <  in2) ? 32'h00001 : 32'h00000;
           bgeu  : out <= (in1 >= in2) ? 32'h00001 : 32'h00000; 
           sltiu : out <= (in1 <  in2) ? 32'h00001 : 32'h00000;
           xori  : out <=  in1 ^  in2;
           ori   : out <=  in1 | in2;
           andi  : out <=  in1 & in2;
           slli  : out <= in1 << 1;
           srli  : out <= in1 >> 1;
           srai  : out <= {in1[31],in1[31:1]};
           sub   : out <= in1 - in2;
           sll   : out <= in1 << 1;
           sltu  : out <= (in1 <  in2) ? 32'h00001 : 32'h00000;
           xor_  : out <= in1 ^ in2;
           srl   : out <= in1 >> 1;
           sra   : out <= {in1[31],in1[31:1]};
           or_   : out <=  in1 | in2;
           and_  : out <=  in1 | in2;
           default:out <= in1;

endcase
endmodule

module mux_3ip(in1,in2,in3,sel,out);// 3 input mux
input [31:0] in1,in2,in3;
input [1:0] sel;
output reg [31:0] out; 

always @(*)
begin
case(sel)
0:out <= in1;
1:out<= in2;
2:out<= in3;
default : out <= in1;
endcase
end
endmodule

module mux_2ip(in1,in2,sel,out); // 2 input mux
input [31:0] in1,in2;
input  sel;
output reg [31:0] out; 

always @(*)
begin
case(sel)
0:out <= in1;
1:out<= in2;
default : out <= in1;
endcase
end
endmodule

module demux_4op(in,out1,out2,out3,out4,sel);
input [31:0] in;
output reg [31:0] out1,out2,out3,out4;
input [1:0] sel;
always @(*)
begin
case(sel)
0:begin out1 <= in;out2<=0;out3<=0;out4<=0; end
1:begin out1 <= 0;out2<=in;out3<=0;out4<=0; end
2:begin out1 <= 0;out2<=0;out3<=in;out4<=0; end
3:begin out1 <= 0;out2<=0;out3<=0;out4<=in; end
default :begin out1 <= in;out2<=0;out3<=0;out4<=0; end
endcase
end
endmodule

module lessthan_signed(in1,in2,out);
input [31:0] in1,in2;
output reg [31:0] out;

always @(*) begin
if(in1[31] && in2[31]) out <= ((~in1 + 1) > (~in2 + 1)) ? 32'h00001 : 32'h00000; // in1 and in2 is neg
else if(!in1[31] && in2[31]) out <= 32'h00000;  // in2 is neg
else if(in1[31] && !in2[31])out <= 32'h00001; // in2 is neg
else out <= (in1 <  in2) ? 32'h00001 : 32'h00000; // both are pos
end
endmodule

module greaterthanequal_sign(in1,in2,out);

input [31:0] in1,in2;
output reg [31:0] out;

always @(*) begin
if(in1[31] && in2[31]) out <= ((~in1 +1) <= (~in2 +1)) ? 32'h00001 : 32'h00000; // in1 and in2 is neg
else if(~in1[31] && in2[31]) out <= 32'h00001;  // in2 is neg
else if(in1[31] && ~in2[31])out <= 32'h00000; // in2 is neg
else out <= (in1 >=  in2) ? 32'h00001 : 32'h00000; // both are pos
end

endmodule


// memory 
module memory_stage (data_mem,rd_data,ld_reg,npc,inc_npc_cu,imm_sx_b,count_in,count_mod,cs,rd_wrbar,shft,sign_unsign,out_rd_data,
                     out_npc,out_inc_npc_cu,out_imm_sx_b,shft_prev,clk,sel_m5,in_pc_new_noadd,out_in_pc_new_noadd,rd_in,rd_out);
input [31:0] data_mem,rd_data,ld_reg,npc,inc_npc_cu,imm_sx_b;
input [1:0] count_in,count_mod;
input cs,rd_wrbar,sign_unsign,shft,clk,sel_m5;
output reg [31:0] out_rd_data,out_npc,out_inc_npc_cu,out_imm_sx_b;
output [31:0] shft_prev;
input [31:0] in_pc_new_noadd;
output reg [31:0] out_in_pc_new_noadd;
wire [31:0] mux5_in1,data_out,out_rd_data_tem;
input [4:0] rd_in;
output reg [4:0] rd_out;


demux_2op dm3(rd_data,shft_prev,mux5_in1,shft);
data_mem mem_dat(ld_reg,data_mem,data_out,count_in,count_mod,clk,rd_wrbar,cs,sign_unsign);
mux_2ip m5(mux5_in1,data_out,sel_m5,out_rd_data_tem);

always @(posedge clk)
begin
rd_out <= rd_in;
out_rd_data <= out_rd_data_tem;
out_npc <= npc;
out_inc_npc_cu <= inc_npc_cu;
out_imm_sx_b <= imm_sx_b;
out_in_pc_new_noadd <= in_pc_new_noadd;
end
endmodule

module data_mem(address,data_in,data_out_ext,count_in,count_mod,clk,rd_wrbar_mem,cs_mem,s_ubar);
input [31:0] data_in,address;
input [1:0] count_in;
output [31:0] data_out_ext;
input clk,rd_wrbar_mem,cs_mem,s_ubar;
input [1:0] count_mod;
wire [31:0] mem_address,dataout,datain,muxin,demuxout,data_out;
//modules

//count_reg m1(count_in,ld_count,dec_count,clk,count_out);
alu_add add(address,count_mod,mem_address);
memory_32 mem(mem_address,rd_wrbar_mem,cs_mem,dataout,datain,clk);
mux_4bit mux1(data_in[7:0],data_in[15:8],data_in[23:16],data_in[31:24],datain,count_mod);
demux_4bit demux1(dataout,data_out[7:0],data_out[15:8],data_out[23:16],data_out[31:24],count_mod);
sign_unsigned_extension sign(data_out,s_ubar,count_in,data_out_ext);
//data_reg datain(data_in,muxin,rd_wrbar,clk,data_ld);
//data_reg dataout_mem(demuxout,data_out,rd_wrbar_mem,clk,data_ld);
endmodule

module alu_add (in1,in2,out);
input [31:0] in1;
input [1:0] in2;
output [31:0] out;
assign out = in1 + in2;
endmodule

module memory_32(address,rd_wrbar,cs,dataout,datain,clk);
input [31:0] address;
input [7:0] datain;
output reg [7:0] dataout;
reg [7:0] mem [1023:0];
input rd_wrbar,clk,cs;

always @(*)
begin
if(rd_wrbar)dataout = mem[address[9:0]]; // here inplace of dout data can't be used bcz data is wire type hence dout reg is taken and it is assigned to data
else dataout = 8'bZ;
end


always @(*)
begin
if (cs && ~rd_wrbar)mem[address[9:0]] = datain;
end
endmodule


module demux_4bit(in,out1,out2,out3,out4,sel);
input [7:0] in;
output reg [7:0] out1,out2,out3,out4;
input [1:0] sel;

always @(*)
begin

case(sel)
0:out1 = in;
1:out2 = in;
2:out3 = in;
3:out4 = in;
endcase
end
 
endmodule

module mux_4bit(in1,in2,in3,in4,out,sel);
input [7:0] in1,in2,in3,in4;
output reg [7:0] out;
input [1:0] sel;

always @(*)
begin
case(sel)
0:out = in1;
1:out = in2;
2:out = in3;
3:out = in4;
endcase
end
endmodule

module sign_unsigned_extension(in,s_ubar,countin,out);
input [31:0] in;
output reg [31:0] out;
input s_ubar;
input [1:0] countin;

always @(*)
begin

out[7:0] <= in[7:0];

if(countin == 2'b00 && s_ubar) out[15:8] <= {in[7],in[7],in[7],in[7],in[7],in[7],in[7],in[7]};
else if(countin == 2'b00 && !s_ubar) out[15:8] <= 0;
else out[15:8] <= in[15:8];

if(countin == 2'b00 && s_ubar) out[23:16] <= {in[7],in[7],in[7],in[7],in[7],in[7],in[7],in[7]};
else if(countin == 2'b00 && !s_ubar) out[23:16] <= 0;
else if(countin == 2'b01 && s_ubar) out[23:16] <= {in[15],in[15],in[15],in[15],in[15],in[15],in[15],in[15]};
else if(countin == 2'b01 && !s_ubar) out[23:16] <= 0;
else out[23:16] <= in[23:16];

if(countin == 2'b00 && s_ubar) out[31:24] <= {in[7],in[7],in[7],in[7],in[7],in[7],in[7],in[7]};
else if(countin == 2'b00 && !s_ubar) out[31:24] <= 0;
else if(countin == 2'b01 && s_ubar) out[31:24] <= {in[15],in[15],in[15],in[15],in[15],in[15],in[15],in[15]};
else if(countin == 2'b01 && !s_ubar) out[31:24] <= 0;
else out[31:24] <= in[31:24];

end
endmodule


// write back

module wb(rd_data,npc,inc_npc_cu,imm_sx_b,out_rd_data,out_inc_npc_cu,out_imm_sx_b,sel_m6,rd_in,rd_out);
input [31:0] rd_data,npc,inc_npc_cu,imm_sx_b;
output [31:0] out_rd_data,out_inc_npc_cu,out_imm_sx_b;
input sel_m6;
input [4:0] rd_in;
output [4:0] rd_out;

assign out_inc_npc_cu = inc_npc_cu;
assign out_imm_sx_b = imm_sx_b;
assign rd_out = rd_in;

mux_2ip m6(rd_data,npc,sel_m6,out_rd_data);




endmodule




// CONTROL UNIT

module control_unit(clk,inc_pc,inc_pc_new_noadd,inc_pc_new,reset,id_opcode,id_fun3,id_fun7,id_shamt,id_sel_dm2,ex_sel_m3,ex_sel_m2,ex_sel_dm1,ex_sel_shft,
                    mem_count_in,mem_count_mod,mem_cs,mem_rd_wrbar,mem_sign_unsignbar,wb_sel_m6,clk_out_rest,clk_out_ex_mem,wb_wr_en,sel_m5,ex_sel_m7,sel_shamt_id,ex_mem_inc_npc_cu,wb_inc_npc_cu,
                    id_cu_rd,id_cu_rs1,id_cu_rs2,out_hold_data_haz,id_ex_pc_cu);

output clk_out_rest,clk_out_ex_mem;
//if stage
input clk;
output inc_pc,inc_pc_new_noadd,inc_pc_new,reset,wb_wr_en;

//id stage
input [6:0] id_opcode,id_fun7;
input [2:0] id_fun3;
input [4:0] id_shamt;
output id_sel_dm2,sel_shamt_id,out_hold_data_haz;
wire [6:0] id_ex_opcode;
wire [2:0] id_ex_fun3;
wire [4:0] id_ex_shamt;
wire shft_hold_clk_id;
input [4:0] id_cu_rd,id_cu_rs1,id_cu_rs2;
wire [2:0] id_ex_count_haz;
//ex stage
input [31:0] id_ex_pc_cu;
output ex_sel_m3;
output [1:0] ex_sel_m2,ex_sel_dm1;
wire [6:0] ex_mem_opcode;
wire [2:0] ex_mem_fun3;
output ex_sel_shft,ex_sel_m7;
wire ex_mem_shft_hold_clk_id;
wire [4:0] ex_mem_shamt;
wire [2:0] ex_mem_count_haz;
//mem stage
output [1:0] mem_count_in,mem_count_mod;
output mem_cs,mem_rd_wrbar,mem_sign_unsignbar,sel_m5;
wire hold_clk_mem;
wire [6:0] mem_wb_opcode;
wire hold2_mem_clk,mem_wb_wr_en_shft;
wire mem_wb_shft_hold_clk_id,clk_mod_shft;
wire [4:0] mem_wb_shamt;
wire [2:0] mem_wb_out_fun3;
input [31:0] ex_mem_inc_npc_cu;

//wb
output wb_sel_m6;
input [31:0] wb_inc_npc_cu;


assign clk_out_rest = clk & ~hold_clk_mem & ~shft_hold_clk_id;
assign clk_out_ex_mem = clk & ~hold_clk_mem;
assign clk_mod_shft = clk & ~shft_hold_clk_id;

if_cu if_cu_stage(reset);
id_cu id_cu_stage(clk_out_rest,clk_out_ex_mem,id_opcode,id_fun3,id_fun7,id_shamt,id_sel_dm2,id_ex_opcode,id_ex_fun3,id_ex_shamt,shft_hold_clk_id,sel_shamt_id,id_cu_rd,id_cu_rs1,id_cu_rs2,out_hold_data_haz,id_ex_count_haz);
ex_cu ex_cu_stage(clk,id_ex_opcode,id_ex_fun3,id_ex_shamt,ex_sel_m3,ex_sel_m2,ex_sel_dm1,ex_mem_opcode,ex_mem_fun3,ex_sel_shft,ex_sel_m7,hold2_mem_clk,clk_out_ex_mem,clk_out_rest,ex_mem_shamt,id_ex_pc_cu,id_ex_count_haz,ex_mem_count_haz);
mem_cu mem_cu_stage(clk_out_rest,ex_mem_opcode,ex_mem_fun3,mem_count_in,mem_count_mod,mem_cs,mem_rd_wrbar,mem_sign_unsignbar,clk,hold_clk_mem,mem_wb_opcode,sel_m5,hold2_mem_clk,ex_mem_shamt,mem_wb_shamt,mem_wb_out_fun3,mem_wb_wr_en_shft,ex_mem_inc_npc_cu,ex_mem_count_haz);
wb_cu wb_cu_stage(inc_pc,inc_pc_new_noadd,inc_pc_new,clk_out_rest,clk,mem_wb_opcode,wb_sel_m6,wb_wr_en,mem_wb_shamt,mem_wb_out_fun3,mem_wb_wr_en_shft,wb_inc_npc_cu);

endmodule


//if stage

module if_cu(reset);
output reset;
assign reset=0;
endmodule



//id
module id_cu(clk_rest,clk_mem,opcode,fun3,fun7,shamt,sel_dm2,out_opcode,out_fun3,out_shamt,shft_hold_clk,sel_shamt,rd,rs1,rs2,out_hold_data_haz,out_count_haz);
input [6:0] opcode;
input [2:0] fun3;
input [6:0] fun7;
input [4:0] shamt,rd,rs1,rs2;
input clk_rest,clk_mem;
output reg sel_dm2,sel_shamt;
output reg [6:0] out_opcode ;
output reg [2:0] out_fun3;
output reg [4:0] out_shamt;
output shft_hold_clk;
reg [4:0] shamt_tem=0;
reg [4:0] shift_dec =0;
reg shft_hold1= 0,shft_hold2= 0;
reg [4:0] data_hazard_reg [3:0]; 
reg hold_data_haz_rs1 =0,hold_data_haz_rs2 =0;
wire hold_data_haz,clk_rest_haz; 
output out_hold_data_haz;
wire clk_mem_haz;
 reg [2:0] count_haz2=0;
 reg [2:0] count_haz1=0,count_haz=0;
output reg [2:0] out_count_haz=0;


assign shft_hold_clk = shft_hold1 | shft_hold2;
assign out_hold_data_haz = hold_data_haz;
always @(*)
begin
if(opcode == 7'b0100011)sel_dm2 <= 0; // stype
else sel_dm2 <=1;    //B,R type and for rest all also it will be 1

if(opcode == 7'b0010011) sel_shamt <= 0; // imm type for shift inst
else sel_shamt <= 1; // reg type for shift inst
end

 assign hold_data_haz = hold_data_haz_rs1 | hold_data_haz_rs2;
 
 
always @(*)
begin
if(count_haz1 > count_haz2) count_haz <= count_haz1;
else  count_haz <= count_haz2;
end

always @(*)
begin
if(opcode == 7'b1100111 || opcode ==7'b1100011 || opcode == 7'b0000011 || opcode == 7'b0100011 || opcode ==7'b0010011 || opcode == 7'b0110011) // rs1
begin
if(rs1 == data_hazard_reg[3]) begin hold_data_haz_rs1 =1; count_haz1 <=4; end
else if(rs1 == data_hazard_reg[2]) begin hold_data_haz_rs1 =1; count_haz1 <=3; end
else if(rs1 == data_hazard_reg[1]) begin hold_data_haz_rs1 =1; count_haz1 <=2; end
else if(rs1 == data_hazard_reg[0]) begin hold_data_haz_rs1 =1; count_haz1 <=1; end
else begin hold_data_haz_rs1 =0; count_haz1 <=0; end
end
else begin hold_data_haz_rs1 =0; count_haz1 <=0; end

if( opcode ==7'b1100011  || opcode == 7'b0100011  || opcode == 7'b0110011) //rs2
begin
if(rs2 == data_hazard_reg[3]) begin hold_data_haz_rs2 =1; count_haz2 <=4; end
else if(rs2 == data_hazard_reg[2]) begin hold_data_haz_rs2 =1; count_haz2 <=3; end
else if(rs2 == data_hazard_reg[1]) begin hold_data_haz_rs2 =1; count_haz2 <=2;end
else if(rs2 == data_hazard_reg[0]) begin hold_data_haz_rs2 =1; count_haz2 <=1; end
else begin hold_data_haz_rs2 =0; count_haz2 <=0; end
end
else begin hold_data_haz_rs2 =0; count_haz2 <=0; end


end


always @(posedge clk_rest)
begin
if(opcode ==7'b0110111 || opcode ==7'b0010111 ||opcode == 7'b1101111 ||opcode == 7'b1100111  ||opcode == 7'b0000011  ||opcode == 7'b0010011 ||opcode == 7'b0110011)
begin
data_hazard_reg[0] <= data_hazard_reg[1];  
data_hazard_reg[1] <= data_hazard_reg[2];  
data_hazard_reg[2] <= data_hazard_reg[3];  
data_hazard_reg[3] <= rd;  
end

else 
begin
data_hazard_reg[0] <= data_hazard_reg[1];  
data_hazard_reg[1] <= data_hazard_reg[2];  
data_hazard_reg[2] <= data_hazard_reg[3];  
data_hazard_reg[3] <= 5'b00000;  
end

end

assign clk_rest_haz = (clk_rest&~hold_data_haz);

always @(posedge clk_rest)
begin

 out_count_haz <= count_haz;
end

always @(posedge clk_rest_haz)
begin

out_opcode <= opcode; 
out_fun3   <= fun3;
out_shamt  <= shamt;

end
always @(posedge clk_rest_haz)
begin
if((opcode == 7'b0010011 || opcode == 7'b0110011 ) && (fun3 == 3'b001 || fun3 == 3'b101)) // shift operation shft_hold_clk
begin
//if(hold_data_haz != 1)
begin shamt_tem <= shamt; #1 shft_hold1 <= 1;end
//else begin shamt_tem <= 0; #1 shft_hold1 <= 0;end
end
end

assign clk_mem_haz = (clk_mem & ~hold_data_haz);

always @(posedge clk_mem)
begin

if((shamt_tem - shift_dec)  > 1)begin shift_dec <= shift_dec + 1;#1 shft_hold2 <= 1;#1 shft_hold1 <= 0; end
else if((shamt_tem - shift_dec ) == 1)
begin shift_dec <= 0;shamt_tem <= 0;#1 shft_hold2 <= 0; end
end


endmodule



//ex
module ex_cu(clk,opcode,fun3,shamt,sel_m3,sel_m2,sel_dm1,out_opcode,out_fun3,sel_shft,ex_sel_m7,hold2,clk_mod,clk_mod_rest,out_shamt,pc_cu,in_count_haz,out_count_haz);
input [6:0] opcode;
input [2:0] fun3;
input [4:0] shamt;
input [31:0] pc_cu;
input clk,clk_mod,clk_mod_rest;
output reg sel_m3;
output reg [1:0] sel_m2,sel_dm1;
output reg [6:0] out_opcode;
output reg [2:0] out_fun3;
output reg sel_shft;
output reg ex_sel_m7;
reg [4:0] shift_dec=0;
output reg hold2=0;
reg rst_hold2=0;
reg [1:0] count_hold = 0;
output reg [4:0] out_shamt;
reg [31:0] pc_reg_shft_haz=32'h0;
reg out_haz =0;
input [2:0] in_count_haz;
output  [2:0] out_count_haz;

always @(*)
begin

//mux7 
if(opcode == 7'b0100011) ex_sel_m7 <= 1; // i type
else ex_sel_m7 <= 0;  // s type 

// mux3
if(opcode == 7'b1101111) sel_m3 <= 1; // j type 
else sel_m3 <= 0;  // u tyoe and for rest all 
//mux2
case(opcode)
7'b1100011  : sel_m2 <= 2'b00; //B  type 
7'b0110011  : begin if(fun3 == 3'b001 || fun3 == 3'b101)sel_m2 <= 2'b10; else sel_m2 <= 2'b00; end // and R type
7'b0110111, 7'b0010111, 7'b1101111: sel_m2 <= 2'b01; // u tpe and J type
default: sel_m2 <= 2'b10; //so for i,s type and rest all it will be 10
endcase
//demux1
case (opcode)
7'b1101111, 7'b1100111 : sel_dm1 <= 2'b01; // j , jalr
7'b0100011, 7'b0000011 : sel_dm1 <= 2'b10; // s type and itype load 
7'b1100011 : sel_dm1 = 2'b11; // B type
default: sel_dm1 = 2'b00; 
endcase


if(opcode == 7'b0100011 || opcode == 7'b0000011) rst_hold2 <= 0;
//else rst_hold2 <= 1;



end
 
always @(*) begin
if(pc_reg_shft_haz != pc_cu) begin  shift_dec <= 0;sel_shft <=1; out_haz <=1; end
else out_haz <=0;
end

assign out_count_haz = in_count_haz;

always @(posedge clk_mod)
begin
out_shamt <= shamt;
out_opcode <=  opcode;
out_fun3 <= fun3;


if((opcode == 7'b0010011 || opcode == 7'b0110011 ) && (fun3 == 3'b001 || fun3 == 3'b101)) // shift operation
begin

pc_reg_shft_haz <= pc_cu;

if((shamt - shift_dec) <= 1)begin sel_shft <= 1;shift_dec <= 0; end
else 

begin shift_dec <= shift_dec + 1;sel_shft <=0; end

end
else begin  shift_dec <= 0;sel_shft <=1; end

end


always @(posedge clk ) begin


// this will give hold signal for clk which is used in the next stage

if(opcode == 7'b0100011 || opcode == 7'b0000011) // store and load type // if 2 same instructions are there then count_in might change 
begin

if(opcode == 7'b0100011 ) begin //store type 

case(fun3)
3'b010 :begin if(count_hold == 0)begin #1 hold2 <= 1; count_hold <= 3; end else begin count_hold <= count_hold -1;hold2 <= 0; end end// sw  #1 models practical comb delay
//3'b010 : #1 hold2 <= 1;// sw
3'b001 :begin if(count_hold == 0)begin #1 hold2 <= 1; count_hold <= 1; end else begin count_hold <= count_hold -1;hold2 <= 0; end end// sh
default :begin hold2 <= 0;count_hold <= 0; end/// sb
endcase


end
else begin  // load type
case(fun3)
3'b001 : begin if(count_hold == 0)begin #1 hold2 <= 1; count_hold <= 1; end else begin count_hold <= count_hold -1;hold2 <= 0; end end//lh
3'b010 : begin if(count_hold == 0)begin #1 hold2 <= 1; count_hold <= 3; end else begin count_hold <= count_hold -1;hold2 <= 0; end end//lw
3'b100 : hold2 <= 0;//lbu
3'b101 : begin if(count_hold == 0)begin #1 hold2 <= 1; count_hold <= 1; end else begin count_hold <= count_hold -1;hold2 <= 0; end end //lhu
default :begin hold2 <= 0;
count_hold <= 0; 
end//lb
endcase

end


end


else begin hold2 <= 0; count_hold <= 0; end
end

endmodule



// mem

module  mem_cu (clk_out_rest,opcode,fun3,count_in,count_mod,cs_en,rd_wrbar,sign_unsignbar,clk,hold_out,out_opcode,sel_m5,hold2,shamt,out_shamt,out_fun3,wr_en_shft,ex_mem_inc_npc_cu,in_count_haz);
input [6:0] opcode;
input [2:0] fun3;
input clk;
output reg [1:0] count_in;
output [1:0] count_mod;
output reg rd_wrbar,sign_unsignbar;
output cs_en;
reg hold = 0;
output  hold_out;
input hold2;
reg [2:0] count_mod_new;
reg [1:0]count_dec,count_in_tem;
output reg [6:0] out_opcode;
output reg sel_m5;
reg cs;
input [4:0] shamt;
output reg [4:0] out_shamt;
output reg [2:0] out_fun3;
reg [4:0] count_shft_reg_hold=0;
output reg  wr_en_shft ;
reg [1:0] branch_count_mem_hold=0;
reg branch_mem_hold =0;
input [31:0] ex_mem_inc_npc_cu;
input clk_out_rest;
input [2:0] in_count_haz;
reg [2:0] count_haz_tem =0 ;

assign hold_out = hold | hold2 ;
assign count_mod = count_in - count_dec; // count_in_tem is used bcz for sw inst for 4th byte storing count_in becoming 0 as the clk comes and count_in moves forward i.e. nxt wb stage
assign cs_en = cs & ~clk & ~branch_mem_hold; // so ths cs_en for mem will set at neg edge instead of posedge

always @(posedge clk_out_rest) begin

if(opcode ==7'b1101111 || opcode == 7'b1100111 || (opcode == 7'b1100011 && ex_mem_inc_npc_cu == 1)) //branch type inst. mem hold generation so that 
begin branch_count_mem_hold <= 3;branch_mem_hold <=1;end
else begin
if(branch_count_mem_hold > 0) begin branch_count_mem_hold <= branch_count_mem_hold -1;branch_mem_hold <=1;end
else begin branch_count_mem_hold <= 0;branch_mem_hold <=0;end
end

end

always @(*)
begin


if(opcode == 7'b0000011) sel_m5 <= 1;
else sel_m5 <= 0;

if(opcode == 7'b0100011 || opcode == 7'b0000011) // store and load type // if 2 same instructions are there then count_in might change 
begin
cs <= 1'b1;



if(opcode == 7'b0100011 ) begin //store type 
rd_wrbar <= 1'b0; 
sign_unsignbar <= 1'b1;

case(fun3)
3'b010 :begin  count_in<= 2'b11; end // sw
3'b001 :begin  count_in<= 2'b01;end // sh
default :begin count_in<= 2'b00;end // sb
endcase


end
else begin  // load type
rd_wrbar <= 1'b1; 

case(fun3)
3'b001 :begin  count_in<= 2'b01;sign_unsignbar <= 1'b1; end //lh
3'b010 :begin  count_in<= 2'b11;sign_unsignbar <= 1'b1; end //lw
3'b100 :begin  count_in<= 2'b00;sign_unsignbar <= 1'b0; end //lbu
3'b101 :begin  count_in<= 2'b01;sign_unsignbar <= 1'b0; end //lhu
default :begin  count_in<= 2'b00;sign_unsignbar <= 1'b1; end //lb
endcase

end


end


else begin
cs <= 1'b0;
count_in <= 2'b00;
//count_mod <= 2'b00;
rd_wrbar <= 1'b1;
sign_unsignbar <= 1'b0;

end


end

always @(posedge clk) begin  // wr_en for reg is made low during the shfting is going and during the data hazard
if((opcode == 7'b0010011 || opcode == 7'b0110011 ) && (fun3 == 3'b001 || fun3 == 3'b101)) 
begin

if((shamt + count_haz_tem  - count_shft_reg_hold ) > 1)begin  count_shft_reg_hold <= count_shft_reg_hold + 1;wr_en_shft <= 0;  end
else begin 

count_shft_reg_hold<=0;
wr_en_shft <= 1;
if(in_count_haz > 0)
count_haz_tem <= in_count_haz-1;  
else count_haz_tem <= 0; 
end
end
else begin count_shft_reg_hold<=0;wr_en_shft <= 1; 
if(in_count_haz > 0)
count_haz_tem <= in_count_haz - 1;  
else count_haz_tem <= 0; 
end
end

always @(posedge clk) begin
out_shamt <= shamt; 
out_opcode <= opcode;
count_in_tem <= count_in;
out_fun3 <= fun3;

if(count_in > 0 && count_dec < (count_in))begin count_dec <= count_dec + 1; hold <= 1; end 
//else if(count_in > 0 && count_dec ==  (count_in - 2'b01))begin count_dec <= count_dec + 1;hold <= 1; end
else begin hold <= 0; count_dec<=0;   end 
end 
//always @(count_mod) begin 
//count_mod_new <= count_mod -1; 
//end 
 
endmodule 
 
//wb 
 
module wb_cu(inc_pc,inc_pc_new_noadd,inc_pc_new,clk_out_rest,clk,opcode,sel_m6,wr_en,shamt,fun3,wr_en_shft,wb_inc_npc_cu); 
output reg inc_pc=1,inc_pc_new_noadd=0,inc_pc_new=0;
input [6:0] opcode; 
output reg sel_m6;
output reg wr_en;
input [4:0] shamt;
input [2:0] fun3;
input clk,clk_out_rest;
input wr_en_shft;
input [31:0] wb_inc_npc_cu;
reg [1:0] branch_count_reg_hold = 0;
reg branch_reg_hold =0;

always @(posedge clk_out_rest) begin

if(opcode ==7'b1101111 || opcode == 7'b1100111 || (opcode == 7'b1100011 && wb_inc_npc_cu == 1)) //branch type inst. mem hold generation so that 
begin branch_count_reg_hold <= 3;branch_reg_hold <=1;end
else begin
if(branch_count_reg_hold > 0) begin branch_count_reg_hold <= branch_count_reg_hold -1;branch_reg_hold <=1;end
else begin branch_count_reg_hold <= 0;branch_reg_hold <=0;end
end

end

always @(*)
begin

// pc changing due to branch and jump inst.
if(opcode == 7'b1101111 || opcode ==7'b1100111) begin inc_pc<=0;inc_pc_new_noadd<=1;inc_pc_new<=0; end // jump
else if(opcode == 7'b1100011 && wb_inc_npc_cu == 1) begin inc_pc<=0;inc_pc_new_noadd<=0;inc_pc_new<=1; end //branch
else begin inc_pc<=1;inc_pc_new_noadd<=0;inc_pc_new<=0; end

if(opcode == 7'b1101111 || opcode == 7'b1100111) sel_m6 <= 1;
else sel_m6 <= 0;

if(opcode == 7'b0100011)wr_en <= 0;
else wr_en <= 1 & wr_en_shft & ~branch_reg_hold;
end


endmodule
