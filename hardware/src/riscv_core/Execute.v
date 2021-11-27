module Execute (
	input clk,
	input rst,
	input [31:0] Data_A_mux,
	input [31:0] Data_B_mux,
	input [31:0] PC_addr_Decode,
	input [31:0] Inst_Decode,
	input ASel,
	input BSel,
	input [3:0] ALUSel,
	input [31:0] Imm,
	input BrUn,
	output BrEq,
	output BrLT,
	output [31:0] ALU_out,
	output [31:0] Add_out,
	output reg [31:0] ALU_out_reg,
	output reg [31:0] PC_addr_Execute,
	output [31:0] Data_W,
	output reg [31:0] Inst_Execute
);

	wire [31:0] ALU_Data_A, ALU_Data_B;

	assign ALU_Data_A = ASel ? PC_addr_Decode : Data_A_mux;
	assign ALU_Data_B = BSel ? Imm : Data_B_mux;	

	ALU ALU (
    	.Data_A(ALU_Data_A),
    	.Data_B(ALU_Data_B),
    	.ALUSel(ALUSel),
    	.ALU_out(ALU_out),
    	.Add_out(Add_out)
	);	

	Branch_Comp Branch_Comp(
    .Data_A(Data_A_mux),
    .Data_B(Data_B_mux),
    .BrUn(BrUn),
    .BrEq(BrEq),
    .BrLT(BrLT)
	);

	always @(posedge clk) begin
		if(rst) begin
			ALU_out_reg <= 0;
		end
		else begin
			ALU_out_reg <= ALU_out;
		end
	end

	always @(posedge clk) begin
		if(rst) begin
			PC_addr_Execute <= 0;
		end
		else begin
			PC_addr_Execute <= PC_addr_Decode;
		end
	end	

	assign Data_W = Data_B_mux;

	always @(posedge clk) begin
		if(rst) begin
			Inst_Execute <= 0;
		end
		else begin
			Inst_Execute <= Inst_Decode;
		end
	end	
endmodule