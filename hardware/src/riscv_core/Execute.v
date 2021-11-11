module Execute (
	input clk,
	input rst,
	input [31:0] Data_A,
	input [31:0] Data_B,
	input [31:0] Data_D,
	input [31:0] Data_D_ff1,
	input [31:0] PC_addr_Decode,
	input [31:0] Inst_Decode,
	input [1:0]  Data_ASel,
	input [1:0]  Data_BSel,
	input ASel,
	input BSel,
	input [3:0] ALUSel,
	input [2:0] ImmSel,
	input BrUn,
	output BrEq,
	output BrLT,
	output [31:0] ALU_out,
	output reg [31:0] ALU_out_reg,
	output reg [31:0] PC_addr_Execute,
	output [31:0] Data_W,
	output reg [31:0] Inst_Execute
);
    localparam REG = 2'b00;
    localparam DATA_D = 2'b10;
    localparam DATA_D_ff1 = 2'b11;

	reg [31:0] Data_A_mux, Data_B_mux;

	always @(*) begin
		case(Data_ASel)
			REG:Data_A_mux = Data_A;
			DATA_D:Data_A_mux = Data_D;
			DATA_D_ff1:Data_A_mux = Data_D_ff1;
			default: Data_A_mux = 2'bx;
		endcase
	end

	always @(*) begin
		case(Data_BSel)
			REG:Data_B_mux = Data_B;
			DATA_D:Data_B_mux = Data_D;
			DATA_D_ff1:Data_B_mux = Data_D_ff1;
			default: Data_B_mux = 2'bx;
		endcase
	end

	wire [31:0] ALU_Data_A, ALU_Data_B;
	wire [31:0] Imm;

	assign ALU_Data_A = ASel ? PC_addr_Decode : Data_A_mux;
	assign ALU_Data_B = BSel ? Imm : Data_B_mux;	
	
	Imm_Gen Imm_Gen(
		.Inst(Inst_Decode),
		.ImmSel(ImmSel),
		.Imm(Imm)
	);

	ALU ALU (
    	.Data_A(ALU_Data_A),
    	.Data_B(ALU_Data_B),
    	.ALUSel(ALUSel),
    	.ALU_out(ALU_out)
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