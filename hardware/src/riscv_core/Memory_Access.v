module Memory_Access (
	input [31:0] PC_addr_Execute,
	input [31:0] ALU_out_reg,
	input [31:0] DMem_out,
	input [31:0] Inst_Execute,
	input [2:0] LdSel,
	input [1:0] WBSel,
	output reg [31:0] Data_D,
	output [4:0] Addr_D
);
	localparam ALU    = 2'b00;
	localparam DMEM    = 2'b01;
	localparam PC_ADD4 = 2'b10;

	wire [31:0] Ld_out;

	Load_Extension Load_Extension(
		.DMem_Sel(ALU_out_reg[1:0]),
		.DMem_out(DMem_out),
		.LdSel(LdSel),
		.Ld_out(Ld_out)
	);

	always @(*) begin
		case(WBSel)
			ALU:  Data_D = ALU_out_reg;
			PC_ADD4: Data_D = PC_addr_Execute + 32'd4;
			DMEM: Data_D = Ld_out;
			default: Data_D = 32'bx;
		endcase
	end

	assign Addr_D = Inst_Execute[11:7];


endmodule