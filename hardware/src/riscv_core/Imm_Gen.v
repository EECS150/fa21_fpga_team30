module Imm_Gen(
	input [31:0] Inst,
	input [2:0] ImmSel,
	output reg [31:0] Imm
);

	localparam I = 3'b000;
	localparam S = 3'b001;
	localparam B = 3'b010;
	localparam J = 3'b011;
	localparam U = 3'b100;
	localparam C = 3'b101;

	always @(*) begin
		case(ImmSel)
			I: Imm <= {{20{Inst[31]}},Inst[31:20]};
			S: Imm <= {{20{Inst[31]}},Inst[31:25],Inst[11:7]};
			B: Imm <= {{20{Inst[31]}},Inst[7],Inst[30:25],Inst[11:8],1'b0};
			J: Imm <= {{12{Inst[31]}},Inst[19:12],Inst[20],Inst[30:21],1'b0};
			U: Imm <= {Inst[31:12],{12'b0}};
			C: Imm <= {{27{1'b0}},Inst[19:15]};
		endcase
	end
endmodule