module control_unit_EX (
	input clk,
	input rst,
    input BrEq,
    input BrLT,
    input [31:0] Inst,
    input Hold_decode_reg,
	input [1:0] MemRW_decode_reg,
	input RegWen_decode_reg,
	input [2:0] LdSel_decode_reg,
	input [1:0] WBSel_decode_reg,
	input CSRSel_decode_reg,
	output [1:0] MemRW_EX,
	output reg RegWen_EX_reg,
	output reg [2:0] LdSel_EX_reg,
	output reg [1:0] WBSel_EX_reg,
	output reg CSRSel_EX_reg,
	output PCSel,
	output control_hazards
);

	localparam opcode_R     = 5'b01100;
    localparam opcode_I     = 5'b00100;
    localparam opcode_L     = 5'b00000;
    localparam opcode_S     = 5'b01000;
    localparam opcode_B     = 5'b11000;
    localparam opcode_JALR  = 5'b11001;
    localparam opcode_JAL   = 5'b11011;
    localparam opcode_AUIPC = 5'b00101;
    localparam opcode_LUI   = 5'b01101;
    localparam opcode_CSR   = 5'b11100;

    localparam BEQ  = 3'b000;
    localparam BNE  = 3'b001;
    localparam BLT  = 3'b100;
    localparam BGE  = 3'b101;
    localparam BLTU = 3'b110;
    localparam BGEU = 3'b111;

    localparam MEMRW_0 = 2'b00;
    localparam SW = 2'b01;
    localparam SH = 2'b10;
    localparam SB = 2'b11;

    wire [4:0] opcode;
    wire [3:0] funct3;
    assign opcode = Inst[6:2];
    assign funct3 = Inst[14:12];
	
	reg control_hazards_detect;
	reg control_hazards_reg, control_hazards_reg_ff1, control_hazards_reg_ff2;
	
	wire control_hazards;
	assign control_hazards = control_hazards_reg || control_hazards_reg_ff1 || control_hazards_reg_ff2;

	assign PCSel = control_hazards_reg;

	always @(*) begin
		if(Hold_decode_reg || control_hazards_reg || control_hazards_reg_ff1 || control_hazards_reg_ff2) begin
			control_hazards_detect = 0;
		end
		else if(opcode == opcode_B) begin
			case(funct3)
				BEQ : control_hazards_detect = BrEq;
				BNE : control_hazards_detect = !BrEq;
				BLT : control_hazards_detect = BrLT;
				BGE : control_hazards_detect = !BrLT;
				BLTU: control_hazards_detect = BrLT;
				BGEU: control_hazards_detect = !BrLT;
				default: control_hazards_detect = 0; 				
			endcase
		end
		else if(opcode == opcode_JALR || opcode == opcode_JAL) begin
			control_hazards_detect = 1;
		end
		else begin
			control_hazards_detect = 0; 
		end
	end
	
	always @(posedge clk) begin
		if(rst) begin
			control_hazards_reg <= 0;
			control_hazards_reg_ff1 <= 0;
			control_hazards_reg_ff2 <= 0;
		end
		else begin
			control_hazards_reg <= control_hazards_detect;
			control_hazards_reg_ff1 <= control_hazards_reg;
			control_hazards_reg_ff2 <= control_hazards_reg_ff1;
		end
	end
	
	assign MemRW_EX = (control_hazards_reg || control_hazards_reg_ff1 || control_hazards_reg_ff2) ? 2'b0 : MemRW_decode_reg;

	always @(posedge clk) begin
		if(rst) begin
			RegWen_EX_reg <= 0;
		end
		else if(control_hazards_reg || control_hazards_reg_ff1 || control_hazards_reg_ff2)begin
			RegWen_EX_reg <= 0;
		end
		else begin
			RegWen_EX_reg <= RegWen_decode_reg;
		end
	end

	always @(posedge clk) begin
		if(rst) begin
			LdSel_EX_reg <= 0;
		end
		else if(control_hazards_reg || control_hazards_reg_ff1 || control_hazards_reg_ff2)begin
			LdSel_EX_reg <= 0;
		end
		else begin
			LdSel_EX_reg <= LdSel_decode_reg;
		end
	end

	always @(posedge clk) begin
		if(rst) begin
			WBSel_EX_reg <= 0;
		end
		else if(control_hazards_reg || control_hazards_reg_ff1 || control_hazards_reg_ff2)begin
			WBSel_EX_reg <= 0;
		end
		else begin
			WBSel_EX_reg <= WBSel_decode_reg;
		end
	end

	always @(posedge clk) begin
		if(rst) begin
			CSRSel_EX_reg <= 0;
		end
		else if(control_hazards_reg || control_hazards_reg_ff1 || control_hazards_reg_ff2)begin
			CSRSel_EX_reg <= 0;
		end
		else begin
			CSRSel_EX_reg <= CSRSel_decode_reg;
		end
	end

endmodule