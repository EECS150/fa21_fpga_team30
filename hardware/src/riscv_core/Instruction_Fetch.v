module Instruction_Fetch (
	input clk,
	input rst,
	input Hold,
	input PCSel,
	input [31:0] ALU,
	input [31:0] base_addr,
	output reg [31:0] PC_addr,
	output reg [31:0] mem_addr
);
	
	always @(posedge clk) begin
		if (rst) begin
			PC_addr <= base_addr - 32'd4;
		end
		else begin 
			PC_addr <= mem_addr;
		end
	end
	
	// PCSel = 1 -> ALU / 0 -> +4
	always @(*) begin
		case({Hold,PCSel})
			2'b00: mem_addr = PC_addr + 32'd4;
			2'b10: mem_addr = PC_addr;
			2'b01, 2'b11: mem_addr = ALU;
		endcase
	end 

endmodule