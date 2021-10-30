module DMem_pre (
	input [31:0] ALU_out,
	input [31:0] Data_W,
	input [1:0] MemRW_EX,
	input [31:0] PC_addr_Decode,
	output reg [31:0] Mem_Data_W,
	output [13:0] DMem_Data_addr,
	output reg [3:0]  DMem_WE,
	output [13:0] IMem_Data_addr,
	output reg [3:0]  IMem_WE,
	output [13:0] IO_Data_addr,
	output reg [3:0]  IO_WE,
	output [11:0] bios_Data_addr
);
    localparam MEMRW_0 = 2'b00;
    localparam SW = 2'b01;
    localparam SH = 2'b10;
    localparam SB = 2'b11;
    wire [3:0] addr_space;
    reg [3:0] Mem_WE;

    assign addr_space = ALU_out[31:28];

	assign DMem_Data_addr = ALU_out[15:2];
	assign IMem_Data_addr = ALU_out[15:2];
	assign IO_Data_addr = ALU_out[15:2];
	assign bios_Data_addr = ALU_out[13:2];

	always @(*) begin
		case(MemRW_EX)
			MEMRW_0:begin
				Mem_Data_W = Data_W;
				Mem_WE = 4'b0000;
			end
			SW:begin
				Mem_Data_W = Data_W;
				Mem_WE = 4'b1111;
			end
			SH:begin
				if(ALU_out[1] == 1'b0) begin
					Mem_Data_W = Data_W;
					Mem_WE = 4'b0011;
				end
				else begin
					Mem_Data_W = {Data_W[15:0],16'b0};
					Mem_WE = 4'b1100;
				end
			end
			SB:begin
				if(ALU_out[1:0] == 2'b00) begin
					Mem_Data_W = Data_W;
					Mem_WE = 4'b0001;
				end
				else if(ALU_out[1:0] == 2'b01)begin
					Mem_Data_W = {16'b0, Data_W[7:0], 8'b0};
					Mem_WE = 4'b0010;
				end
				else if(ALU_out[1:0] == 2'b10)begin
					Mem_Data_W = {8'b0, Data_W[7:0], 16'b0};
					Mem_WE = 4'b0100;
				end
				else if(ALU_out[1:0] == 2'b11)begin
					Mem_Data_W = {Data_W[7:0], 24'b0};
					Mem_WE = 4'b1000;
				end
			end
		endcase
	end

	assign DMem_WE = (addr_space == 4'b0001 || addr_space == 4'b0011)? Mem_WE : 4'b0;
	assign IMem_WE = ((addr_space == 4'b0010 || addr_space == 4'b0011) && PC_addr_Decode[30] == 1'b1)? Mem_WE:4'b0;
	assign IO_WE = (addr_space == 4'b1000)? Mem_WE:4'b0;
endmodule