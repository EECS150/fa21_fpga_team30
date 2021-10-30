module Instruction_Decode(
	input clk,
	input rst,
	input [31:0] Inst_Fetch,
	input [13:0] PC_addr_Fetch,
	input wr_en,
	input [4:0] Addr_D,
	input [31:0] Data_D,
	output reg [31:0] Data_A,
	output reg [31:0] Data_B,
	output reg [13:0] PC_addr_Decode,
	output reg [31:0] Inst_Decode
);
    
    // Register file
    // Asynchronous read: read data is available in the same cycle
    // Synchronous write: write takes one cycle
    //wire we;
    wire [4:0] ra1, ra2, wa;
    wire [31:0] wd;
    wire [31:0] rd1, rd2;
    reg_file rf (
        .clk(clk),
        .rst(rst),
        .we(wr_en),
        .ra1(ra1), .ra2(ra2), .wa(Addr_D),
        .wd(Data_D),
        .rd1(rd1), .rd2(rd2)
    );

    assign ra1 = Inst_Fetch[19:15];
    assign ra2 = Inst_Fetch[24:20];

    always @(posedge clk) begin
    	if (rst) begin
    		Data_A <= 0;
    	end
    	else begin
    		Data_A <= rd1;
    	end
    end

    always @(posedge clk) begin
    	if (rst) begin
    		Data_B <= 0;
    	end
    	else begin
    		Data_B <= rd2;
    	end
    end

    always @(posedge clk) begin
        if (rst) begin
            PC_addr_Decode <= 0;
        end
        else begin
            PC_addr_Decode <= PC_addr_Fetch;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            Inst_Decode <= 0;
        end
        else begin
            Inst_Decode <= Inst_Fetch;
        end
    end

endmodule