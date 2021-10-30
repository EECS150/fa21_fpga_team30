`timescale 1ns/1ns
module ALU_tb ();
	localparam CPU_CLOCK_PERIOD = 20;
	reg clk;
	initial clk = 0;
	always #(CPU_CLOCK_PERIOD/2) clk = ~clk;
	reg [31:0] data_A, data_B, ALU_out;
	reg [3:0] ALUsel;

	ALU ALU(
	.Data_A(data_A),
	.Data_B(data_B),
	.ALU_out(ALU_out),
	.ALUSel(ALUsel)
	);

    localparam ADD = 4'b0000;
    localparam SUB = 4'b1000;
    localparam SLL = 4'b0001;
    localparam SLT = 4'b0010;
    localparam SLTU = 4'b0011;
    localparam XOR = 4'b0100;
    localparam SRL = 4'b0101;
    localparam SRA = 4'b1101;
    localparam OR = 4'b0110;
    localparam AND = 4'b0111;  

    initial begin
        `ifndef IVERILOG
            $vcdpluson;
        `endif
        `ifdef IVERILOG
            $dumpfile("system_tb.fst");
            $dumpvars(0, system_tb);
        `endif

        data_A = 32'hF0002323;
        data_B = 32'h12345678;
        ALUsel = ADD;
        repeat (5) @(posedge clk); #1;
        ALUsel = SUB;
        repeat (5) @(posedge clk); #1;
        ALUsel = SLL;
        repeat (5) @(posedge clk); #1;
        ALUsel = SLT;
        repeat (5) @(posedge clk); #1;
        ALUsel = SLTU;
        repeat (5) @(posedge clk); #1;
        ALUsel = XOR;
        repeat (5) @(posedge clk); #1;        
        ALUsel = SRL;
        repeat (5) @(posedge clk); #1;
        ALUsel = SRA;
        repeat (5) @(posedge clk); #1;
        ALUsel = OR;
        repeat (5) @(posedge clk); #1;
        ALUsel = AND;
        repeat (5) @(posedge clk); #1;


        `ifndef IVERILOG
            $vcdplusoff;
        `endif
        $finish();
    end

endmodule