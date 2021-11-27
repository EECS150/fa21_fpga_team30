module reg_file (
    input clk,
    input rst,
    input we,
    input [4:0] ra1, ra2, wa,
    input [31:0] wd,
    output [31:0] rd1, rd2
);
    parameter DEPTH = 32;
    reg [31:0] mem [0:31];
    assign rd1 = mem[ra1];
    assign rd2 = mem[ra2];

    always @(posedge clk) begin
        if(rst) begin  
            mem[0] <= 32'b0;
        end
    end

    genvar i;
    generate
        for(i = 1; i < 32 ; i = i+1) begin : REG_FILE
            always @(posedge clk) begin
                if(i == wa && we) begin
                    mem[i] <= wd;                    
                end
            end
        end
    endgenerate


    wire [31:0] reg_1;
    wire [31:0] reg_2;
    wire [31:0] reg_3;
    wire [31:0] reg_4;
    wire [31:0] reg_5;
    wire [31:0] reg_6;
    wire [31:0] reg_7;
    wire [31:0] reg_8;
    wire [31:0] reg_9;
    wire [31:0] reg_10;
    wire [31:0] reg_11;
    wire [31:0] reg_12;
    wire [31:0] reg_13;
    wire [31:0] reg_14;
    wire [31:0] reg_15;
    wire [31:0] reg_16;
    wire [31:0] reg_17;
    wire [31:0] reg_18;
    wire [31:0] reg_19;
    wire [31:0] reg_20;
    wire [31:0] reg_21;
    wire [31:0] reg_22;
    wire [31:0] reg_23;
    wire [31:0] reg_24;
    wire [31:0] reg_25;
    wire [31:0] reg_26;
    wire [31:0] reg_27;
    wire [31:0] reg_28;
    wire [31:0] reg_29;
    wire [31:0] reg_30;
    wire [31:0] reg_31;

    assign reg_1  = mem[1];
    assign reg_2  = mem[2];
    assign reg_3  = mem[3];
    assign reg_4  = mem[4];
    assign reg_5  = mem[5];
    assign reg_6  = mem[6];
    assign reg_7  = mem[7];
    assign reg_8  = mem[8];
    assign reg_9  = mem[9];
    assign reg_10 = mem[10];
    assign reg_11 = mem[11];
    assign reg_12 = mem[12];
    assign reg_13 = mem[13];
    assign reg_14 = mem[14];
    assign reg_15 = mem[15];
    assign reg_16 = mem[16];
    assign reg_17 = mem[17];
    assign reg_18 = mem[18];
    assign reg_19 = mem[19];
    assign reg_20 = mem[20];
    assign reg_21 = mem[21];
    assign reg_22 = mem[22];
    assign reg_23 = mem[23];
    assign reg_24 = mem[24];
    assign reg_25 = mem[25];
    assign reg_26 = mem[26];
    assign reg_27 = mem[27];
    assign reg_28 = mem[28];
    assign reg_29 = mem[29];
    assign reg_30 = mem[30];
    assign reg_31 = mem[31];    


endmodule
