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
        for(i = 1; i < 32 ; i++) begin
            always @(posedge clk) begin
                if(rst) begin  
                    mem[i] <= 32'b0;
                end
                else if(i == wa && we) begin
                    mem[i] <= wd;                    
                end
            end
        end
    endgenerate

endmodule
