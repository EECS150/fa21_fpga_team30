module iomem (
    input clk,
    input en,
    input [3:0] we,
    input [13:0] addr,
    input [31:0] din,
    output reg [31:0] dout
);
    parameter DEPTH = 6;
    reg [31:0] mem [6-1:0];

//mem[0] UART control
//mem[1] UART receiver data
//mem[2] UART transmitter data
//mem[3] Cycle counter
//mem[4] Instruction counter
//mem[5] Reset counters to 0
/*
    wire rst, RST;
    assign RST = mem[5] != 0;
    always @(posedge clk) begin
        if(rst) begin
            mem[0] <= 32'b0;
        end
        else begin
            mem[0] <= 32'b0;
        end
    end

    always @(posedge clk) begin
        if(rst) begin
            mem[1] <= 32'b0;
        end
        else begin
            mem[1] <= 32'b0;
        end
    end

    genvar i;
    generate for (i = 0; i < 4; i = i+1) begin
        always @(posedge clk) begin
            if (rst) begin
                mem[2][i*8 +: 8] <= 8'b0;
            end
            if (addr = 2 && we[i] && en) begin
                mem[2][i*8 +: 8] <= din[i*8 +: 8];
            end
            else begin
                mem[2][i*8 +: 8] <= mem[2][i*8 +: 8];
            end
        end
    end endgenerate

    always @(posedge clk) begin
        if(rst || RST) begin
            mem[3] <= 32'b0;
        end
        else begin
            mem[3] <= mem[3] + 'd1;
        end
    end

    always @(posedge clk) begin
        if(rst || RST) begin
            mem[4] <= 32'b0;
        end
        else if(control_hazard || Hold) begin
            mem[4] <= mem[4];
        end
        else begin
            mem[4] <= mem[4] + 'd1;
        end
    end

    genvar i;
    generate for (i = 0; i < 4; i = i+1) begin
        always @(posedge clk) begin
            if (rst) begin
                mem[5][i*8 +: 8] <= 8'b0;
            end
            if (addr = 5 && we[i] && en) begin
                mem[5][i*8 +: 8] <= din[i*8 +: 8];
            end
            else begin
                mem[5][i*8 +: 8] <= mem[5][i*8 +: 8];
            end
        end
    end endgenerate
*/
    always @(posedge clk) begin
        if (en) begin
            dout <= mem[addr];
        end
    end
    
    genvar i;
    generate for (i = 0; i < 4; i = i+1) begin
        always @(posedge clk) begin
            if (we[i] && en) begin
                mem[addr][i*8 +: 8] <= din[i*8 +: 8];
            end
        end
    end endgenerate
endmodule