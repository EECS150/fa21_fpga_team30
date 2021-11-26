module fifo #(
    parameter WIDTH = 8,
    parameter DEPTH = 32,
    parameter POINTER_WIDTH = $clog2(DEPTH)
) (
    input clk, rst,

    // Write side
    input wr_en,
    input [WIDTH-1:0] din,
    output full,

    // Read side
    input rd_en,
    output [WIDTH-1:0] dout,
    output empty
);
    reg [POINTER_WIDTH:0] cnt;
    reg [POINTER_WIDTH:0] wr_ptr, rd_ptr;
    reg [WIDTH-1:0] fifo_mem [DEPTH-1:0];

    localparam S0 = 2'b00;
    localparam S1 = 2'b01;
    localparam S2 = 2'b10;
    localparam S3 = 2'b11;
    wire [1:0] state;
    assign state[0] = wr_en && !full;
    assign state[1] = rd_en && !empty;

    always @(posedge clk) begin
        if(rst) begin
            cnt <= 0;
            wr_ptr <= 0;
            rd_ptr <= 0;
        end
        else begin
            case(state)
                S0: begin
                    cnt <= cnt;
                    wr_ptr <= wr_ptr;
                    rd_ptr <= rd_ptr;                 
                end
                S1: begin
                    cnt <= cnt + 1'b1;
                    fifo_mem[wr_ptr] <= din;
                    wr_ptr <= wr_ptr == DEPTH-1 ? 0: wr_ptr+1;
                    rd_ptr <= rd_ptr;
                end
                S2: begin
                    cnt <= cnt - 1'b1;
                    wr_ptr <= wr_ptr;
                    rd_ptr <= rd_ptr == DEPTH-1 ? 0: rd_ptr+1;               
                end
                S3: begin
                    fifo_mem[wr_ptr] <= din;
                    wr_ptr <= wr_ptr == DEPTH-1 ? 0: wr_ptr+1;
                    rd_ptr <= rd_ptr == DEPTH-1 ? 0: rd_ptr+1;
                end
            endcase
        end
    end

    assign dout = state[1] ? fifo_mem[rd_ptr]: 8'b0;
    assign full = cnt == DEPTH;
    assign empty = cnt == 0;
endmodule
