module synchronizer #(parameter WIDTH = 1) (
    input [WIDTH-1:0] async_signal,
    input clk,
    output reg [WIDTH-1:0] sync_signal
);
    reg [WIDTH-1 :0] inter;

    always @(posedge clk) begin
        inter <= async_signal;
        sync_signal <= inter;
    end
    
endmodule
