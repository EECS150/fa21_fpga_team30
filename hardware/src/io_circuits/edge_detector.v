module edge_detector #(
    parameter WIDTH = 1
)(
    input clk,
    input [WIDTH-1:0] signal_in,
    output [WIDTH-1:0] edge_detect_pulse
);
    reg [WIDTH-1 :0] signal_in_ff1;
    
    always @(posedge clk) begin
        signal_in_ff1 <= signal_in;
    end
    
    assign edge_detect_pulse = signal_in & ~signal_in_ff1;
endmodule
