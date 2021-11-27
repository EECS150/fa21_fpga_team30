module nco(
    input clk,
    input rst,
    input [23:0] fcw,
    input next_sample,
    output [13:0] sample
);
    wire [7:0] index;


    sine_lut sine_lut(
        .address(index),
        .data(sample)
    );

    reg [23:0] cnt = 0;

    always @(posedge clk) begin
        if(rst) begin
            cnt <= 0;
        end
        else if(next_sample) begin
            cnt <= cnt + fcw;
        end
        else begin
            cnt <= cnt;
        end
    end
    
    assign index = cnt[23:23-7];
endmodule
