module sigma_delta_dac #(
    parameter CODE_WIDTH = 10
)(
    input clk,
    input rst,
    input [CODE_WIDTH-1:0] code,
    output pwm
);
    reg [CODE_WIDTH:0] sum_reg;
    wire [CODE_WIDTH:0] sum;
    always @(posedge clk) begin
        if(rst) begin
            sum_reg <= 0; 
        end
        else begin
            sum_reg <= sum; 
        end
    end

    assign sum = sum_reg + {1'b0, code};
    // Remove this line once you have implemented this module
    assign pwm = sum[CODE_WIDTH] ^ sum_reg[CODE_WIDTH];
endmodule
