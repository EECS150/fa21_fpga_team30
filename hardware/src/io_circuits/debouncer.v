module debouncer #(
    parameter WIDTH              = 1,
    parameter SAMPLE_CNT_MAX     = 62500,
    parameter PULSE_CNT_MAX      = 200,
    parameter WRAPPING_CNT_WIDTH = $clog2(SAMPLE_CNT_MAX),
    parameter SAT_CNT_WIDTH      = $clog2(PULSE_CNT_MAX) + 1
) (
    input clk,
    input [WIDTH-1:0] glitchy_signal,
    output [WIDTH-1:0] debounced_signal
);

    wire sample_pulse_gen;

    reg [SAT_CNT_WIDTH-1:0] saturating_counter [WIDTH-1:0];
    integer k;
    initial begin
        for(k=0; k<WIDTH; k=k+1) begin
            saturating_counter[k] = 0;
        end
    end

    reg [WRAPPING_CNT_WIDTH-1:0] cnt_wrap =0;

    always @(posedge clk) begin
        if (cnt_wrap < SAMPLE_CNT_MAX) begin
            cnt_wrap <= cnt_wrap + 1;
        end
        else begin
            cnt_wrap <= 0;
        end
    end

    assign sample_pulse_gen = cnt_wrap == SAMPLE_CNT_MAX;

    genvar i;
    generate
        for(i=0; i<WIDTH; i=i+1) begin:debouncer_reg
            always @(posedge clk) begin
                if (glitchy_signal[i] == 0) begin
                    saturating_counter[i] <= 0; 
                end
                else if (sample_pulse_gen && saturating_counter[i] < PULSE_CNT_MAX) begin
                    saturating_counter[i] <= saturating_counter[i] + 1;
                end
                else begin
                    saturating_counter[i] <= saturating_counter[i];
                end
            end

            assign debounced_signal[i] = (saturating_counter[i] == PULSE_CNT_MAX);    
        end

        
    endgenerate
endmodule
