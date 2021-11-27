module sampler (
    input clk,
    input rst,
    input synth_valid,
    input [9:0] scaled_synth_code,
    output synth_ready,
    output reg pwm_out
);
    localparam CYCLES_PER_WINDOW=1024;
    localparam CODE_WIDTH = $clog2(CYCLES_PER_WINDOW);
    localparam CYCLES_PER_SAMPLE=2500;
    localparam CYCLES_PER_SAMPLE_WIDTH = $clog2(CYCLES_PER_SAMPLE);
    reg [9:0] code;

    reg[CODE_WIDTH-1:0] cnt_cycle=0;
    reg[CYCLES_PER_SAMPLE_WIDTH-1:0] cnt_cycle_sample=0;    


    always @(posedge clk) begin
        if(rst) begin
            code <= 10'b0;
        end
        else if(synth_valid) begin
            code <= scaled_synth_code;
        end
        else begin
            code <= code;
        end
    end    

    always @(posedge clk) begin
        if(rst) begin
            cnt_cycle <= 0;
        end
        else if (cnt_cycle < CYCLES_PER_WINDOW - 1) begin
            cnt_cycle <= cnt_cycle + 1;
        end
        else begin
            cnt_cycle <= 0;
        end
    end
    
    always @(posedge clk) begin
        if(rst) begin
            pwm_out <= 0;
        end
        else if (cnt_cycle < code) begin
            pwm_out <= 1;
        end
        else begin
            pwm_out <= 0;
        end
    end

    always @(posedge clk) begin
        if(rst) begin
            cnt_cycle_sample <= 0;
        end
        else if (cnt_cycle < CYCLES_PER_SAMPLE - 1) begin
            cnt_cycle_sample <= cnt_cycle_sample + 1;
        end
        else begin
            cnt_cycle_sample <= 0;
        end
    end

    assign synth_ready = cnt_cycle_sample == (CYCLES_PER_SAMPLE - 2);

endmodule