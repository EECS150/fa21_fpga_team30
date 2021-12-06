module synth #(
    parameter N_VOICES = 1
)(
    input clk,
    input rst,
    input [N_VOICES-1:0] [23:0] carrier_fcws,
    input [23:0] mod_fcw,
    input [4:0] mod_shift,
    input [N_VOICES-1:0] note_en,

    output reg [13:0] sample,
    output sample_valid,
    input sample_ready
);
    wire [N_VOICES-1:0][13:0] mod_samp;
    wire [N_VOICES-1:0][23:0] carrier_fcw_modulated;
    reg [N_VOICES-1:0][23:0] carrier_fcw_modulated_reg;
    wire [N_VOICES-1:0][13:0] carrier_sample;
    wire [N_VOICES-1:0][23:0] mod_fcw_input;
    wire [N_VOICES-1:0][23:0] carrier_fcw_input;

    reg sample_valid_reg_ff1;
    reg sample_valid_reg_ff2;
    reg sample_valid_reg_ff3;
    reg sample_valid_reg_ff4;
    reg [13:0] sample_sum;

    genvar i;
    generate
        for(i=0;i<N_VOICES;i=i+1) begin : SYNTH_NCO
            assign mod_fcw_input[i] = note_en[i] ? mod_fcw : 24'b0;
            nco modulator_nco(
                .clk(clk),
                .rst(rst),
                .fcw(mod_fcw_input[i]),
                .next_sample(sample_ready),
                .sample(mod_samp[i])
            );
        
            assign carrier_fcw_modulated[i] = carrier_fcws[i] + (mod_samp[i] << mod_shift);

            always @(posedge clk) begin
                if(rst) begin
                    carrier_fcw_modulated_reg[i] <= 24'b0;
                end
                else begin
                    carrier_fcw_modulated_reg[i] <= carrier_fcw_modulated[i];
                end
            end
            assign carrier_fcw_input[i] = note_en[i] ? carrier_fcw_modulated_reg[i] : 24'b0;
            
            nco carrier_nco(
                .clk(clk),
                .rst(rst),
                .fcw(carrier_fcw_input[i]),
                .next_sample(sample_valid_reg_ff2),
                .sample(carrier_sample[i])
            );
        end
        
    endgenerate

    always @(*) begin
        case(N_VOICES)
            1:sample_sum = (note_en[0] ? carrier_sample[0] : 14'b0);
            2:sample_sum = (note_en[0] ? carrier_sample[0] : 14'b0) + (note_en[1] ? carrier_sample[1] : 14'b0);
            3:sample_sum = (note_en[0] ? carrier_sample[0] : 14'b0) + (note_en[1] ? carrier_sample[1] : 14'b0) + (note_en[2] ? carrier_sample[2] : 14'b0);
            default:sample_sum = (note_en[0] ? carrier_sample[0] : 14'b0) + (note_en[1] ? carrier_sample[1] : 14'b0) + (note_en[2] ? carrier_sample[2] : 14'b0) + (note_en[3] ? carrier_sample[3] : 14'b0);
        endcase
    end

    always @(posedge clk)begin
        if(rst) begin
            sample <= 0;
        end
        else if(sample_valid_reg_ff3) begin
            sample <= sample_sum;
        end
    end

    always @(posedge clk) begin
        if(rst) begin
            sample_valid_reg_ff1 <= 1'b0;
            sample_valid_reg_ff2 <= 1'b0;
            sample_valid_reg_ff3 <= 1'b0;
            sample_valid_reg_ff4 <= 1'b0;
        end
        else begin
            sample_valid_reg_ff1 <= sample_ready;
            sample_valid_reg_ff2 <= sample_valid_reg_ff1;
            sample_valid_reg_ff3 <= sample_valid_reg_ff2;
            sample_valid_reg_ff4 <= sample_valid_reg_ff3;
        end
    end

    assign sample_valid = sample_valid_reg_ff4;
endmodule