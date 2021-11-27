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
    wire [N_VOICES-1:0][13:0] carrier_sample;
//    wire [N_VOICES-1:0][13:0] sum_sample;
    wire [N_VOICES-1:0][23:0] mod_fcw_input;
    wire [N_VOICES-1:0][23:0] carrier_fcw_input;

    reg sample_valid_reg;

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
            assign carrier_fcw_input[i] = note_en[i] ? carrier_fcw_modulated[i] : 24'b0;
            nco carrier_nco(
                .clk(clk),
                .rst(rst),
                .fcw(carrier_fcw_input[i]),
                .next_sample(sample_ready),
                .sample(carrier_sample[i])
            );
        end
        
    endgenerate

    always @(*) begin
        case(N_VOICES)
            1:sample = (note_en[0] ? carrier_sample[0] : 14'b0);
            2:sample = (note_en[0] ? carrier_sample[0] : 14'b0) + (note_en[1] ? carrier_sample[1] : 14'b0);
            3:sample = (note_en[0] ? carrier_sample[0] : 14'b0) + (note_en[1] ? carrier_sample[1] : 14'b0) + (note_en[2] ? carrier_sample[2] : 14'b0);
            default:sample = (note_en[0] ? carrier_sample[0] : 14'b0) + (note_en[1] ? carrier_sample[1] : 14'b0) + (note_en[2] ? carrier_sample[2] : 14'b0) + (note_en[3] ? carrier_sample[3] : 14'b0);
        endcase
    end

    always @(posedge clk) begin
        if(rst) begin
            sample_valid_reg <= 1'b0;
        end
        else begin
            sample_valid_reg <= sample_ready;
        end
    end

    assign sample_valid = sample_valid_reg;
    //assign sample_valid = 0;
    //assign sample = 0;
endmodule