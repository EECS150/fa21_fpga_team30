module cpu_to_synth_cdc #(
    parameter N_VOICES = 1
)(
    input cpu_clk,
    input [N_VOICES-1:0] [23:0] cpu_carrier_fcws,
    input [23:0] cpu_mod_fcw,
    input [4:0] cpu_mod_shift,
    input [N_VOICES-1:0] cpu_note_en,
//    input [4:0] cpu_synth_shift,
    input cpu_req,
    output cpu_ack,

    input synth_clk,
    output [N_VOICES-1:0] [23:0] synth_carrier_fcws,
    output [23:0] synth_mod_fcw,
    output [4:0] synth_mod_shift,
    output [N_VOICES-1:0] synth_note_en
//    output [4:0] synth_synth_shift
);
    wire cpu_req_rx;
    reg cpu_req_rx_reg;
    wire synth_en;
    synchronizer synchronizer_RX(
        .async_signal(cpu_req),
        .clk(synth_clk),
        .sync_signal(cpu_req_rx)
    );

    always @(posedge synth_clk) begin
        cpu_req_rx_reg <= cpu_req_rx;
    end

    assign synth_en = cpu_req_rx_reg;

    synchronizer cpu_RX(
        .async_signal(cpu_req_rx_reg),
        .clk(cpu_clk),
        .sync_signal(cpu_ack)
    );

    reg [N_VOICES-1:0] [23:0] synth_carrier_fcws_reg;
    reg [23:0] synth_mod_fcw_reg;
    reg [4:0] synth_mod_shift_reg;
    reg [N_VOICES-1:0] synth_note_en_reg;
//    reg [4:0] synth_synth_shift_reg;    

    always @(posedge synth_clk) begin
        if(synth_en) begin
            synth_mod_fcw_reg <= cpu_mod_fcw;
            synth_mod_shift_reg <= cpu_mod_shift;
            synth_note_en_reg <= cpu_note_en;
//            synth_synth_shift_reg <= cpu_synth_shift;
        end
    end

    genvar i;
    generate
        for(i=0; i<N_VOICES; i=i+1)begin : SYNTH_CARRIER_FCW
            always @(posedge synth_clk)begin
                if(synth_en) begin
                    synth_carrier_fcws_reg[i] <= cpu_carrier_fcws[i];
                end
            end
        end
    endgenerate

    assign synth_carrier_fcws = synth_carrier_fcws_reg;
    assign synth_mod_fcw = synth_mod_fcw_reg;
    assign synth_mod_shift = synth_mod_shift_reg;
    assign synth_note_en = synth_note_en_reg;
//    assign synth_synth_shift = synth_synth_shift_reg;
endmodule