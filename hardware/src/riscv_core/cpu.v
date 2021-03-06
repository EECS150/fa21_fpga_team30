module cpu #(
    parameter CPU_CLOCK_FREQ = 50_000_000,
    parameter RESET_PC = 32'h4000_0000,
    parameter BAUD_RATE = 115200
) (
    input clk,
    input rst,
    input serial_in,
    output serial_out,
    input [2:0] buttons,
    input [1:0] switches,
    output [5:0] LEDS,
    output [23:0] iomem_carrier_nco_1_fcw,
    output [23:0] iomem_carrier_nco_2_fcw,
    output [23:0] iomem_carrier_nco_3_fcw,
    output [23:0] iomem_carrier_nco_4_fcw,
    output [23:0] iomem_mod_nco_fcw,
    output [4:0] iomem_mod_nco_shift_term,
    output [3:0] iomem_note_enable_signal,
    output iomem_cpu_tx_req,
    input tx_ack
);
    // BIOS Memory
    // Synchronous read: read takes one cycle
    // Synchronous write: write takes one cycle
    wire [11:0] bios_addra, bios_addrb;
    wire [31:0] bios_douta, bios_doutb;
    wire bios_ena, bios_enb;
    assign bios_ena = 1'b1;
    assign bios_enb = 1'b1;
    bios_mem bios_mem (
      .clk(clk),
      .ena(bios_ena),
      .addra(bios_addra),
      .douta(bios_douta),
      .enb(bios_enb),
      .addrb(bios_addrb),
      .doutb(bios_doutb)
    );

    // Data Memory
    // Synchronous read: read takes one cycle
    // Synchronous write: write takes one cycle
    // Write-byte-enable: select which of the four bytes to write
    wire [13:0] dmem_addr;
    wire [31:0] dmem_din, dmem_dout;
    wire [3:0] dmem_we;
    wire dmem_en;
    assign dmem_en = 1'b1;
    dmem dmem (
      .clk(clk),
      .en(dmem_en),
      .we(dmem_we),
      .addr(dmem_addr),
      .din(dmem_din),
      .dout(dmem_dout)
    );

    // Instruction Memory
    // Synchronous read: read takes one cycle
    // Synchronous write: write takes one cycle
    // Write-byte-enable: select which of the four bytes to write
    wire [31:0] imem_dina, imem_doutb;
    wire [13:0] imem_addra, imem_addrb;
    wire [3:0] imem_wea;
    wire imem_ena;
    assign imem_ena = 1'b1;
    imem imem (
      .clk(clk),
      .ena(imem_ena),
      .wea(imem_wea),
      .addra(imem_addra),
      .dina(imem_dina),
      .addrb(imem_addrb),
      .doutb(imem_doutb)
    );

    // On-chip UART
    //// UART Receiver
    wire [7:0] uart_rx_data_out;
    wire uart_rx_data_out_valid;
    wire uart_rx_data_out_ready;
    //// UART Transmitter
    wire [7:0] uart_tx_data_in;
    wire uart_tx_data_in_valid;
    wire uart_tx_data_in_ready;
    uart #(
        .CLOCK_FREQ(CPU_CLOCK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) on_chip_uart (
        .clk(clk),
        .reset(rst),

        .serial_in(serial_in),
        .data_out(uart_rx_data_out),
        .data_out_valid(uart_rx_data_out_valid),
        .data_out_ready(uart_rx_data_out_ready),

        .serial_out(serial_out),
        .data_in(uart_tx_data_in),
        .data_in_valid(uart_tx_data_in_valid),
        .data_in_ready(uart_tx_data_in_ready)
    );
    wire control_hazards_sum;
    wire control_hazards_reg, control_hazards_reg_ff1, control_hazards_reg_ff2;
    reg control_hazards_sum_ff1;
    wire Hold;
    wire Hold_decode;

    always @(posedge clk) begin
        if(rst) begin
            control_hazards_sum_ff1 <= 1'b0;
        end
        else begin
            control_hazards_sum_ff1 <= control_hazards_sum;
        end
    end

    //iomem
    reg [31:0] iomem [13-1:0];

    reg [31:0] iomem_carrier_nco_1_fcw_reg;
    reg [31:0] iomem_carrier_nco_2_fcw_reg;
    reg [31:0] iomem_carrier_nco_3_fcw_reg;
    reg [31:0] iomem_carrier_nco_4_fcw_reg;
    reg [31:0] iomem_mod_nco_fcw_reg;
    reg [4:0] iomem_mod_nco_shift_term_reg;
    reg [3:0] iomem_note_enable_signal_reg;
    reg iomem_cpu_tx_req_reg;


    wire [13:0] iomem_addr;
    wire [31:0] iomem_din;
    reg  [31:0] iomem_dout_reg;
    wire  [31:0] iomem_dout;
    wire [3:0] iomem_we;
    wire iomem_en;
    wire iomem_rst;
    assign iomem_en = 1'b1;

    //mem[0] UART control
    //mem[1] UART receiver data
    //mem[2] UART transmitter data
    //mem[3] Cycle counter
    //mem[4] Instruction counter
    //mem[5] Reset counters to 0
    assign uart_tx_data_in = iomem[2][7:0];

    always @(posedge clk) begin
        if (iomem_en) begin
            if(iomem_addr == 'd133) begin
                iomem_dout_reg <= {31'b0,tx_ack};
            end 
            else begin
                iomem_dout_reg <= iomem[iomem_addr[3:0]];
            end
        end
    end

    always @(*) begin
        iomem[0] = {30'b0, uart_rx_data_out_valid, uart_tx_data_in_ready};
        iomem[1] = {24'b0, uart_rx_data_out};
    end

    always @(posedge clk) begin
        if (rst) begin
            iomem[2] <= 32'b0;
        end
        else if (iomem_addr == 'd2 && iomem_we[0] == 1'b1  && iomem_en) begin
            iomem[2][7:0] <= iomem_din[7:0];
        end
        else begin
            iomem[2] <= iomem[2];
        end
    end   

    always @(posedge clk) begin
        if(rst || iomem_rst) begin
            iomem[4] <= 32'b0;
        end
        else begin
            iomem[4] <= iomem[4] + 'd1;
        end
    end

    always @(posedge clk) begin
        if(rst || iomem_rst) begin
            iomem[5] <= 32'b0;
        end
        else if(control_hazards_sum || Hold) begin
            iomem[5] <= iomem[5];
        end
        else begin
            iomem[5] <= iomem[5] + 'd1;
        end
    end

    reg [2:0] buttons_ff1;

    always @(posedge clk) begin
        if(rst) begin
            buttons_ff1 <= 3'b0; 
        end
        else begin
            buttons_ff1 <= buttons;
        end
    end

    wire fifo_wr_en;
    wire [2:0] fifo_din;
    wire fifo_full;
    wire fifo_rd_en;
    reg fifo_rd_en_ff1;
    wire [2:0] fifo_dout;
    wire fifo_empty;

    assign fifo_wr_en = ~fifo_full && ((buttons[0] && ~buttons_ff1[0]) || (buttons[1] && ~buttons_ff1[1]) || (buttons[2] && ~buttons_ff1[2]));
    assign fifo_din = buttons;

    fifo #(
        .WIDTH(3)
    ) fifo (
        .clk(clk),
        .rst(rst),
        // Write side
        .wr_en(fifo_wr_en),
        .din(fifo_din),
        .full(fifo_full),
    
        // Read side
        .rd_en(fifo_rd_en),
        .dout(fifo_dout),
        .empty(fifo_empty)
    );

    always @(*) begin
        iomem[8] = {31'd0, fifo_empty};

        iomem[10] = {30'b0, switches};
    end

    always @(posedge clk) begin
        if (rst) begin
            iomem[12] <= 32'b0;
        end
        else if (iomem_addr == 'd12 && iomem_we[0] == 1'b1  && iomem_en) begin
            iomem[12][5:0] <= iomem_din[5:0];
        end
        else begin
            iomem[12] <= iomem[12];
        end
    end   
    assign LEDS = iomem[12][5:0];

    assign iomem_dout = fifo_rd_en_ff1 ? {29'd0, fifo_dout} : iomem_dout_reg;

    reg alarm;

    integer i;
    generate   
        always @(posedge clk) begin
            if(iomem_en) begin
                for(i=0; i<4; i=i+1) begin
                    if (iomem_we[i]) begin
                        case(iomem_addr)
                            'd64: iomem_carrier_nco_1_fcw_reg[i*8 +: 8] <= iomem_din[i*8 +: 8];
                            'd65: iomem_carrier_nco_2_fcw_reg[i*8 +: 8] <= iomem_din[i*8 +: 8];
                            'd66: iomem_carrier_nco_3_fcw_reg[i*8 +: 8] <= iomem_din[i*8 +: 8];
                            'd67: iomem_carrier_nco_4_fcw_reg[i*8 +: 8] <= iomem_din[i*8 +: 8];
                            'd128:iomem_mod_nco_fcw_reg[i*8 +: 8] <= iomem_din[i*8 +: 8];
                            default: alarm <=1'b1;
                        endcase
                    end
                end
            end        
        end 
    endgenerate

    always @(posedge clk) begin
        if (iomem_addr == 'd129 && iomem_we[0] == 1'b1  && iomem_en) begin
            iomem_mod_nco_shift_term_reg <= iomem_din[4:0];
        end
    end   

    always @(posedge clk) begin
        if (iomem_addr == 'd130 && iomem_we[0] == 1'b1  && iomem_en) begin
            iomem_note_enable_signal_reg <= iomem_din[3:0];
        end
    end   

    always @(posedge clk) begin
        if (iomem_addr == 'd132 && iomem_we[0] == 1'b1  && iomem_en) begin
            iomem_cpu_tx_req_reg <= iomem_din[0];
        end
    end   

    assign iomem_carrier_nco_1_fcw = iomem_carrier_nco_1_fcw_reg[23:0];
    assign iomem_carrier_nco_2_fcw = iomem_carrier_nco_2_fcw_reg[23:0];
    assign iomem_carrier_nco_3_fcw = iomem_carrier_nco_3_fcw_reg[23:0];
    assign iomem_carrier_nco_4_fcw = iomem_carrier_nco_4_fcw_reg[23:0];
    assign iomem_mod_nco_fcw = iomem_mod_nco_fcw_reg[23:0];
    assign iomem_mod_nco_shift_term = iomem_mod_nco_shift_term_reg;
    assign iomem_note_enable_signal = iomem_note_enable_signal_reg;
    assign iomem_cpu_tx_req = iomem_cpu_tx_req_reg;



    //CPU pipeline
    reg [31:0] tohost_csr = 0;

    wire [31:0] PC_addr;
    reg [31:0] PC_addr_Fetch;
    wire [31:0] Inst_mem;
    reg [31:0] Inst_Fetch;
    wire PCSel;
    wire [31:0] ALU_out_reg;
    wire [31:0] Inst_addr;
    wire Inst_addr_sel; // 0 for bios , 1 for Imem
    
    assign Inst_addr_sel = Inst_addr[31:28] == 4'b0001 ;
    assign bios_addra = Inst_addr_sel ? 12'b0 : Inst_addr[13:2];
    assign imem_addrb = Inst_addr_sel ? Inst_addr[15:2] : 14'b0;
    assign Inst_mem =  Inst_addr_sel? imem_doutb : bios_douta;

    Instruction_Fetch Instruction_Fetch(
        .clk(clk),
        .rst(rst),
        .Hold(Hold),
        .PCSel(PCSel),
        .ALU(ALU_out_reg),
        .base_addr(RESET_PC),
        .PC_addr(PC_addr),
        .mem_addr(Inst_addr)
    );

    always @(posedge clk) begin
        if(rst) begin
            PC_addr_Fetch <= 0;
        end
        else if(Hold) begin
            PC_addr_Fetch <= PC_addr_Fetch;            
        end
        else begin
            PC_addr_Fetch <= PC_addr;
        end       
    end

    always @(posedge clk) begin
        if(rst) begin
            Inst_Fetch <= 0;
        end
        else if(Hold) begin
            Inst_Fetch <= Inst_Fetch;
        end
        else begin
            Inst_Fetch <= Inst_mem;
        end
    end
    

    wire [31:0] Data_A, Data_B;
    wire [31:0] Data_D; 
    wire [4:0] Addr_D;
    reg [31:0] PC_addr_Decode;
    reg [31:0] Inst_Decode;
    wire [31:0] Inst_Execute;
    wire RegWen_EX;
    // Register file
    // Asynchronous read: read data is available in the same cycle
    // Synchronous write: write takes one cycle

    wire we;
    wire [4:0] ra1, ra2, wa;
    wire [31:0] wd;
    wire [31:0] rd1, rd2;
    reg_file rf (
        .clk(clk),
        .rst(rst),
        .we(we),
        .ra1(ra1), .ra2(ra2), .wa(Addr_D),
        .wd(Data_D),
        .rd1(rd1), .rd2(rd2)
    );

    assign ra1 = Inst_Fetch[19:15];
    assign ra2 = Inst_Fetch[24:20];
    assign we = RegWen_EX;

    assign Data_A = rd1;
    assign Data_B = rd2;

    always @(posedge clk) begin
        if (rst) begin
            PC_addr_Decode <= 0;
        end
        else if(Hold) begin
            PC_addr_Decode <= PC_addr_Decode;  
        end
        else begin
            PC_addr_Decode <= PC_addr_Fetch;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            Inst_Decode <= 0;
        end
        else if(Hold) begin
            Inst_Decode <= Inst_Decode;  
        end
        else begin
            Inst_Decode <= Inst_Fetch;
        end
    end

    wire [3:0] ALUSel;
    wire [2:0] ImmSel;
    wire BrUn, ASel, BSel, RegWen_decode;
    wire [1:0] Data_ASel, Data_BSel;
    wire [1:0] MemRW_decode;
    wire [2:0] LdSel_decode;
    wire [1:0] WBSel_decode;
    wire CSRSel_decode;
    wire [31:0] Imm;
    reg [31:0] Imm_reg;
    reg [31:0] Data_A_mux, Data_B_mux;
    reg [31:0] Data_A_mux_reg, Data_B_mux_reg;

    control_unit_decode control_unit_decode (
        .clk(clk),
        .rst(rst),
        .Inst_Fetch(Inst_Fetch),
        .Inst_Decode(Inst_Decode),
        .Inst_Execute(Inst_Execute),
        .control_hazards_sum(control_hazards_sum),
        .ImmSel(ImmSel),
        .BrUn_reg(BrUn),
        .ASel_reg(ASel),
        .BSel_reg(BSel),
        .Data_ASel(Data_ASel),
        .Data_BSel(Data_BSel),
        .ALUSel_reg(ALUSel),
        .MemRW_reg(MemRW_decode),
        .RegWen_reg(RegWen_decode),
        .LdSel_reg(LdSel_decode),
        .WBSel_reg(WBSel_decode),
        .CSRSel_reg(CSRSel_decode),
        .Hold(Hold),
        .Hold_reg(Hold_decode)
    );

    wire [31:0] ALU_out;
    wire [31:0] Add_out;
    wire [31:0] PC_addr_Execute;
    wire [31:0] Data_W;
    wire BrEq, BrLT;

    Imm_Gen Imm_Gen(
        .Inst(Inst_Fetch),
        .ImmSel(ImmSel),
        .Imm(Imm)
    );

    always @(posedge clk) begin
        if(rst) begin
            Imm_reg <= 0;
        end    
        else begin
            Imm_reg <= Imm;
        end
    end

    localparam REG = 2'b00;
    localparam DATA_D = 2'b10;
    localparam DATA_D_ff1 = 2'b11;

    always @(*) begin
        case(Data_ASel)
            REG:Data_A_mux = Data_A;
            DATA_D:Data_A_mux = ALU_out;
            DATA_D_ff1:Data_A_mux = Data_D;
            default: Data_A_mux = 32'bx;
        endcase
    end

    always @(*) begin
        case(Data_BSel)
            REG:Data_B_mux = Data_B;
            DATA_D:Data_B_mux = ALU_out;
            DATA_D_ff1:Data_B_mux = Data_D;
            default: Data_B_mux = 32'bx;
        endcase
    end

    always @(posedge clk) begin
        if(rst) begin
            Data_A_mux_reg <= 0;
        end    
        else begin
            Data_A_mux_reg <= Data_A_mux;
        end
    end

    always @(posedge clk) begin
        if(rst) begin
            Data_B_mux_reg <= 0;
        end    
        else begin
            Data_B_mux_reg <= Data_B_mux;
        end
    end

    Execute Execute (
        .clk(clk),
        .rst(rst),
        .Data_A_mux(Data_A_mux_reg),
        .Data_B_mux(Data_B_mux_reg),
        .PC_addr_Decode(PC_addr_Decode),
        .Inst_Decode(Inst_Decode),
        .ASel(ASel),
        .BSel(BSel),
        .ALUSel(ALUSel),
        .Imm(Imm_reg),
        .BrUn(BrUn),
        .BrEq(BrEq),
        .BrLT(BrLT),
        .ALU_out(ALU_out),
        .Add_out(Add_out),
        .ALU_out_reg(ALU_out_reg),
        .PC_addr_Execute(PC_addr_Execute),
        .Data_W(Data_W),
        .Inst_Execute(Inst_Execute)
    );



    wire [1:0] MemRW_EX;
    wire [2:0] LdSel_EX;
    wire [1:0] WBSel_EX;
    wire CSRSel_EX;

    control_unit_EX control_unit_EX (
        .clk(clk),
        .rst(rst),
        .BrEq(BrEq),
        .BrLT(BrLT),
        .Inst(Inst_Decode),
        .Hold_decode_reg(Hold_decode),
        .MemRW_decode_reg(MemRW_decode),
        .RegWen_decode_reg(RegWen_decode),
        .LdSel_decode_reg(LdSel_decode),
        .WBSel_decode_reg(WBSel_decode),
        .CSRSel_decode_reg(CSRSel_decode),
        .MemRW_EX(MemRW_EX),
        .RegWen_EX_reg(RegWen_EX),
        .LdSel_EX_reg(LdSel_EX),
        .WBSel_EX_reg(WBSel_EX),
        .CSRSel_EX_reg(CSRSel_EX),
        .PCSel(PCSel),
        .control_hazards_reg(control_hazards_reg),
        .control_hazards_reg_ff1(control_hazards_reg_ff1),
        .control_hazards_reg_ff2(control_hazards_reg_ff2)
    );
    
    assign control_hazards_sum = control_hazards_reg || control_hazards_reg_ff1 || control_hazards_reg_ff2;

    always @(posedge clk) begin
        if(rst) begin
            tohost_csr <= 0;
        end
        else if(CSRSel_EX) begin
            tohost_csr <= ALU_out_reg;
        end
        else begin
            tohost_csr <= tohost_csr;
        end
    end

    wire [31:0] mem_in;

    DMem_pre DMem_pre (
        .ALU_out(Add_out),
        .Data_W(Data_W),
        .MemRW_EX(MemRW_EX),
        .PC_addr_Decode(PC_addr_Decode),
        .Mem_Data_W(mem_in),
        .DMem_Data_addr(dmem_addr),
        .DMem_WE(dmem_we),
        .IMem_Data_addr(imem_addra),
        .IMem_WE(imem_wea),
        .IO_Data_addr(iomem_addr),
        .IO_WE(iomem_we),
        .bios_Data_addr(bios_addrb)
    );

    assign dmem_din = mem_in;
    assign imem_dina = mem_in;
    assign iomem_din = mem_in;
    wire [3:0] addr_space;
    assign addr_space = ALU_out_reg[31:28];
    reg [31:0] mem_out;

    always @(*) begin
        case(addr_space)
            4'b0001, 4'b0011: mem_out = dmem_dout;
            4'b0100: mem_out = bios_doutb;
            4'b1000: mem_out = iomem_dout;
            default: mem_out = 32'bx;
        endcase
    end
    assign iomem_rst = ~control_hazards_sum_ff1 && Inst_Execute[6:2] == 5'b01000 && ALU_out_reg == 32'h80000018;
    
    Memory_Access Memory_Access (
    .PC_addr_Execute(PC_addr_Execute),
    .ALU_out_reg(ALU_out_reg),
    .DMem_out(mem_out),
    .Inst_Execute(Inst_Execute),
    .LdSel(LdSel_EX),
    .WBSel(WBSel_EX),
    .Data_D(Data_D),
    .Addr_D(Addr_D)
    );

    reg uart_rx_data_out_ready_reg;
    always @(posedge clk) begin
        if(rst) begin
            uart_rx_data_out_ready_reg <= 0;
        end
        else begin
            uart_rx_data_out_ready_reg <= (Inst_Decode[6:2] == 5'b00000 && Add_out == 32'h80000004 && ~control_hazards_sum);
        end
    end

    assign uart_rx_data_out_ready = uart_rx_data_out_ready_reg;

    reg has_byte;
    always @(posedge clk) begin
        if(rst) begin
            has_byte <= 1'b0;
        end
        else if(Inst_Execute[6:2] == 5'b01000 && ALU_out_reg == 32'h80000008 && ~control_hazards_sum) begin
            has_byte <= 1'b1;
        end
        else if(has_byte && uart_tx_data_in_ready) begin
            has_byte <= 1'b0;
        end
    end

    assign uart_tx_data_in_valid = has_byte;

    assign fifo_rd_en = Inst_Decode[6:2] == 5'b00000 && Add_out == 32'h80000024 && ~control_hazards_sum;

    always @(posedge clk) begin
        if(rst) begin
            fifo_rd_en_ff1 <= 1'b0;
        end
        else begin
            fifo_rd_en_ff1 <= fifo_rd_en;
        end
    end
endmodule
