module ALU (
    input [31:0] Data_A,
    input [31:0] Data_B,
    input [3:0] ALUSel,
    output reg [31:0] ALU_out,
    output [31:0] Add_out
);
    localparam ADD  = 4'b0000;
    localparam SUB  = 4'b1000;
    localparam SLL  = 4'b0001;
    localparam SLT  = 4'b0010;
    localparam SLTU = 4'b0011;
    localparam XOR  = 4'b0100;
    localparam SRL  = 4'b0101;
    localparam SRA  = 4'b1101;
    localparam OR   = 4'b0110;
    localparam AND  = 4'b0111;
    localparam SEL_A= 4'b1110;
    localparam SEL_B= 4'b1111;  
    wire [4:0] shift_bit;
    assign shift_bit = Data_B[4:0];

    always @(*) begin
        case(ALUSel) 
            ADD:   ALU_out = Data_A + Data_B;
            SUB:   ALU_out = Data_A - Data_B;
            SLL:   ALU_out = Data_A << Data_B[4:0];
            SLT:   ALU_out = $signed(Data_A) < $signed(Data_B) ? 'b1: 'b0;
            SLTU:  ALU_out = $unsigned(Data_A) < $unsigned(Data_B) ? 'b1: 'b0;
            XOR:   ALU_out = Data_A ^ Data_B;
            SRL:   ALU_out = Data_A >> Data_B[4:0];
            SRA:   ALU_out = $signed(Data_A) >>> Data_B[4:0];
            OR:    ALU_out = Data_A | Data_B;
            AND:   ALU_out = Data_A & Data_B;
            SEL_A: ALU_out = Data_A;
            SEL_B: ALU_out = Data_B;
            default: ALU_out = 32'bx;
        endcase
    end

    assign Add_out = Data_A + Data_B;
endmodule