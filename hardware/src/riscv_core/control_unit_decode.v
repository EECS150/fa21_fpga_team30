module control_unit_decode (
	input clk,
	input rst,
    input [31:0] Inst_Fetch,
    input [31:0] Inst_Decode,
    input [31:0] Inst_Execute,
    input control_hazards_sum,
	output reg [2:0] ImmSel_reg,
	output reg BrUn_reg,
	output reg ASel_reg,
	output reg BSel_reg,
    output reg [1:0] Data_ASel_reg,
    output reg [1:0] Data_BSel_reg,
	output reg [3:0] ALUSel_reg,
	output reg [1:0] MemRW_reg,
	output reg RegWen_reg,
	output reg [2:0] LdSel_reg,
	output reg [1:0] WBSel_reg,
    output reg CSRSel_reg,
    output reg Hold,
    output reg Hold_reg
	);

//    reg Data_ASel_reg = 2'b0;
//    reg Data_BSel_reg = 2'b0;

	localparam opcode_R     = 5'b01100;
    localparam opcode_I     = 5'b00100;
    localparam opcode_L     = 5'b00000;
    localparam opcode_S     = 5'b01000;
    localparam opcode_B     = 5'b11000;
    localparam opcode_JALR  = 5'b11001;
    localparam opcode_JAL   = 5'b11011;
    localparam opcode_AUIPC = 5'b00101;
    localparam opcode_LUI   = 5'b01101;
    localparam opcode_CSR   = 5'b11100;

    wire [4:0] opcode;
    wire [2:0] funct3;
    wire [4:0] ra1, ra2, rd_Decode, rd_Execute;

    assign opcode = Inst_Fetch[6:2];
    assign funct3 = Inst_Fetch[14:12];
    assign ra1 = Inst_Fetch[19:15];
    assign ra2 = Inst_Fetch[24:20];

    wire [4:0] opcode_Decode;
    wire [4:0] opcode_Execute;   

    assign opcode_Decode = Inst_Decode[6:2];
    assign rd_Decode = Inst_Decode[11:7];
    assign opcode_Execute = Inst_Execute[6:2];
    assign rd_Execute = Inst_Execute[11:7];

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

    localparam I = 3'b000;
    localparam S = 3'b001;
    localparam B = 3'b010;
    localparam J = 3'b011;
    localparam U = 3'b100;
    localparam C = 3'b101;

    localparam ALU    = 2'b00;
    localparam DMEM     = 2'b01;
    localparam PC_ADD4 = 2'b10;

    localparam MEMRW_0 = 2'b00;
    localparam SW = 2'b01;
    localparam SH = 2'b10;
    localparam SB = 2'b11;

    localparam REG = 2'b00;
    localparam DATA_D = 2'b10;
    localparam DATA_D_ff1 = 2'b11;

    reg [3:0] ALUSel;
    reg [2:0] ImmSel;
    wire BrUn, ASel, BSel, RegWen;
    reg [1:0] MemRW;
    wire [2:0] LdSel;
    reg [1:0] WBSel;
    wire CSRSel;
    reg [1:0] Data_ASel, Data_BSel;
    
    reg control_hazards_sum_ff1;

    always @(posedge clk) begin
        if(rst) begin
            control_hazards_sum_ff1 <= 1'b0;
        end
        else begin
            control_hazards_sum_ff1 <= control_hazards_sum;
        end
    end

    wire control_hazards_sum_negedge;


    assign control_hazards_sum_negedge = ~control_hazards_sum && control_hazards_sum_ff1;
    


    always @(*) begin
        if(Hold_reg) begin
            Hold = 1'b0;
        end
        else if(rd_Decode == ra1 && opcode_Decode == opcode_L && Inst_Fetch[1:0] == 2'b11 && (opcode == opcode_R || opcode == opcode_I ||opcode == opcode_S || opcode == opcode_L || opcode == opcode_B ||opcode == opcode_JALR)) begin
            Hold = 1'b1;
        end
        else if(rd_Decode == ra2 && opcode_Decode == opcode_L && Inst_Fetch[1:0] == 2'b11 && (opcode == opcode_R ||opcode == opcode_S || opcode == opcode_B))begin
            Hold = 1'b1;            
        end
        else begin
            Hold = 1'b0;
        end
    end

//(opcode == opcode_R || opcode == opcode_I ||opcode == opcode_S || opcode == opcode_L || opcode == opcode_B ||opcode == opcode_JAL ||opcode == opcode_JALR ||opcode == opcode_LUI ||opcode == opcode_AUIPC)
    always @(*) begin
        case(opcode)
            opcode_R, opcode_I, opcode_L, opcode_S, opcode_CSR, opcode_B, opcode_JALR: begin
                if(control_hazards_sum && control_hazards_sum_ff1) begin
                    Data_ASel = REG;
                end
                else if((rd_Decode == ra1) && (ra1 != 5'b0) && (opcode_Decode == opcode_R || opcode_Decode == opcode_I || opcode_Decode == opcode_AUIPC || opcode_Decode == opcode_LUI )) begin
                    Data_ASel = DATA_D;
                end
                else if((rd_Execute == ra1) && (ra1 != 5'b0) && ~control_hazards_sum_negedge && (opcode_Execute == opcode_R || opcode_Execute == opcode_I || opcode_Execute == opcode_L || opcode_Execute == opcode_AUIPC || opcode_Execute == opcode_LUI )) begin
                    Data_ASel = DATA_D_ff1;                 
                end
                else begin
                    Data_ASel = REG;
                end
            end
            default: Data_ASel = REG;
        endcase
    end 
//    wire a, b, c, d;
//    assign a = opcode_Decode == opcode_R;
//    assign b = rd_Decode == ra2;
//    assign c = opcode_Decode == opcode_R || opcode_Decode == opcode_I || opcode_Decode == opcode_AUIPC || opcode_Decode == opcode_LUI;
//    assign d = opcode == opcode_R;
    always @(*) begin
        case(opcode)
            opcode_R, opcode_S, opcode_B: begin
                if(control_hazards_sum && control_hazards_sum_ff1) begin
                    Data_BSel = REG;
                end
                else if((rd_Decode == ra2) && (ra2 != 5'b0) && (opcode_Decode == opcode_R || opcode_Decode == opcode_I || opcode_Decode == opcode_AUIPC || opcode_Decode == opcode_LUI)) begin
                    Data_BSel = DATA_D;
                end
                else if((rd_Execute == ra2) && (ra2 != 5'b0) && ~control_hazards_sum_negedge && (opcode_Execute == opcode_R || opcode_Execute == opcode_I || opcode_Execute == opcode_L || opcode_Execute == opcode_AUIPC || opcode_Execute == opcode_LUI)) begin
                    Data_BSel = DATA_D_ff1;                 
                end
                else begin
                    Data_BSel = REG;
                end
            end
            default: Data_BSel = REG;
        endcase
    end 

    always @(*) begin
    	if (opcode == opcode_L || opcode == opcode_S || opcode == opcode_B || opcode == opcode_JALR || opcode == opcode_JAL || opcode == opcode_AUIPC) begin
    		ALUSel <= ADD;
    	end
    	else if(opcode == opcode_R) begin
    		ALUSel <= {Inst_Fetch[30],funct3};
    	end
        else if(opcode == opcode_I) begin
            if(Inst_Fetch[14:12] == 3'b101) begin
                ALUSel <= {Inst_Fetch[30],funct3};
            end
            else begin
                ALUSel <= {1'b0,funct3};
            end
        end
        else if(opcode == opcode_LUI) begin
            ALUSel <= SEL_B;
        end
        else if(opcode == opcode_CSR) begin
            if(Inst_Fetch[14:12] == 3'b001) begin
                ALUSel <= SEL_A;
            end
            else begin
                ALUSel <= SEL_B;                
            end
        end
        else begin
            ALUSel <= 4'b0;            
        end
    end

    always @(*) begin
        case(opcode)
            opcode_R    : ImmSel <= I; // no_need
            opcode_I    : ImmSel <= I;
            opcode_L    : ImmSel <= I;
            opcode_S    : ImmSel <= S;
            opcode_B    : ImmSel <= B;
            opcode_JALR : ImmSel <= I;
            opcode_JAL  : ImmSel <= J;
            opcode_AUIPC: ImmSel <= U;
            opcode_LUI  : ImmSel <= U; 
            opcode_CSR  : ImmSel <= C;
            default     : ImmSel <= I;           
        endcase
    end

    assign BrUn = (opcode == opcode_B && (funct3 == 3'b110 || funct3 == 3'b111));
    assign ASel = (opcode == opcode_B || opcode == opcode_JAL || opcode == opcode_AUIPC);
    assign BSel = opcode != opcode_R;
    assign RegWen = (opcode == opcode_R || opcode == opcode_I || opcode == opcode_L ||opcode == opcode_JALR ||opcode == opcode_JAL ||opcode == opcode_AUIPC ||opcode == opcode_LUI);
    assign LdSel = opcode == opcode_L ? funct3 : 3'b0;
    assign CSRSel = opcode == opcode_CSR; 
/*
    localparam DMEM    = 2'b00;
    localparam ALU     = 2'b01;
    localparam PC_ADD4 = 2'b10;
*/
    always @(*) begin
        case(opcode)
            opcode_R    : WBSel <= ALU; // no_need
            opcode_I    : WBSel <= ALU;
            opcode_L    : WBSel <= DMEM;
            opcode_S    : WBSel <= ALU;// no_need
            opcode_B    : WBSel <= ALU;// no_need
            opcode_JALR : WBSel <= PC_ADD4;
            opcode_JAL  : WBSel <= PC_ADD4;
            opcode_AUIPC: WBSel <= ALU;
            opcode_LUI  : WBSel <= ALU; 
            default     : WBSel <= ALU; // no_need
        endcase
    end

    always @(*) begin
        if(opcode != opcode_S) begin
            MemRW <= MEMRW_0;            
        end
        else begin
            case(funct3)
                3'b000: MemRW <= SB;
                3'b001: MemRW <= SH;
                3'b010: MemRW <= SW;
                default:MemRW <= SW;
            endcase
        end
    end


    always @(posedge clk) begin
        if(rst) begin
            ImmSel_reg <= 0;  
            BrUn_reg   <= 0;
            ASel_reg   <= 0;
            BSel_reg   <= 0;
            Data_ASel_reg <= 0;
            Data_BSel_reg <= 0;
            ALUSel_reg <= 0;
            MemRW_reg  <= 0;
            RegWen_reg <= 0;
            LdSel_reg  <= 0;
            WBSel_reg  <= 0;
            CSRSel_reg <= 0;
            Hold_reg   <= 0;
        end
        else begin
            ImmSel_reg <= ImmSel;  
            BrUn_reg   <= BrUn;
            ASel_reg   <= ASel;
            BSel_reg   <= BSel;
            Data_ASel_reg <= Data_ASel;
            Data_BSel_reg <= Data_BSel;
            ALUSel_reg <= ALUSel;
            MemRW_reg  <= MemRW;
            RegWen_reg <= RegWen;
            LdSel_reg  <= LdSel;
            WBSel_reg  <= WBSel;
            CSRSel_reg <= CSRSel;
            Hold_reg   <= Hold;
        end
    end
endmodule