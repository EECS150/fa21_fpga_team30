module Load_Extension(
	input [1:0] DMem_Sel,
	input [31:0] DMem_out,
	input [2:0] LdSel,
	output reg [31:0] Ld_out
);
	localparam LW = 3'b010;
	localparam LH = 3'b001;
	localparam LB = 3'b000;
	localparam LHU = 3'b101;
	localparam LBU = 3'b100;
	wire [31:0] test;
	

	always @(*) begin
		case(LdSel)
			LW: Ld_out = DMem_out;
			LH: begin
				if(DMem_Sel[1] == 1'b0) begin
					Ld_out = {{16{DMem_out[15]}}, DMem_out[15:0]};
				end
				else begin
					Ld_out = {{16{DMem_out[31]}}, DMem_out[31:16]};					
				end
			end 
			LB: begin
				case(DMem_Sel)
					2'b00: Ld_out = {{24{DMem_out[7]}}, DMem_out[7:0]};
					2'b01: Ld_out = {{24{DMem_out[15]}}, DMem_out[15:8]};
					2'b10: Ld_out = {{24{DMem_out[23]}}, DMem_out[23:16]};
					2'b11: Ld_out = {{24{DMem_out[31]}}, DMem_out[31:24]};
				endcase
			end 
			LHU: begin
				if(DMem_Sel[1] == 1'b0) begin
					Ld_out = {{16{1'b0}}, DMem_out[15:0]};
				end
				else begin
					Ld_out = {{16{1'b0}}, DMem_out[31:16]};					
				end
			end
			LBU: begin
				case(DMem_Sel)
					2'b00: Ld_out = {{24{1'b0}}, DMem_out[7:0]};
					2'b01: Ld_out = {{24{1'b0}}, DMem_out[15:8]};
					2'b10: Ld_out = {{24{1'b0}}, DMem_out[23:16]};
					2'b11: Ld_out = {{24{1'b0}}, DMem_out[31:24]};
				endcase
			end 
			default: Ld_out = 32'bx;
		endcase
	end
endmodule
