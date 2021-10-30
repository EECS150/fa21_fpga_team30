module Branch_Comp(
    input [31:0] Data_A,
    input [31:0] Data_B,
    input BrUn,
    output BrEq,
    output BrLT
);

	assign BrEq = Data_A == Data_B;
	assign BrLT = BrUn ? $unsigned(Data_A) < $unsigned(Data_B) : $signed(Data_A) < $signed(Data_B);
endmodule