//`timescale 1ns/1ps
`define NUM_TEST 30
`define TESTID_SIZE 5

module TB ();

	parameter CLK_PER = 4;
	parameter NUM_CLK = 10000;


	// --------------------------------------------
	// Wires and Regs
	// --------------------------------------------
	reg				CLK;
	reg				RESET_N;

	wire			IREQ;
	wire	[31:0]	IADDR;
	wire	[31:0]	INSTR;

	wire			DREQ;
	wire	[31:0]	DADDR;
	wire			DWE;
	wire	[1:0]	DSIZE;
	wire	[31:0]	DIN;
	wire	[31:0]	DOUT;

	wire	[31:0]	OUTPUT_PORT;
	wire	[31:0]	NUM_INST;

	reg		[3:0]	DBE;


	always #(CLK_PER/2) CLK = ~CLK;

	initial begin
		CLK = 1'b0;
		RESET_N = 1'b0;

		#(CLK_PER/4);
		
		#(CLK_PER*4);
			RESET_N = 1;
	end

	
	CortexM0 CortexM0 (
		.CLK(CLK),
		.RESET_N(RESET_N),
		
		// For instruction memory
		.IREQ(IREQ),
		.IADDR(IADDR),
		.INSTR(INSTR),

		// For data memory
		.DREQ(DREQ),
		.DADDR(DADDR),
		.DRW(DWE),		// read/write
		.DSIZE(DSIZE),	// Data memory access size 
		.DIN(DIN),
		.DOUT(DOUT),

		.OUTPUT_PORT(OUTPUT_PORT),
		.NUM_INST(NUM_INST)
	);

	SRAM MEM (
		.CLK (CLK),
		.CSN1 (1'b0),		// always chip select
		.ADDR1 (IADDR[13:2]),
		.WE1 (1'b0),			// only read operation
		.BE1 (4'b1111),		// word access
		.DI1 (),				// not used
		.DO1 (INSTR),			// read data

		.CSN2 (~DREQ),
		.ADDR2 (DADDR[13:2]),
		.WE2 (DWE),
		.BE2 (DBE),
		.DI2 (DOUT),
		.DO2 (DIN)
	);

	always @* begin
		casex( {DSIZE, DADDR[1:0]} )
			{2'b00, 2'b00}	:	DBE = 4'b0001;
			{2'b00, 2'b01}	:	DBE = 4'b0010;
			{2'b00, 2'b10}	:	DBE = 4'b0100;
			{2'b00, 2'b11}	:	DBE = 4'b1000;
			{2'b01, 2'b00}	:	DBE = 4'b0011;
			{2'b01, 2'b10}	:	DBE = 4'b1100;
			{2'b10, 2'b00}	:	DBE = 4'b1111;
		endcase
	end

	reg [31:0] TestNumInst [`NUM_TEST-1:0];
	reg [31:0] TestAns[`NUM_TEST-1:0];
	reg TestPassed[`NUM_TEST-1:0];
	reg [31:0] i;
	reg [31:0] cycle;

	initial begin
		TestNumInst[0] <= 4;		TestAns[0] <= 0;		TestPassed[0] <= 1'b0; //ADDI
		TestNumInst[1] <= 14;		TestAns[1] <= 0;		TestPassed[1] <= 1'b0; //JAL
		TestNumInst[2] <= 17;		TestAns[2] <= 1;		TestPassed[2] <= 1'b0; //ADDI
		TestNumInst[3] <= 20;		TestAns[3] <= 32'h0000ffd0;		TestPassed[3] <= 1'b0; //SUB_SPimm
		TestNumInst[4] <= 24;		TestAns[4] <= 32'h00000220;		TestPassed[4] <= 1'b0; //LDR_PC
		TestNumInst[5] <= 28;		TestAns[5] <= 0;		TestPassed[5] <= 1'b0; //STRB_imm
		TestNumInst[6] <= 32;		TestAns[6] <= 32'h0000ffe0;		TestPassed[6] <= 1'b0; //LSL_imm
		TestNumInst[7] <= 36;		TestAns[7] <= 32'h0000ffa8;		TestPassed[7] <= 1'b0; //ADD_SPimm1
		TestNumInst[8] <= 40;		TestAns[8] <= 0;		TestPassed[8] <= 1'b0; //MOV_imm
		TestNumInst[9] <= 44;		TestAns[9] <= 32'h0000000c;		TestPassed[9] <= 1'b0; //LDR_5bimm

		TestNumInst[10] <= 50;	TestAns[10] <= 32'h00000000;	TestPassed[10] <= 1'b0; //
		TestNumInst[11] <= 64;	TestAns[11] <= 32'h0000ffe1;	TestPassed[11] <= 1'b0; //
		TestNumInst[12] <= 74;	TestAns[12] <= 32'h0000000c;	TestPassed[12] <= 1'b0; //
		TestNumInst[13] <= 84;	TestAns[13] <= 32'h00000045;	TestPassed[13] <= 1'b0; //
		TestNumInst[14] <= 94;	TestAns[14] <= 32'h0000ffe3;	TestPassed[14] <= 1'b0; //
		TestNumInst[15] <= 99;	TestAns[15] <= 32'h00000052;	TestPassed[15] <= 1'b0; //STRB_imm
		TestNumInst[16] <= 120;	TestAns[16] <= 32'hffff0000;	TestPassed[16] <= 1'b0; //CMP(reg1)
		TestNumInst[17] <= 155;	TestAns[17] <= 32'h00000007;	TestPassed[17] <= 1'b0; //LDR(5bimm)
		TestNumInst[18] <= 177;	TestAns[18] <= 32'h00000009;	TestPassed[18] <= 1'b0; //STR(5bimm)
		TestNumInst[19] <= 199;	TestAns[19] <= 32'h0000ffea;	TestPassed[19] <= 1'b0; //ADD(reg1)

		TestNumInst[20] <= 255;	TestAns[20] <= 32'h00000057;	TestPassed[20] <= 1'b0; //ADD(reg1)
		TestNumInst[21] <= 275;	TestAns[21] <= 32'h00000000;	TestPassed[21] <= 1'b0; //LW
		TestNumInst[22] <= 375;	TestAns[22] <= 32'h0000ffd0;	TestPassed[22] <= 1'b0; //LW
		TestNumInst[23] <= 475;	TestAns[23] <= 32'hffff0000;	TestPassed[23] <= 1'b0; //SLLI
		TestNumInst[24] <= 575;	TestAns[24] <= 32'h00000005;	TestPassed[24] <= 1'b0; //LW
		TestNumInst[25] <= 675;	TestAns[25] <= 32'h00000001;	TestPassed[25] <= 1'b0; //ADD
		TestNumInst[26] <= 775;	TestAns[26] <= 32'h00000004;	TestPassed[26] <= 1'b0; //LW
		TestNumInst[27] <= 875;	TestAns[27] <= 32'h0000ffd8;	TestPassed[27] <= 1'b0; //SLLI
		TestNumInst[28] <= 975;	TestAns[28] <= 32'h0000ffd0;	TestPassed[28] <= 1'b0; //LW
		TestNumInst[29] <= 1000;	TestAns[29] <= 32'h0000ffd0;	TestPassed[29] <= 1'b0; //SW
	end

	// --------------------------------------------
	// Load test vector to inst and data memory
	// --------------------------------------------
	// Caution : Assumption : input file has hex data like below. 
	//			 input file : M[0x03]M[0x02]M[0x01]M[0x00]
	//                        M[0x07]M[0x06]M[0x05]M[0x04]
	//									... 
	//           If the first 4 bytes in input file is 1234_5678
	//           then, the loaded value is mem[0x0000] = 0x1234_5678 (LSB)

	// defparam TB.MEM.ROMDATA = "test.hex";

	// --------------------------------------------
	// For Dump variables
	// --------------------------------------------

	always @ (negedge CLK) begin
		if (RESET_N) begin
			cycle <= cycle + 1;
			// $display("%d, %d", OUTPUT_PORT, NUM_INST);

			for(i=0; i<`NUM_TEST; i=i+1) begin
				if ((NUM_INST==TestNumInst[i]) && (TestPassed[i]==0)) begin
					if (OUTPUT_PORT == TestAns[i]) begin
						TestPassed[i] <= 1'b1;
						$display("Test #%0d has been passed", i+1);
						if(i == `NUM_TEST-1) begin
							$display("All tests completed. Success");
							$finish();
						end
					end
					else begin
						TestPassed[i] <= 1'b0;
						$display("Test #%0d has been failed! NUM_INST : %0d", i+1, NUM_INST);
						$display("output_port = 0x%0x (Ans : 0x%0x)", OUTPUT_PORT, TestAns[i]);
						$finish();
					end
				end
			end

			// if (HALT == 1) begin
			// 	$display("Finish: %d cycle", cycle);
			// 	$display("Success.");
			// 	$finish();
			// end
		end
	end

	initial begin
		$dumpfile("myfile.dmp");
		$dumpvars;
	end

	initial begin
		i <= 0;
		cycle <= 0;
		#(CLK_PER * NUM_CLK); $finish;
	end


endmodule

