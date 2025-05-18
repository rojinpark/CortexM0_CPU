
module CortexM0 (
	input	wire		      CLK,
	input	wire	        RESET_N, // reset when negative
	
	// For instruction memory
	output wire	    		IREQ, // Useless
	output wire [31:0]	IADDR, // 13:2 is used for PC
	input	wire  [31:0]	INSTR, // Instruction from I$ 

	// For data memory
	output wire	    		DREQ,   // CSN2 = ~DREQ = ~RESET_N
	output wire	[31:0]	DADDR,  // DADDR[13:2] is for addr of D$
	output wire	  			DRW,    // WE2. Write at 1, read at 0
	output wire	[ 1:0]	DSIZE,  // D$ access size : 00 for byte, 01 for halfword, 10 for word
	input	wire	[31:0]	DIN,    // read value from D$
	output wire	[31:0]	DOUT,    // write value into D$

  output wire [31:0] OUTPUT_PORT, // For tests
  output reg [31:0] NUM_INST // For tests
);

  /* ==================== Registers ==================== */
  /* Program Counter */
  reg [31:0] PC;
  reg INST_SELECTOR;
  /* APSR */
  reg [3:0] APSR; // N, Z, C, V

  /* Pipeline CTRL registers */
  reg [4:0] ALUSel1;
  reg APSR_Update1;
  reg RmSel1;

  reg [2:0] LDALUSel1, LDALUSel2;
  reg DRW1, DRW2, DRW3; // WE2. D$ Write at 1, read at 0. DRW3 for OUTPUT_PORT
  reg [1:0] DSIZE1, DSIZE2;

  reg WBSel1, WBSel2, WBSel3;
  reg RegWE1, RegWE2, RegWE3;

  /* IF/ID pipeline registers */
  reg [31:0] PC0;
  reg [31:0] IR32;
  reg [15:0] IR16;

  /* ID/EX pipeline registers */
  reg [31:0] PC1; 
  reg [31:0] Rt, Rn, Rm; // Contains register data
  reg [3:0] Rt1, Rn1, Rm1, Rd1; // Contains regsiter addr
  reg INST_COUNT1;

  /* EX/MEM pipeline registers */
  reg [31:0] ALUOut1;
  reg [31:0] DOUTReg;
  reg [3:0] Rt2, Rn2, Rm2, Rd2;
  reg INST_COUNT2;

  /* WB/MEM pipeline registers */
  reg [31:0] MDR;
  reg [31:0] ALUOut2;
  reg [31:0] DOUTReg3;
  reg [3:0] Rt3, Rn3, Rm3, Rd3;
  reg INST_COUNT3;

  /* ==================== For the register file ==================== */

  wire WE1;
  wire [3:0] WA1;
  wire [31:0] DI1;
  wire [3:0] RA0, RA1, RA2;
  wire [31:0] RD0, RD1, RD2;

  REGFILE REGFILE (
    .CLK(CLK),
    .nRST(RESET_N),
    .WEN1(WE1),
    .WA1(WA1), 
    .DI1(DI1), 
    .WEN2(1'b0),
    .WA2(4'b0),
    .DI2(32'b0),
    .RA0(RA0),
    .RA1(RA1),
    .RA2(RA2),
    .DOUT0(RD0),
    .DOUT1(RD1),
    .DOUT2(RD2)
  );

  assign WE1 = (REGMODE_CTRL) ? WEN1_MCTRL : RegWE3;
  assign DI1 = (REGMODE_CTRL) ? DI1_MCTRL : 
                (WBSel3) ? ALUOut2 : MDR;
  assign WA1 = (REGMODE_CTRL) ? WA1_MCTRL : Rd3;
  assign RA0 = RA0_CTRL;
  assign RA1 = RA1_CTRL;
  assign RA2 = RA2_CTRL;

  // your code here

  /* ==================== For ALU ==================== */
  wire [31:0] A, B, C;
  wire [4:0] ALUSel;
  wire APSR_Update;
  wire [3:0] AI, AO;

  ALU ALU (
    .A(A), .B(B), .C(C),
    .ALUSel(ALUSel),
    .APSR_Update(APSR_Update),
    .AI(AI), .AO(AO) 
  );

  assign A = (ASel == 3'b000) ? Rn : 
            (ASel == 3'b001) ? PC1 :
            (ASel == 3'b010) ? MODIFIED_DATA : 
            (ASel == 3'b011) ? ALUOut1 : DI1;
  assign B = (BSel == 2'b00) ? Rm : 
            (BSel == 2'b01) ? MODIFIED_DATA : 
            (BSel == 2'b10) ? ALUOut1 : DI1;
  assign ALUSel = ALUSel1;
  assign APSR_Update = APSR_Update1;
  assign AI = APSR;

  /* ==================== For LDALU ==================== */
  wire [31:0] RAW_DATA;
  wire [1:0] OFFSET;
  wire [2:0] LDALUSel;
  wire [31:0] MODIFIED_DATA;

  LDALU LDALU (
    .RAW_DATA(RAW_DATA), .LDALUSel(LDALUSel), .OFFSET(OFFSET),
    .MODIFIED_DATA(MODIFIED_DATA)
  );

  assign RAW_DATA = DIN;
  assign LDALUSel = LDALUSel2;
  assign OFFSET = DADDR[1:0];

  /* ==================== For STALU ==================== */
  wire [31:0] ORIGINAL;
  wire [1:0] OFFSET2;
  wire [1:0] DSIZE_ST;
  wire [31:0] MODIFIED;

  STALU STALU (
    .ORIGINAL(ORIGINAL), .OFFSET2(OFFSET2), .DSIZE_ST(DSIZE_ST),
    .MODIFIED(MODIFIED)
  );

  assign ORIGINAL = (DMODE_CTRL) ? DOUT_MCTRL : DOUTReg;
  assign DSIZE_ST = DSIZE;
  assign OFFSET2 = DADDR[1:0];

  /* ==================== For ImmGen ==================== */
  wire [15:0] INST;
  wire [2:0] immsel;
  wire [31:0] imm;

  ImmGen ImmGen (
    .INST(INST), .immsel(immsel), .imm(imm)
  );

  assign INST = IR16;
  assign immsel = ImmSel_CTRL;

  /* ==================== For Forwarding ==================== */
  wire [15:0] F_ID_INST16;
  wire [3:0] ID_rm;

  wire [3:0] EX_rt;
  wire [3:0] EX_rn;
  wire [3:0] EX_rm;

  wire [3:0] EX_rd;
  wire [3:0] MEM_rd;
  wire [3:0] WB_rd;

  wire RegWE_EX;
  wire RegWE_MEM;
  wire RegWE_WB;

  wire RmSel_EX;
  wire [4:0] ALUSel_EX;
  wire WBSel_MEM;
  wire WBSel_EX;

  wire [2:0] ASel;
  wire [1:0] BSel;
  wire [1:0] DOUTSel;
  wire [2:0] AGSel;
  wire DataHazard_F;

  Forwarding Forwarding (
    .F_ID_INST16(F_ID_INST16), .ID_rm(ID_rm), .EX_rt(EX_rt), .EX_rn(EX_rn), 
    .EX_rm(EX_rm), .EX_rd(EX_rd), .MEM_rd(MEM_rd), .WB_rd(WB_rd), 
    .RegWE_MEM(RegWE_MEM), .RegWE_WB(RegWE_WB), .RegWE_EX(RegWE_EX), 
    .RmSel_EX(RmSel_EX), .ALUSel_EX(ALUSel_EX), .WBSel_EX(WBSel_EX),
    .WBSel_MEM(WBSel_MEM), .ASel_F(ASel), .BSel_F(BSel), 
    .DOUTSel_F(DOUTSel), .AGSel_F(AGSel), .DataHazard_F(DataHazard_F)
  );

  assign F_ID_INST16 = IR16;
  assign ID_rm = RA2_CTRL;
  assign EX_rt = Rt1;
  assign EX_rn = Rn1;
  assign EX_rm = Rm1;
  assign EX_rd = Rd1;
  assign MEM_rd = Rd2;
  assign WB_rd = Rd3;
  assign RegWE_EX = RegWE1;
  assign RegWE_MEM = RegWE2;
  assign RegWE_WB = RegWE3;
  assign RmSel_EX = RmSel1;
  assign ALUSel_EX = ALUSel1;
  assign WBSel_EX = WBSel1;
  assign WBSel_MEM = WBSel2;

  /* ==================== For Branch_Predictor ==================== */
  wire [15:0] BP_IF_INST16;
  wire [15:0] BP_ID_INST16;
  wire [31:0] BP_IADDR;
  wire [31:0] Prediction;

  Branch_Predictor Branch_Predictor (
    .BP_IF_INST16(BP_IF_INST16), .BP_ID_INST16(BP_ID_INST16), 
    .BP_IADDR(BP_IADDR), .Prediction(Prediction)
  );

  assign BP_IF_INST16 = (INST_SELECTOR) ? INSTR[31:16] : INSTR[15:0];
  assign BP_ID_INST16 = IR16;
  assign BP_IADDR = IADDR;

  /* ==================== For AddrGen ==================== */
  wire [31:0] AG_DOUT2;
  wire [3:0] AG_APSR;
  wire [15:0] AG_INST;
  wire [31:0] AG_PC;
  wire [31:0] AG_ADDR;
  wire CondBtaken;

  AddrGen AddrGen (
    .AG_DOUT2(AG_DOUT2), .AG_APSR(AG_APSR), .AG_INST(AG_INST),
    .AG_PC(AG_PC), .AG_ADDR(AG_ADDR), .CondBtaken(CondBtaken)
  );

  assign AG_DOUT2 = RD2;
  assign AG_APSR = AO;
  assign AG_INST = IR16;
  assign AG_PC = PC0;

  /* ==================== For CTRL ==================== */
  wire RESET_N_CTRL;
  wire CLK_CTRL;

  wire [15:0] IF_INST16;
  wire [31:0] IF_INST32;
  wire [15:0] ID_INST16;
  wire [31:0] ID_INST32;

  wire CondBtaken_CTRL;
  wire DataHazard;

  wire [31:0] DO0_CTRL;
  wire [31:0] DO1_CTRL;
  wire [31:0] DO2_CTRL;
  wire [31:0] DIN_CTRL;

  wire [1:0] PCSel;     
  wire PC_LOAD_CTRL;      
  wire IF_ID_FLUSH_CTRL;  
  wire IF_ID_LOAD_CTRL;   
  wire ID_EX_FLUSH_CTRL;  
  wire [3:0] RA0_CTRL;  
  wire [3:0] RA1_CTRL;   
  wire [3:0] RA2_CTRL;   
  wire [3:0] WA1_CTRL;    
  wire [2:0] ImmSel_CTRL;
  wire RmSel_CTRL;             
  wire [4:0] ALUSel_CTRL; 
  wire APSR_Update_CTRL; 
  wire DRW_CTRL;          
  wire [1:0] DSIZE_CTRL;  
  wire [2:0] LDALUSel_CTRL; 
  wire WBSel_CTRL;        
  wire RegWE_CTRL;       

  wire DMODE_CTRL;       
  wire REGMODE_CTRL;       

  wire [31:0] DOUT_MCTRL;
  wire [31:0] DADDR_MCTRL;
  wire DRW_MCTRL;
  wire [1:0] DSIZE_MCTRL;

  wire [31:0] DI1_MCTRL;
  wire [3:0] WA1_MCTRL;
  wire WEN1_MCTRL;

  wire [31:0] POPPED_PC_MCTRL;

  wire NUM_INST_MCTRL;

  CTRL CTRL (
    .IF_INST16(IF_INST16), .ID_INST16(ID_INST16), .RESET_N_CTRL(RESET_N_CTRL),
    .CLK_CTRL(CLK_CTRL), .IF_INST32(IF_INST32), .ID_INST32(ID_INST32),
    .CondBtaken_CTRL(CondBtaken_CTRL), .DataHazard(DataHazard), .DO0_CTRL(DO0_CTRL), 
    .DO1_CTRL(DO1_CTRL), .DO2_CTRL(DO2_CTRL), .DIN_CTRL(DIN_CTRL), .PCSel(PCSel),
    .PC_LOAD_CTRL(PC_LOAD_CTRL), .IF_ID_FLUSH_CTRL(IF_ID_FLUSH_CTRL),
    .IF_ID_LOAD_CTRL(IF_ID_LOAD_CTRL), .ID_EX_FLUSH_CTRL(ID_EX_FLUSH_CTRL),
    .RA0_CTRL(RA0_CTRL), .RA1_CTRL(RA1_CTRL), .RA2_CTRL(RA2_CTRL),
    .WA1_CTRL(WA1_CTRL), .ImmSel_CTRL(ImmSel_CTRL), .RmSel_CTRL(RmSel_CTRL),
    .ALUSel_CTRL(ALUSel_CTRL), .APSR_Update_CTRL(APSR_Update_CTRL),
    .DRW_CTRL(DRW_CTRL), .DSIZE_CTRL(DSIZE_CTRL), .LDALUSel_CTRL(LDALUSel_CTRL),
    .WBSel_CTRL(WBSel_CTRL), .RegWE_CTRL(RegWE_CTRL), .DMODE_CTRL(DMODE_CTRL),
    .REGMODE_CTRL(REGMODE_CTRL), .DOUT_MCTRL(DOUT_MCTRL), .DADDR_MCTRL(DADDR_MCTRL),
    .DRW_MCTRL(DRW_MCTRL), .DSIZE_MCTRL(DSIZE_MCTRL), .DI1_MCTRL(DI1_MCTRL),
    .WA1_MCTRL(WA1_MCTRL), .WEN1_MCTRL(WEN1_MCTRL), .POPPED_PC_MCTRL(POPPED_PC_MCTRL),
    .NUM_INST_MCTRL(NUM_INST_MCTRL)
  );

  assign RESET_N_CTRL = RESET_N;
  assign CLK_CTRL = CLK;
  assign IF_INST16 = (INST_SELECTOR) ? INSTR[31:16] : INSTR[15:0];
  assign IF_INST32 = INSTR;
  assign ID_INST16 = IR16;
  assign ID_INST32 = IR32;
  assign CondBtaken_CTRL = CondBtaken;
  assign DataHazard = DataHazard_F;
  assign DO0_CTRL = RD0;
  assign DO1_CTRL = RD1;
  assign DO2_CTRL = RD2;
  assign DIN_CTRL = DIN;

  /* ==================== For Outputs ==================== */
  assign IADDR = PC;
  assign DREQ = RESET_N;
  assign DADDR = (DMODE_CTRL) ? DADDR_MCTRL : ALUOut1;
  assign DRW = (DMODE_CTRL) ? DRW_MCTRL : DRW2;
  assign DSIZE = (DMODE_CTRL) ? DSIZE_MCTRL : DSIZE2;
  assign DOUT = MODIFIED;
  assign OUTPUT_PORT = (RegWE3) ? DI1 : 
                      (DRW3) ? DOUTReg3 : 32'hffff0000;

  /* ==================== Pipeline registers updates ==================== */
  initial begin
    NUM_INST <= 0;
    PC <= 32'h000000d0;
    // PC <= 32'b0;
  end

  always @(negedge CLK) begin
    APSR <= AO;

    /* IF stage */
    if(!RESET_N) begin
      PC0 <= 32'b0;
      IR32 <= 32'b0;
      IR16 <= 16'b0;
    end
    else begin
      if(PC_LOAD_CTRL) begin
        PC <= (PCSel == 2'b00) ? POPPED_PC_MCTRL : 
              (PCSel == 2'b01) ? PC + 2 : 
              (PCSel == 2'b10) ? Prediction : AG_ADDR;
      end
      if(IF_ID_FLUSH_CTRL) begin
        PC0 <= 32'b0;
        IR16 <= 16'b0;
        IR32 <= 32'b0;
      end
      else if(IF_ID_LOAD_CTRL) begin
        PC0 <= PC;
        IR16 <= (INST_SELECTOR) ? INSTR[31:16] : INSTR[15:0];
        IR32 <= INSTR;
      end
    end

    /* ID stage */
    if(ID_EX_FLUSH_CTRL) begin
      PC1 <= 32'b0;
      Rt <= 32'b0;
      Rn <= 32'b0;
      Rm <= 32'b0;
      Rt1 <= 4'b0;
      Rn1 <= 4'b0;
      Rm1 <= 4'b0;
      Rd1 <= 4'b0;
      ALUSel1 <= 5'b0;
      APSR_Update1 <= 1'b0;
      RmSel1 <= 1'b0;
      DRW1 <= 1'b0;
      DSIZE1 <= 2'b11;
      LDALUSel1 <= 3'b0;
      WBSel1 <= 1'b0;
      RegWE1 <= 1'b0;
      INST_COUNT1 <= 1'b0;
    end
    else begin
      PC1 <= PC0;
      Rt <= RD0;
      Rn <= RD1;
      Rm <= (RmSel_CTRL) ? imm : RD2;
      Rt1 <= RA0_CTRL;
      Rn1 <= RA1_CTRL;
      Rm1 <= RA2_CTRL;
      Rd1 <= WA1_CTRL;
      ALUSel1 <= ALUSel_CTRL;
      APSR_Update1 <= APSR_Update_CTRL;
      RmSel1 <= RmSel_CTRL;
      DRW1 <= DRW_CTRL;
      DSIZE1 <= DSIZE_CTRL;
      LDALUSel1 <= LDALUSel_CTRL;
      WBSel1 <= WBSel_CTRL;
      RegWE1 <= RegWE_CTRL;
      INST_COUNT1 <= 1'b1;
    end

    /* EX stage */
    ALUOut1 <= C;
    DOUTReg <= (DOUTSel == 2'b00) ? Rt : 
                (DOUTSel == 2'b01) ? ALUOut1 : 
                (DOUTSel == 2'b10) ? MODIFIED_DATA : DI1;
    Rt2 <= Rt1;
    Rn2 <= Rn1;
    Rm2 <= Rm1;
    Rd2 <= Rd1;
    DRW2 <= DRW1;
    DSIZE2 <= DSIZE1;
    LDALUSel2 <= LDALUSel1;
    WBSel2 <= WBSel1;
    RegWE2 <= RegWE1;
    INST_COUNT2 <= INST_COUNT1;

    /* MEM stage */
    MDR <= MODIFIED_DATA;
    ALUOut2 <= ALUOut1;
    DOUTReg3 <= DOUTReg;
    Rt3 <= Rt2;
    Rn3 <= Rn2;
    Rm3 <= Rm2;
    Rd3 <= Rd2;
    DRW3 <= DRW2;
    WBSel3 <= WBSel2;
    RegWE3 <= RegWE2;
    INST_COUNT3 <= INST_COUNT2;

    /* WB stage */

  end

  always @(posedge CLK) begin
    INST_SELECTOR <= IADDR[1];
    if(NUM_INST_MCTRL) begin
      NUM_INST <= NUM_INST + 1;
    end
    else if(RESET_N && INST_COUNT3) begin
      NUM_INST <= NUM_INST + 1;
    end
  end

endmodule

/*===================================================*/
// your code here (for other modules)

/* ALU */
module ALU (
  input wire [31:0] A,
  input wire [31:0] B,
  
  input wire [4:0] ALUSel,
  input wire APSR_Update,
  input wire [3:0] AI, // N Z C V

  output reg [31:0] C,
  output reg [3:0] AO // N Z C V
);

  /* No APSR updates cases : 
  ADD(reg2), ADD(SP imm1), ADD(SP imm2), SUB(SP imm), MOV(reg1), 
  LDR(PC-rel), ADR, SXTH, SXTB, UXTH, UXTB, push, pop,
  STM, LDM, REV, REV16, REVSH, BX, BLX, B_cond, STR-LDRSH, 
  STR(5bimm)-LDR(8bimm) */
  reg [32:0] temp33_A, temp33_B, temp33_C;
  reg [31:0] temp32_A, temp32_B;

  always @(*) begin

    temp33_A = A; 
    temp33_B = B;
    temp33_C = 33'b0;
    temp32_A = A;
    temp32_B = B;

    case(ALUSel)

      0 : begin // lsl_imm
        C = A << B;
        if(B != 0) begin
          AO[3] = C[31];
          AO[2] = (C == 0);
          AO[1] = (B > 32) ? 1'b0 : A[32-B];
          AO[0] = AI[0];
        end
      end

      1 : begin // lsr_imm
        if(B == 0) temp32_B = 32;
        C = A >> temp32_B;
        AO[3] = C[31];
        AO[2] = (C == 0);
        AO[1] = (temp32_B > 32) ? 1'b0 : A[temp32_B-1];
        AO[0] = AI[0];
      end

      2 : begin // asr_imm
        if(B == 0) temp32_B = 32;
        C = $signed(A) >>> temp32_B;
        AO[3] = C[31];
        AO[2] = (C == 0);
        AO[1] = (temp32_B > 32) ? A[31] : A[temp32_B-1];
        AO[0] = AI[0];
      end

      3 : begin // lsl_reg
        temp32_B = B & 32'h000000ff;
        C = A << temp32_B;
        if(temp32_B != 0) begin
          AO[3] = C[31];
          AO[2] = (C == 0);
          AO[1] = (temp32_B > 32) ? 1'b0 : A[32-temp32_B];
          AO[0] = AI[0];
        end
      end

      4 : begin // lsr_reg
        temp32_B = B & 32'h000000ff;
        C = A << temp32_B;
        if(temp32_B != 0) begin
          AO[3] = C[31];
          AO[2] = (C == 0);
          AO[1] = A[temp32_B-1];
          AO[0] = AI[0];
        end
      end

      5 : begin // asr_reg
        temp32_B = B & 32'h000000ff;
        C = $signed(A) >>> temp32_B;
        if(temp32_B != 0) begin
          AO[3] = C[31];
          AO[2] = (C == 0);
          AO[1] = A[temp32_B-1];
          AO[0] = AI[0];
        end
      end

      6 : begin // add
        temp33_C = temp33_A + temp33_B;
        C = temp33_C[31:0];
        if(APSR_Update) begin
          AO[3] = C[31];
          AO[2] = (C == 0);
          AO[1] = (temp33_C[32]);
          AO[0] = (A[31] == B[31] && C[31] != A[31]);
        end
        else begin
          AO[3] = AI[3];
          AO[2] = AI[2];
          AO[1] = AI[1];
          AO[0] = AO[0];
        end
      end

      7 : begin // sub
        temp33_B = ~B;
        temp33_C = temp33_A + temp33_B + 1;
        C = temp33_C[31:0];
        if(APSR_Update) begin
          AO[3] = C[31];
          AO[2] = (C == 0);
          AO[1] = !(temp33_C[32]);
          AO[0] = ((A[31] > B[31] && !C[31]) || (A[31] < B[31] && C[31]));
        end
        else begin
          AO[3] = AI[3];
          AO[2] = AI[2];
          AO[1] = AI[1];
          AO[0] = AO[0];
        end
      end

      8 : begin // adc
        temp33_C = temp33_A + temp33_B + AI[1];
        C = temp33_C[31:0];
        if(APSR_Update) begin
          AO[3] = C[31];
          AO[2] = (C == 0);
          AO[1] = (temp33_C[32]);
          AO[0] = (A[31] == B[31] && C[31] != A[31]);
        end
        else begin
          AO[3] = AI[3];
          AO[2] = AI[2];
          AO[1] = AI[1];
          AO[0] = AO[0];
        end
      end

      9 : begin // sbc
        temp33_B = ~B;
        temp33_C = temp33_A + temp33_B + AI[1];
        C = temp33_C[31:0];
        if(APSR_Update) begin
          AO[3] = C[31];
          AO[2] = (C == 0);
          AO[1] = !(temp33_C[32]);
          AO[0] = ((A[31] > B[31] && !C[31]) || (A[31] < B[31] && C[31]));
        end
        else begin
          AO[3] = AI[3];
          AO[2] = AI[2];
          AO[1] = AI[1];
          AO[0] = AO[0];
        end
      end

      10 : begin // adr, ldr
        C = ((A+4) & 32'hfffffffc) + B;
      end

      11 : begin // mov_imm
        C = B;
        AO[3] = C[31];
        AO[2] = (C == 0);
        AO[1] = AI[1];
        AO[0] = AI[0];
      end

      12 : begin // mov_reg1
        C = B;
        AO[3] = AI[3];
        AO[2] = AI[2];
        AO[1] = AI[1];
        AO[0] = AO[0];
      end

      13 : begin // and
        C = A & B;
        AO[3] = C[31];
        AO[2] = (C == 0);
        AO[1] = AI[1];
        AO[0] = AI[0];
      end

      14 : begin // or
        C = A | B;
        AO[3] = C[31];
        AO[2] = (C == 0);
        AO[1] = AI[1];
        AO[0] = AI[0];
      end

      15 : begin // not
        C = ~B;
        AO[3] = C[31];
        AO[2] = (C == 0);
        AO[1] = AI[1];
        AO[0] = AI[0];
      end
      
      16 : begin // xor
        C = A ^ B;
        AO[3] = C[31];
        AO[2] = (C == 0);
        AO[1] = AI[1];
        AO[0] = AI[0];
      end

      17 : begin // bic
        C = A & (~B);
        AO[3] = C[31];
        AO[2] = (C == 0);
        AO[1] = AI[1];
        AO[0] = AI[0];
      end

      18 : begin // ror
        temp32_B = B & 32'h000000ff;
        if(temp32_B == 0) begin
          C = A;
          AO[3] = AI[3];
          AO[2] = AI[2];
          AO[1] = AI[1];
          AO[0] = AO[0];
        end
        else begin
          C = (A >> B) | (A << (32-B));
          AO[3] = C[31];
          AO[2] = (C == 0);
          AO[1] = C[31];
          AO[0] = AI[0];
        end
      end

      19 : begin // mul
        C = A * B;
        AO[3] = C[31];
        AO[2] = (C == 0);
        AO[1] = AI[1];
        AO[0] = AI[0];
      end

      20 : begin // reglist count
        C = B[0] + B[1] + B[2] + B[3] + B[4] + B[5] + B[6] + B[7];
      end

      21 : begin // sign extend halfword
        C = {{16{B[15]}}, B[15:0]};
      end
      
      22 : begin // sign extend byte
        C = {{24{B[15]}}, B[7:0]};
      end

      23 : begin // unsign extend halfword
        C = {16'b0, B[15:0]};
      end

      24 : begin // unsign extend byte
        C = {24'b0, B[7:0]};
      end

      25 : begin // rev
        C = {B[7:0], B[15:8], B[23:16], B[31:24]};
      end

      26 : begin // rev16
        C = {B[23:16], B[31:24], B[7:0], B[15:8]};
      end
      
      27 : begin // revsh
        C = {{16{B[7]}}, B[7:0], B[15:8]};
      end

      28 : begin // BLX
        C = (A+2) | 32'b1; // store into LR later. A is currentPC-4
      end

      29 : begin // BL
        C = (A+4) | 32'b1; // store into LR later. A is currentPC-4
      end

      30 : begin // RSB
        C = -A;
      end

      default : C = 0;

    endcase
  end

endmodule

/* ALU for loaded value from D$ */
module LDALU (
  input wire [31:0] RAW_DATA,
  input wire [1:0] OFFSET,
  input wire [2:0] LDALUSel,
  output reg [31:0] MODIFIED_DATA
);

  always @(*) begin
    if(LDALUSel == 3'b000) begin // sign extend byte (LDRSB)
      if(OFFSET == 2'b00) begin
        MODIFIED_DATA = {{24{RAW_DATA[7]}}, RAW_DATA[7:0]};
      end
      else if(OFFSET == 2'b01) begin
        MODIFIED_DATA = {{24{RAW_DATA[15]}}, RAW_DATA[15:8]};
      end
      else if(OFFSET == 2'b10) begin
        MODIFIED_DATA = {{24{RAW_DATA[23]}}, RAW_DATA[23:16]};
      end
      else if(OFFSET == 2'b11) begin
        MODIFIED_DATA = {{24{RAW_DATA[31]}}, RAW_DATA[31:24]};
      end
    end

    else if(LDALUSel == 3'b001) begin // sign extend halfword (LDRSH)
      if(OFFSET == 2'b00 || OFFSET == 2'b01) begin
        MODIFIED_DATA = {{16{RAW_DATA[15]}}, RAW_DATA[15:0]};
      end
      else begin
        MODIFIED_DATA = {{16{RAW_DATA[31]}}, RAW_DATA[31:16]};
      end
    end

    else if(LDALUSel == 3'b010) begin // unsign extend byte (LDRB)
      if(OFFSET == 2'b00) begin
        MODIFIED_DATA = {24'b0, RAW_DATA[7:0]};
      end
      else if(OFFSET == 2'b01) begin
        MODIFIED_DATA = {24'b0, RAW_DATA[15:8]};
      end
      else if(OFFSET == 2'b10) begin
        MODIFIED_DATA = {24'b0, RAW_DATA[23:16]};
      end
      else if(OFFSET == 2'b11) begin
        MODIFIED_DATA = {24'b0, RAW_DATA[31:24]};
      end
    end

    else if(LDALUSel == 3'b011) begin // unsign extend halfword (LDRH)
      if(OFFSET == 2'b00 || OFFSET == 2'b01) begin
        MODIFIED_DATA = {16'b0, RAW_DATA[15:0]};
      end
      else begin
        MODIFIED_DATA = {16'b0, RAW_DATA[31:16]};
      end
    end

    else begin // LDR
      MODIFIED_DATA = RAW_DATA;
    end
  end
  // assign MODIFIED_DATA = (LDALUSel == 3'b000) ? {{24{RAW_DATA[7]}}, RAW_DATA[7:0]} : // sign extend byte (LDRSB)
  //             (LDALUSel == 3'b001) ? {{16{RAW_DATA[15]}}, RAW_DATA[15:0]} : // sign extend halfword (LDRSH)
  //             (LDALUSel == 3'b010) ? {24'b0, RAW_DATA[7:0]} : // unsign extend byte (LDRB)
  //             (LDALUSel == 3'b011) ? {16'b0, RAW_DATA[15:0]} : RAW_DATA; // unsign extend halfword (LDRH) | (LDR)

endmodule

module STALU (
  input wire [31:0] ORIGINAL,
  input wire [1:0] OFFSET2,
  input wire [1:0] DSIZE_ST,
  output reg [31:0] MODIFIED
);

  always @(*) begin
    if(DSIZE_ST == 2'b00) begin
      if(OFFSET2 == 2'b00) begin
        MODIFIED = {24'b0, ORIGINAL[7:0]};
      end
      else if(OFFSET2 == 2'b01) begin
        MODIFIED = {16'b0, ORIGINAL[7:0], 8'b0};
      end
      else if(OFFSET2 == 2'b10) begin
        MODIFIED = {8'b0, ORIGINAL[7:0], 16'b0};
      end
      else begin
        MODIFIED = {ORIGINAL[7:0], 24'b0};
      end
    end

    else if(DSIZE_ST == 2'b01) begin
      if(OFFSET2 == 2'b00 || OFFSET2 == 2'b01) begin
        MODIFIED = {16'b0, ORIGINAL[15:0]};
      end
      else begin
        MODIFIED = {ORIGINAL[15:0], 16'b0};
      end
    end

    else if(DSIZE_ST == 2'b10) begin
      MODIFIED = ORIGINAL;
    end 

    else begin
      MODIFIED = 32'b0;
    end
  end

endmodule

/* Immediate Generator */
module ImmGen (
  input wire [15:0] INST,
  input wire [2:0] immsel,

  output reg [31:0] imm
);

  always @(*) begin
    case (immsel) 
      0 : begin // 10:6 / LSL_imm, LSR_imm, ASR_imm, STRB, LDRB 
        imm = {27'b0, INST[10:6]};
      end
      1 : begin // 10:6 * 2 / STRH_imm, LDRH_imm
        imm = {26'b0, INST[10:6], 1'b0};
      end
      2 : begin // 10:6 * 4 / STR(5bimm), LDR(5bimm)
        imm = {25'b0, INST[10:6], 2'b0};
      end
      3 : begin // 8:6 / ADD(3bimm), SUB(3bimm)
        imm = {29'b0, INST[8:6]};
      end
      4 : begin // 7:0 /  MOV_imm, CMP_imm, ADD(8bimm), SUB(8bimm), STR(8bimm), LDR(8bimm)
        imm = {24'b0, INST[7:0]};
      end
      5 : begin // 7:0 * 4 / LDR(rel), ADR, ADD(SP imm1), STR(8bimm), LDR(8bimm)
        imm = {22'b0, INST[7:0], 2'b0};
      end
      6 : begin // 6:0 * 4 / ADD(SP_imm2), SUB(SP_imm)
        imm = {23'b0, INST[6:0], 2'b0};
      end
      default : begin // RSB
        imm = 32'b0;
      end
    endcase
  end

endmodule

/* Forwarding logic */
module Forwarding (
  input wire [15:0] F_ID_INST16,
  input wire [3:0] ID_rm,

  input wire [3:0] EX_rt,
  input wire [3:0] EX_rn,
  input wire [3:0] EX_rm,

  input wire [3:0] EX_rd,
  input wire [3:0] MEM_rd,
  input wire [3:0] WB_rd,

  input wire RegWE_EX,
  input wire RegWE_MEM,
  input wire RegWE_WB,

  input wire RmSel_EX,
  input wire [4:0] ALUSel_EX,
  input wire WBSel_EX,
  input wire WBSel_MEM,

  output reg [2:0] ASel_F,
  output reg [1:0] BSel_F,
  output reg [1:0] DOUTSel_F,
  output reg [2:0] AGSel_F,
  output reg DataHazard_F
);

  always @(*) begin
    
    /* A mux selector */
    if(ALUSel_EX == 10 || ALUSel_EX == 28 || ALUSel_EX == 29) ASel_F = 3'b001; // ldr or adr
    else if(EX_rn == MEM_rd && RegWE_MEM && WBSel_MEM) ASel_F = 3'b011;
    else if(EX_rn == MEM_rd && RegWE_MEM && !WBSel_MEM) ASel_F = 3'b010;
    else if(EX_rn == WB_rd && RegWE_WB) ASel_F = 3'b100;
    else ASel_F = 3'b000;

    /* B mux selector */
    if(RmSel_EX == 1'b1) BSel_F = 2'b00; // imm
    else if(EX_rm == MEM_rd && RegWE_MEM && WBSel_MEM) BSel_F = 2'b10;
    else if(EX_rm == MEM_rd && RegWE_MEM && !WBSel_MEM) BSel_F = 2'b01;
    else if(EX_rm == WB_rd && RegWE_WB) BSel_F = 2'b11;
    else BSel_F = 2'b00;

    /* DOUT mux selector */
    if(EX_rt == MEM_rd && RegWE_MEM && WBSel_MEM) DOUTSel_F = 2'b01;
    else if(EX_rt == MEM_rd && RegWE_MEM && !WBSel_MEM) DOUTSel_F = 2'b10;
    else if(EX_rt == WB_rd && RegWE_WB) DOUTSel_F = 2'b11;
    else DOUTSel_F = 2'b00;

    /* AG mux selector */
    if(ID_rm == EX_rd && RegWE_EX) AGSel_F = 3'b001;
    else if(ID_rm == MEM_rd && RegWE_MEM && !WBSel_MEM) AGSel_F = 3'b010;
    else if(ID_rm == MEM_rd && RegWE_MEM && WBSel_MEM) AGSel_F = 3'b011;
    else if(ID_rm == WB_rd && RegWE_WB) AGSel_F = 3'b100;
    else AGSel_F = 3'b000;

    /* DataHazard */
    if(F_ID_INST16[15:8] == 8'b01000111 && ID_rm == EX_rd && RegWE_EX && !WBSel_EX) DataHazard_F = 1'b1;
    else DataHazard_F = 1'b0;

  end

endmodule

/* Branch Predictor */
module Branch_Predictor (
  input wire [15:0] BP_ID_INST16,
  input wire [15:0] BP_IF_INST16,
  input wire [31:0] BP_IADDR,
  output reg [31:0] Prediction
);

  always @(*) begin
      Prediction = (BP_ID_INST16[15:11] == 5'b11110) ? ((BP_IADDR + {{8{BP_ID_INST16[10]}}, !(BP_IF_INST16[13]^BP_ID_INST16[10]), !(BP_IF_INST16[11]^BP_ID_INST16[10]), BP_ID_INST16[9:0], BP_IF_INST16[10:0], 1'b0} + 2) & 32'hfffffffe) : // BL
                    (BP_IF_INST16[15:11] == 5'b11100) ? ((BP_IADDR + {{20{BP_IF_INST16[10]}}, BP_IF_INST16[10:0], 1'b0} + 4) & 32'hfffffffe) : // unconditional B
                      (BP_IF_INST16[15:12] == 4'b1101) ? ((BP_IADDR + {{23{BP_IF_INST16[7]}}, BP_IF_INST16[7:0], 1'b0} + 4) & 32'hfffffffe) : 0; // conditional B (predict - always taken)
  end

endmodule

module AddrGen (
  input wire [31:0] AG_DOUT2,
  input wire [3:0] AG_APSR, // N Z C V
  input wire [15:0] AG_INST,
  input wire [31:0] AG_PC,
  output reg [31:0] AG_ADDR,
  output reg CondBtaken
);

  reg [31:0] imm_cond;
  reg B;

  initial begin
    imm_cond <= 32'b0;
    B <= 1'b0;
  end

  always @(*) begin

    imm_cond = {{23{AG_INST[7]}}, AG_INST[7:0], 1'b0};

    if(AG_INST[15:12] == 4'b1101) begin // conditional B detected
      case(AG_INST[11:8])
        0 : B = (AG_APSR[2] == 1); // EQ
        1 : B = (AG_APSR[2] == 0); // NE
        2 : B = (AG_APSR[1] == 1); // CS
        3 : B = (AG_APSR[1]== 0); // CC
        4 : B = (AG_APSR[3] == 1); // MI
        5 : B = (AG_APSR[3] == 0); // PL
        6 : B = (AG_APSR[0] == 1); // VS
        7 : B = (AG_APSR[0] == 0); // VC
        8 : B = (AG_APSR[1] == 1 && AG_APSR[2] == 0); // HI
        9 : B = (AG_APSR[1] == 0 || AG_APSR[2] == 1); // LS
        10 : B = (AG_APSR[3] == AG_APSR[0]); // GE
        11 : B = (AG_APSR[3] != AG_APSR[0]); // LT
        12 : B = (AG_APSR[2] == 0 && AG_APSR[2] == AG_APSR[3]); // GT
        13 : B = (AG_APSR[2] == 1 && AG_APSR[2] != AG_APSR[3]); // LE
        14 : B = 1; // None(AL)
        default : B = 1'b0;
      endcase

      if(B) begin // Branch taken. 
        AG_ADDR = (AG_PC + imm_cond + 4) & 32'hfffffffe; 
        CondBtaken = 1'b1; 
      end
      else begin // Branch not taken
        AG_ADDR = AG_PC + 2; 
        CondBtaken = 1'b0; 
      end 
    end

    else if(AG_INST[15:8] == 8'b01000111) begin // BX or BLX
      AG_ADDR = AG_DOUT2 & 32'hfffffffe; // Branch always taken
      CondBtaken = 1'b0;
    end

    else begin // Branch not taken
      AG_ADDR = AG_PC + 2;
      CondBtaken = 1'b0;
    end

  end

endmodule

module CTRL (
  input wire RESET_N_CTRL,
  input wire CLK_CTRL,

  input wire [15:0] IF_INST16,
  input wire [31:0] IF_INST32,
  input wire [15:0] ID_INST16,
  input wire [31:0] ID_INST32,

  input wire CondBtaken_CTRL,
  input wire DataHazard,

  input wire [31:0] DO0_CTRL,
  input wire [31:0] DO1_CTRL,
  input wire [31:0] DO2_CTRL,
  input wire [31:0] DIN_CTRL,

  /* CTRL signals for common pipeline behavior */
  output reg [1:0] PCSel,       // ctrl 1
  output reg PC_LOAD_CTRL,      // ctrl 2
  output reg IF_ID_FLUSH_CTRL,  // ctrl 3
  output reg IF_ID_LOAD_CTRL,   // ctrl 4
  output reg ID_EX_FLUSH_CTRL,  //
  output reg [3:0] RA0_CTRL,    // ctrl 
  output reg [3:0] RA1_CTRL,    // ctrl 
  output reg [3:0] RA2_CTRL,    // ctrl 
  output reg [3:0] WA1_CTRL,    // ctrl 
  output reg [2:0] ImmSel_CTRL, // ctrl 
  output reg RmSel_CTRL,             // ctrl 
  output reg [4:0] ALUSel_CTRL ,// ctrl 
  output reg APSR_Update_CTRL,  // ctrl 
  output reg DRW_CTRL,          // ctrl 
  output reg [1:0] DSIZE_CTRL,  // ctrl 
  output reg [2:0] LDALUSel_CTRL, // ctrl 
  output reg WBSel_CTRL,        // ctrl 
  output reg RegWE_CTRL,       // 

  /* CTRL signals for STM, LDM, PUSH, POP instructions */
  output reg DMODE_CTRL,         // Control D$ at active high
  output reg REGMODE_CTRL,       // Control regfile at active high

  /* for D$ */
  output reg [31:0] DOUT_MCTRL,
  output reg [31:0] DADDR_MCTRL,
  output reg DRW_MCTRL,
  output reg [1:0] DSIZE_MCTRL,
  
  /* for regfile */
  output reg [31:0] DI1_MCTRL,
  output reg [3:0] WA1_MCTRL,
  output reg WEN1_MCTRL,

  output reg [31:0] POPPED_PC_MCTRL,

  /* for tests */
  output reg NUM_INST_MCTRL
);

  reg [31:0] CACHE [8:0];
  reg [31:0] SP;
  reg [1:0] count1;
  reg [3:0] count2;
  reg [3:0] count3;
  reg IF_ID_FLUSH; // IF_ID flush indicator
  reg POP_PC; // Indicates that PC is just popped from the stack

  reg [8:0] reglist;
  reg [3:0] regcnt;
  reg [3:0] regcnt_stldm;;

  initial begin
    reglist <= 9'b0;
    regcnt <= 4'b0;
    regcnt_stldm <= 4'b0;
    CACHE[0] <= 32'b0;
    CACHE[1] <= 32'b0;
    CACHE[2] <= 32'b0;
    CACHE[3] <= 32'b0;
    CACHE[4] <= 32'b0;
    CACHE[5] <= 32'b0;
    CACHE[6] <= 32'b0;
    CACHE[7] <= 32'b0;
    CACHE[8] <= 32'b0;
    SP <= 32'b0;
    count1 <= 2'b0;
    count2 <= 4'b0;
    count3 <= 4'b0;
    IF_ID_FLUSH <= 1'b0;
    POP_PC <= 1'b0;
  end

  always @(*) begin

    reglist = ID_INST16[8:0];
    regcnt = reglist[0]+reglist[1]+reglist[2]+reglist[3]+reglist[4]+reglist[5]+reglist[6]+reglist[7]+reglist[8];
    regcnt_stldm = reglist[0]+reglist[1]+reglist[2]+reglist[3]+reglist[4]+reglist[5]+reglist[6]+reglist[7];

    /* PCSel : determine nextPC */
    if(POP_PC) PCSel = 2'b00; // PC is popped from the stack
    else if(IF_INST16[15:12] == 4'b1101 || IF_INST16[15:11] == 5'b11100 || ID_INST16[15:11] == 5'b11110) PCSel = 2'b10; // B(uncond), B(cond), BL
    else if(ID_INST16[15:8] == 8'b01000111 || (ID_INST16[15:12] == 4'b1101 && !CondBtaken_CTRL)) PCSel = 2'b11; // BX, BLX, Branch misprediction
    else PCSel = 2'b01;

    /* PCLOAD_CTRL : whether udpate pc or not */
    if(POP_PC) PC_LOAD_CTRL = 1'b1;
    else if(IF_INST16[15:8] == 8'b01000111) PC_LOAD_CTRL = 1'b0; // BX or BLX is in IF stage
    else if(DataHazard) PC_LOAD_CTRL = 1'b0;
    else if(ID_INST16[15:9] == 7'b1011010 || ID_INST16[15:9] == 7'b1011110 || ID_INST16[15:11] == 5'b11000 || ID_INST16[15:11] == 5'b11001) PC_LOAD_CTRL = 1'b0; // pop/push/stm/ldm in ID stage
    else PC_LOAD_CTRL = 1'b1;

    /* IF_ID_FLUSH_CTRL : whether flush pipeline regs at IF/ID.
    Occurs at BX or BLX with no DataHazard, branch misprediction, and end of push/pop/stm/ldm process */
    if(IF_ID_FLUSH) IF_ID_FLUSH_CTRL = 1'b1; // end of push/pop/stm/ldm process
    else if(ID_INST16[15:12] == 4'b1101 && !CondBtaken_CTRL) IF_ID_FLUSH_CTRL = 1'b1; // BX or BLX is in ID stage or cond_B misprediction
    else if(ID_INST16[15:8] == 8'b01000111 && !DataHazard) IF_ID_FLUSH_CTRL = 1'b1; 
    else if(ID_INST16[15:11] == 5'b11110) IF_ID_FLUSH_CTRL = 1'b1;
    else IF_ID_FLUSH_CTRL = 1'b0;

    /* IF_ID_LOAD_CTRL */
    if(ID_INST16[15:9] == 7'b1011010 || ID_INST16[15:9] == 7'b1011110 || ID_INST16[15:11] == 5'b11000 || ID_INST16[15:11] == 5'b11001) IF_ID_LOAD_CTRL = 1'b0;
    else if(DataHazard) IF_ID_LOAD_CTRL = 1'b0;
    else IF_ID_LOAD_CTRL = 1'b1;

    /* ID_EX_FLUSH_CTRL */
    if(ID_INST16[15:9] == 7'b1011010 || ID_INST16[15:9] == 7'b1011110 || ID_INST16[15:11] == 5'b11000 || ID_INST16[15:11] == 5'b11001) ID_EX_FLUSH_CTRL = 1'b1; // pop/push/stm/ldm in ID stage
    else if(DataHazard) ID_EX_FLUSH_CTRL = 1'b1; 
    else if(ID_INST16 == 16'b0) ID_EX_FLUSH_CTRL = 1'b1;
    else ID_EX_FLUSH_CTRL = 1'b0;

    /* RA0_CTRL : Identify address of Rt */
    if(ID_INST16[15:9] == 7'b1011010 || ID_INST16[15:11] == 5'b11000) RA0_CTRL = (count1 == 2'b11) ? 13 : count1*3; // PUSH, STM
    else if(ID_INST16[15:9] == 7'b1011110) RA0_CTRL = 13; // POP
    else if(ID_INST16[15:11] == 5'b11001) RA0_CTRL = ID_INST16[10:8]; // LDM
    else if(ID_INST16[15:11] == 5'b01001) RA0_CTRL = ID_INST16[10:8]; // LDR
    else if(ID_INST16[15:12] == 4'b0101 || ID_INST16[15:13] == 3'b011 || (ID_INST16[15:13] == 3'b100 && !ID_INST16[12])) RA0_CTRL = ID_INST16[2:0]; // STR(reg) - LDRH(imm)
    else if(ID_INST16[15:13] == 3'b100 && ID_INST16[12]) RA0_CTRL = ID_INST16[10:8]; // STR(8bimm), LDR(8bimm)
    else RA0_CTRL = 4'b0;

    /* RA1_CTRL : Identify address of Rn */
    if(ID_INST16[15:9] == 7'b1011010 || ID_INST16[15:11] == 5'b11000) RA1_CTRL = count1*3 + 1; // PUSH, STM
    else if(ID_INST16[15:13] == 3'b000) RA1_CTRL = ID_INST16[5:3]; // LSL(imm) - SUB(3bimm)
    else if(ID_INST16[15:13] == 3'b001) RA1_CTRL = ID_INST16[10:8]; // MOV(imm) - SUB(8bimm)
    else if(ID_INST16[15:10] == 6'b010000 && ID_INST16[9:6] == 4'b1001) RA1_CTRL = ID_INST16[5:3]; // RSB
    else if(ID_INST16[15:10] == 6'b010000) RA1_CTRL = ID_INST16[2:0]; // AND - MVN
    else if(ID_INST16[15:8] == 8'b01000100) RA1_CTRL = {ID_INST16[7], ID_INST16[2:0]}; // ADD(reg2)
    else if(ID_INST16[15:8] == 8'b01000101) RA1_CTRL = {ID_INST16[7], ID_INST16[2:0]}; // CMP(reg2)
    else if(ID_INST16[15:12] == 4'b0101 || ID_INST16[15:13] == 3'b011 || (ID_INST16[15:13] == 3'b100 && !ID_INST16[12])) RA1_CTRL = ID_INST16[5:3]; // STR(reg) - LDRH(imm)
    else if(ID_INST16[15:11] == 5'b10101 || ID_INST16[15:8] == 8'b10110000 || ID_INST16[15:12] == 4'b1001) RA1_CTRL = 13; // for stack pointer instructions
    else RA1_CTRL = 4'b0;

    /* RA2_CTRL : Identify address of Rm */
    if(ID_INST16[15:9] == 7'b1011010) RA2_CTRL = (count1 == 2'b10) ? 14 : count1*3 + 2; // PUSH
    else if(ID_INST16[15:11] == 5'b11000) RA2_CTRL = (count1 == 2'b10) ? ID_INST16[10:8] : count1*3 + 2; // STM
    else if(ID_INST16[15:11] == 5'b00011) RA2_CTRL = ID_INST16[8:6];
    else if(ID_INST16[15:10] == 6'b010000) RA2_CTRL = ID_INST16[5:3];
    else if(ID_INST16[15:10] == 6'b010001) RA2_CTRL = ID_INST16[6:3];
    else if(ID_INST16[15:12] == 4'b0101) RA2_CTRL = ID_INST16[8:6];
    else if(ID_INST16[15:12] == 4'b1011) RA2_CTRL = ID_INST16[5:3];

    /* WA1_CTRL : Identify the writing addr (Rd addr) */
    if(ID_INST16[15:13] == 3'b0) WA1_CTRL = ID_INST16[2:0]; // lsl - sub(3bimm)
    else if(ID_INST16[15:13] == 3'b001) WA1_CTRL = ID_INST16[10:8]; // mov(imm) - sub(8bimm)
    else if(ID_INST16[15:10] == 6'b010000) WA1_CTRL = ID_INST16[2:0]; // and(reg) - mvn(reg)
    else if(ID_INST16[15:10] == 6'b010001) WA1_CTRL = {ID_INST16[7], ID_INST16[2:0]}; // add(reg2) - mov(reg1)
    else if(ID_INST16[15:11] == 5'b01001) WA1_CTRL = ID_INST16[10:8]; // ldr(pc-rel)
    else if(ID_INST16[15:12] == 4'b0101) WA1_CTRL = ID_INST16[2:0]; // str(reg) - ldrsh(reg)
    else if(ID_INST16[15:13] == 3'b011 || ID_INST16[15:12] == 4'b1000) WA1_CTRL = ID_INST16[2:0]; // str(5bimm) - ldr(imm)
    else if(ID_INST16[15:12] == 4'b1001) WA1_CTRL = ID_INST16[10:8]; // str(8bimm), ldr(8bimm)
    else if(ID_INST16[15:12] == 4'b1010) WA1_CTRL = ID_INST16[10:8]; // adr, add(spimm1)
    else if(ID_INST16[15:8] == 8'b10110000) WA1_CTRL = 13; // add(spimm2), sub(spimm)
    else if(ID_INST16[15:12] == 4'b1011) WA1_CTRL = ID_INST16[2:0];
    else if(ID_INST16[15:11] == 5'b11110 || ID_INST16[15:7] == 9'b010001111) WA1_CTRL = 4'b1110; // BL, BLX

    /* ImmSel_CTRL : Determine the behavior of ImmGen module */
    if(ID_INST16[15:11] < 3 || ID_INST16[15:12] == 4'b0111) ImmSel_CTRL = 0;
    else if(ID_INST16[15:12] == 4'b1000) ImmSel_CTRL = 1;
    else if(ID_INST16[15:12] == 4'b0110) ImmSel_CTRL = 2;
    else if(ID_INST16[15:11] == 5'b00011) ImmSel_CTRL = 3;
    else if(ID_INST16[15:13] == 3'b001 || ID_INST16[15:12] == 4'b1001) ImmSel_CTRL = 4;
    else if(ID_INST16[15:11] == 5'b01001 || ID_INST16[15:12] == 4'b1010 || ID_INST16[15:12] == 4'b1001) ImmSel_CTRL = 5;
    else if(ID_INST16[15:8] == 8'b10110000) ImmSel_CTRL = 6;
    else ImmSel_CTRL = 7;

    /* RmSel_CTRL : Select one btw DOUT2 and Imm */
    if(ID_INST16[15:11] <= 2 || ID_INST16[15:10] == 6'b000111 || ID_INST16[15:13] == 3'b001) RmSel_CTRL = 1'b1;
    else if(ID_INST16[15:11] == 5'b01001) RmSel_CTRL = 1'b1;
    else if(ID_INST16[15:12] <= 10 && ID_INST16[15:12] >= 6) RmSel_CTRL = 1'b1;
    else if(ID_INST16[15:8] == 8'b10110000) RmSel_CTRL = 1'b1;
    else RmSel_CTRL = 1'b0;

    /* ALUSel_CTRL : Determine the behavior of ALU */
    if(ID_INST16[15:11] == 5'b00000) ALUSel_CTRL = 0; // lsl_imm
    else if(ID_INST16[15:11] == 5'b00001) ALUSel_CTRL = 1; // lsr_imm
    else if(ID_INST16[15:11] == 5'b00010) ALUSel_CTRL = 2; // asr_imm
    else if(ID_INST16[15:6] == 10'b0100000010) ALUSel_CTRL = 3; // lsl_reg
    else if(ID_INST16[15:6] == 10'b0100000011) ALUSel_CTRL = 4; // lsr_reg
    else if(ID_INST16[15:6] == 10'b0100000100) ALUSel_CTRL = 5; // asr_reg
    // ADD(reg1), ADD(3bimm), ADD(8bimm), ADD(reg2), STR(reg)-LDR(8bimm), ADD(SPimm1), ADD(SPimm2), CMN(reg)
    else if(ID_INST16[15:9] == 7'b0001100 || ID_INST16[15:9] == 7'b0001110 || ID_INST16[15:11] == 5'b00110 || ID_INST16[15:8] == 8'b01000100 ||
    ID_INST16[15:12] == 4'b0101 || ID_INST16[15:13] == 3'b011 || ID_INST16[15:13] == 3'b100 || ID_INST16[15:11] == 5'b10101 || 
    ID_INST16[15:7] == 9'b101100000 || ID_INST16[15:6] == 10'b0100001011) ALUSel_CTRL = 6; 
    // SUB(reg), SUB(3bimm), CMP(imm), SUB(8bimm), CMP(reg1), CMP(reg2), SUB(SPimm)
    else if(ID_INST16[15:9] == 7'b0001101 || ID_INST16[15:9] == 7'b0001111 || ID_INST16[15:11] == 5'b00101 || ID_INST16[15:11] == 5'b00111 ||
    ID_INST16[15:6] == 10'b0100001010 || ID_INST16[15:8] == 8'b01000101 || ID_INST16[15:7] == 9'b101100001) ALUSel_CTRL = 7; 
    else if(ID_INST16[15:6] == 10'b0100000101) ALUSel_CTRL = 8; // adc
    else if(ID_INST16[15:6] == 10'b0100000110) ALUSel_CTRL = 9; // sbc
    else if(ID_INST16[15:11] == 5'b01001 || ID_INST16[15:11] == 5'b10100) ALUSel_CTRL = 10; // adr, ldr
    else if(ID_INST16[15:11] == 5'b00100) ALUSel_CTRL = 11; // mov_imm
    else if(ID_INST16[15:8] == 8'b01000110) ALUSel_CTRL = 12; // mov_reg
    else if(ID_INST16[15:6] == 10'b0100000000 || ID_INST16[15:6] == 10'b0100001000) ALUSel_CTRL = 13; // and
    else if(ID_INST16[15:6] == 10'b0100001100) ALUSel_CTRL = 14; // or
    else if(ID_INST16[15:6] == 10'b0100001111) ALUSel_CTRL = 15; // not
    else if(ID_INST16[15:6] == 10'b0100000001) ALUSel_CTRL = 16; // xor
    else if(ID_INST16[15:6] == 10'b0100001110) ALUSel_CTRL = 17; // bic
    else if(ID_INST16[15:6] == 10'b0100000111) ALUSel_CTRL = 18; // ror
    else if(ID_INST16[15:6] == 10'b0100001101) ALUSel_CTRL = 19; // mul
    // else if() ALUSel_CTRL = 20; // reglist count
    else if(ID_INST16[15:6] == 10'b1011001000) ALUSel_CTRL = 21; // sign extend halfword
    else if(ID_INST16[15:6] == 10'b1011001001) ALUSel_CTRL = 22; // sign extend byte
    else if(ID_INST16[15:6] == 10'b1011001010) ALUSel_CTRL = 23; // unsign extend halfword
    else if(ID_INST16[15:6] == 10'b1011001011) ALUSel_CTRL = 24; // unsign extend byte
    else if(ID_INST16[15:6] == 10'b1011101000) ALUSel_CTRL = 25; // rev
    else if(ID_INST16[15:6] == 10'b1011101001) ALUSel_CTRL = 26; // rev16
    else if(ID_INST16[15:6] == 10'b1011101011) ALUSel_CTRL = 27; // revsh
    else if(ID_INST16[15:7] == 9'b010001111) ALUSel_CTRL = 28; // blx
    else if(ID_INST16[15:11] == 5'b11110) ALUSel_CTRL = 29; // bl
    else if(ID_INST16[15:6] == 10'b0100001001) ALUSel_CTRL = 30; // RSB
    else ALUSel_CTRL = 30; // default

    /* APSR_Update_CTRL */
    /* ADD(reg2), MOV(reg1), BX, BLX, LDR, STR(reg)-LDR(8bimm), ADR, ADD(SPimm1) -- */
    if(ID_INST16[15:8] == 8'b01000100 || ID_INST16[15:9] == 7'b0100011 || ID_INST16[15:11] == 5'b01001 || ID_INST16[15:12] == 4'b0101 || 
    ID_INST16[15:13] == 3'b011 || ID_INST16[15:13] == 3'b100 || ID_INST16[15:12] == 4'b1010 || ID_INST16[15:12] >= 4'b1011) APSR_Update_CTRL = 1'b0;
    else APSR_Update_CTRL = 1'b1;

    /* DRW_CTRL. Write at 1, Read at 0 */
    if((ID_INST16[15:12] == 4'b0101 && ID_INST16[11:9] <= 3'b010) || ID_INST16[15:11] == 5'b01100 || ID_INST16[15:11] == 5'b01110 || 
    ID_INST16[15:11] == 5'b10000 || ID_INST16[15:11] == 5'b10010) DRW_CTRL = 1'b1;
    else DRW_CTRL = 1'b0;

    /* DSIZE_CTRL. word at 2, halfword at 1, byte at 0*/
    if(ID_INST16[15:9] == 7'b0101010 || ID_INST16[15:9] == 7'b0101011 || ID_INST16[15:9] == 7'b0101110 || ID_INST16[15:11] == 5'b01110 || ID_INST16[15:11] == 5'b01111) DSIZE_CTRL = 2'b00;
    else if(ID_INST16[15:9] == 7'b0101001 || ID_INST16[15:9] == 7'b0101101 || ID_INST16[15:9] == 7'b0101111 || ID_INST16[15:12] == 4'b1000) DSIZE_CTRL = 2'b01;
    else if(ID_INST16[15:9] == 7'b0101000 || ID_INST16[15:9] == 7'b0101100 || ID_INST16[15:12] == 4'b0110 || ID_INST16[15:!2] == 4'b1001) DSIZE_CTRL = 2'b10;
    else DSIZE_CTRL = 2'b11;

    /* LDALUSel_CTRL */
    if(ID_INST16[15:9] == 7'b0101011) LDALUSel_CTRL = 3'b000; // LDRSB
    else if(ID_INST16[15:9] == 7'b0101111) LDALUSel_CTRL = 3'b001; // LDRSH
    else if(ID_INST16[15:9] == 7'b0101110 || ID_INST16[15:11] == 5'b01111) LDALUSel_CTRL = 3'b010; // LDRB
    else if(ID_INST16[15:9] == 7'b0101101 || ID_INST16[15:11] == 5'b10001) LDALUSel_CTRL = 3'b011; // LDRH
    else LDALUSel_CTRL = 3'b100; // o.w

    /* WBSel_CTRL */
    if(ID_INST16[15:11] == 5'b01001 || (ID_INST16[15:9] >= 7'b0101011 && ID_INST16[15:9] <= 7'b0101111) || ID_INST16[15:11] == 5'b01101 
    || ID_INST16[15:11] == 5'b01111 || ID_INST16[15:11] == 5'b10001 || ID_INST16[15:11] == 5'b10011) WBSel_CTRL = 1'b0;
    else WBSel_CTRL = 1'b1;

    /* RegWE_CTRL */
    /* CMP(imm), TST(reg), CMP(reg1), CMN(reg), CMP(reg2), BX, stores, NOP, B(cond), B(uncond) => 0*/
    if(ID_INST16[15:11] == 5'b00101 || ID_INST16[15:6] == 10'b0100001000 || ID_INST16[15:6] == 10'b0100001010 || ID_INST16[15:6] == 10'b0100001011 || 
    ID_INST16[15:8] == 8'b01000101 || ID_INST16[15:7] == 9'b010001110 || (ID_INST16[15:9] <= 7'b0101010 && ID_INST16[15:9] >= 7'b0101000) || 
    ID_INST16[15:11] == 5'b01100 || ID_INST16[15:11] == 5'b01110 || ID_INST16[15:11] == 5'b10000 || ID_INST16[15:11] == 5'b10010 || ID_INST16 == 16'hbf00 ||
    ID_INST16[15:12] == 4'b1101 || ID_INST16[15:11] == 5'b11100) RegWE_CTRL = 1'b0;
    else RegWE_CTRL = 1'b1;  

  end

  /* For control of PUSH, POP, STM, LDM */
  always @(negedge CLK_CTRL) begin
  
    /* PUSH control : If push is detected, empty EX, MEM, WB stages while fetching 
    proper register data from the register file. After that, it stores register values
    into the data cache.  */
    if(ID_INST16[15:9] == 7'b1011010) begin // 1. push detected
      if(count1 == 2'b11) begin // 3. CACHE is full. Store data into D$
        if(count2 == 0) begin // 3-1. Fetch stack pointer
          SP <= DO0_CTRL;
          count2 <= count2 + 1;
        end
        // 3-3. Data store is completed. Update stack pointer. Flush IF_ID pipeline registers. Reset DMODE_CTRL
        else if(count2 == regcnt + 1) begin
          REGMODE_CTRL <= 1'b1;
          WA1_MCTRL <= 4'b1101;
          DI1_MCTRL <= SP - 4*regcnt;
          WEN1_MCTRL <= 1'b1;
          IF_ID_FLUSH <= 1'b1;
          DMODE_CTRL <= 1'b0;
          count2 <= count2 + 1;
          NUM_INST_MCTRL <= 1'b1;
        end

        else if(count2 == regcnt + 2) begin // 3-4. IF_ID flush is completed. Reset the indicator, REGMODE_CTRL, counts
          IF_ID_FLUSH <= 1'b0;
          REGMODE_CTRL <= 1'b0;
          count1 <= 2'b0;
          count2 <= 4'b0;
          count3 <= 4'b0;
          NUM_INST_MCTRL <= 1'b0;
        end

        else begin // 3-2. 1 <= count2 <= regcnt. Store data into D$
          DMODE_CTRL <= 1'b1;
          DRW_MCTRL <= 1'b1; // write
          DSIZE_MCTRL <= 2'b10; // Ready completed to write into D$
          DADDR_MCTRL <= SP - 4*regcnt + 4*(count2 - 1);
          if(reglist[count3]) begin
            DOUT_MCTRL <= CACHE[count3];
            count2 <= count2 + 1;
          end
          count3 <= count3 + 1;
        end
      end

      else begin // 2. Store proper reg values in CACHE (for 3 cycles)
        CACHE[count1*3] <= DO0_CTRL;
        CACHE[count1*3+1] <= DO1_CTRL;
        CACHE[count1*3+2] <= DO2_CTRL; 
        count1 <= count1 + 1;
      end
    end

    /* POP control : */
    else if(ID_INST16[15:9] == 7'b1011110) begin // 1. pop detected
      if(count1 == 2'b11) begin // 3. Wait completed.
        if(count2 == 0) begin // 3-1. Set D$ environments
          DMODE_CTRL <= 1'b1;
          DRW_MCTRL <= 1'b0; // read
          DSIZE_MCTRL <= 2'b10;
          DADDR_MCTRL <= SP;
          count2 <= count2 + 1;
        end

        else if(count2 == regcnt + 1) begin // 3-3. CACHE storing is completed. Move data into regfile
          if(reglist[count3] && count3 < 8) begin // 3-3(1). Store data in CACHE into regfile
            REGMODE_CTRL <= 1'b1;
            WEN1_MCTRL <= 1'b1;
            WA1_MCTRL <= count3;
            DI1_MCTRL <= CACHE[count3];
            count3 <= count3 + 1;
          end

          else if(count3 == 8) begin // 3-3(3) Update stack pointer. Update PC if needed.
            REGMODE_CTRL <= 1'b1;
            WEN1_MCTRL <= 1'b1;
            WA1_MCTRL <= 13;
            DI1_MCTRL <= SP + 4*regcnt;
            if(reglist[count3]) begin
              POP_PC <= 1'b1;
              POPPED_PC_MCTRL <= CACHE[count3];
            end
            IF_ID_FLUSH <= 1'b1;
            count3 <= count3 + 1;
            NUM_INST_MCTRL <= 1'b1;
          end

          else if(count3 == 9) begin // 3-3(4) IF_ID flush is completed. Reset stuffs.
            POP_PC <= 1'b0;
            REGMODE_CTRL <= 1'b0;
            IF_ID_FLUSH <= 1'b0;
            count1 <= 2'b0;
            count2 <= 4'b0;
            count3 <= 4'b0;
            NUM_INST_MCTRL <= 1'b0;
          end

          else begin // 3-3(2). Wait one cycle if failed to find a valid register in the register list
            REGMODE_CTRL <= 1'b1;
            WEN1_MCTRL <= 1'b0;
            count3 <= count3 + 1;
          end
        end

        else begin // 3-2. 1 <= count2 <= regcnt. Fetch data in D$ into CACHE
          if(reglist[count3]) begin
            CACHE[count3] <= DIN_CTRL;
            count2 <= count2 + 1;
            DADDR_MCTRL <= SP + 4*count2;
          end
          if(reglist[count3] && count2 >= regcnt) begin
            count3 <= 0;
            DMODE_CTRL <= 0;
          end
          else count3 <= count3 + 1;
        end
      end

      else begin // 2. Wait until EX, MEM, WB are empty. Also, fetch SP from regfile.
        SP <= DO0_CTRL;
        count1 <= count1 + 1;
      end
    end

    /* STM : Very similar to PUSH.*/
    else if(ID_INST16[15:11] == 5'b11000) begin
      if(count1 == 2'b11) begin
        if(count2 == regcnt_stldm) begin
          REGMODE_CTRL <= 1'b1;
          WA1_MCTRL <= ID_INST16[10:8];
          DI1_MCTRL <= CACHE[8] + 4*regcnt_stldm;
          WEN1_MCTRL <= 1'b1;
          IF_ID_FLUSH <= 1'b1;
          DMODE_CTRL <= 1'b0;
          count2 <= count2 + 1;
          NUM_INST_MCTRL <= 1'b1;
        end

        else if(count2 == regcnt_stldm + 1) begin
          IF_ID_FLUSH <= 1'b0;
          REGMODE_CTRL <= 1'b0;
          count1 <= 2'b0;
          count2 <= 4'b0;
          count3 <= 4'b0;
          NUM_INST_MCTRL <= 1'b0;
        end

        else begin
          DMODE_CTRL <= 1'b1;
          DRW_MCTRL <= 1'b1;
          DSIZE_MCTRL <= 2'b10;
          DADDR_MCTRL <= CACHE[8] + 4*count2;
          if(reglist[count3]) begin
            DOUT_MCTRL <= CACHE[count3];
            count2 <= count2 + 1;
          end
          count3 <= count3 + 1;
        end
      end

      else begin
        CACHE[count1*3] <= DO0_CTRL;
        CACHE[count1*3+1] <= DO1_CTRL;
        CACHE[count1*3+2] <= DO2_CTRL; 
        count1 <= count1 + 1;
      end
    end

    /* LDM : Very similar to POP. */
    else if(ID_INST16[15:11] == 5'b11001) begin
      if(count1 == 2'b11) begin
        if(count2 == 0) begin // 3-1. Set D$ environments
          DMODE_CTRL <= 1'b1;
          DRW_MCTRL <= 1'b0; // read
          DSIZE_MCTRL <= 2'b10;
          DADDR_MCTRL <= CACHE[8];
          count2 <= count2 + 1;
        end

        else if(count2 == regcnt_stldm + 1) begin // 3-3. CACHE storing is completed. Move data into regfile
          if(reglist[count3] && count3 < 8) begin // 3-3(1). Store data in CACHE into regfile
            REGMODE_CTRL <= 1'b1;
            WEN1_MCTRL <= 1'b1;
            WA1_MCTRL <= count3;
            DI1_MCTRL <= CACHE[count3];
            count3 <= count3 + 1;
          end

          else if(count3 == 8) begin // 3-3(3) Update stack pointer. Update PC if needed.
            REGMODE_CTRL <= 1'b1;
            WEN1_MCTRL <= (ID_INST16[10:8]) ? 1'b1 : 1'b0;
            WA1_MCTRL <= ID_INST16[10:8];
            DI1_MCTRL <= CACHE[8] + 4*regcnt_stldm;
            IF_ID_FLUSH <= 1'b1;
            count3 <= count3 + 1;
            NUM_INST_MCTRL <= 1'b1;
          end

          else if(count3 == 9) begin // 3-3(4) IF_ID flush is completed. Reset stuffs.
            REGMODE_CTRL <= 1'b0;
            IF_ID_FLUSH <= 1'b0;
            count1 <= 2'b0;
            count2 <= 4'b0;
            count3 <= 4'b0;
            NUM_INST_MCTRL <= 1'b0;
          end

          else begin // 3-3(2). Wait one cycle if failed to find a valid register in the register list
            REGMODE_CTRL <= 1'b1;
            WEN1_MCTRL <= 1'b0;
            count3 <= count3 + 1;
          end
        end

        else begin // 3-2. 1 <= count2 <= regcnt. Fetch data in D$ into CACHE
          if(reglist[count3]) begin
            CACHE[count3] <= DIN_CTRL;
            count2 <= count2 + 1;
            DADDR_MCTRL <= CACHE[8] + 4*count2;
          end
          if(reglist[count3] && count2 >= regcnt_stldm) begin
            count3 <= 0;
            DMODE_CTRL <= 1'b0;
          end
          else count3 <= count3 + 1;
        end
      end

      else begin
        CACHE[8] <= DO0_CTRL;
        count1 <= count1 + 1;
      end
    end

    else begin // Maintain conditional valid outputs to zero
      DMODE_CTRL <= 1'b0;
      REGMODE_CTRL <= 1'b0;
      POPPED_PC_MCTRL <= 1'b0;
      NUM_INST_MCTRL <= 1'b0;
    end

  end

endmodule
