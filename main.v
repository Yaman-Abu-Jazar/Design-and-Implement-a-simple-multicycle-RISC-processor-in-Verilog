module tb_computer();

  reg clk, reset;
  wire [15:0] data_out_instruction;
  wire MemRead, MemWrite;
  wire [3:0] state;	
  wire signed [15:0] ALUresult, address_inst, data_in, data_out, address_data; 
  wire [1:0] ALUop;

  always #5 clk = ~clk;

  initial begin
	  
	clk = 1;
    reset = 1;
	#1
	reset = 0;
	#10
	reset = 1;
	
    
    #250 $finish;
  end 
  
  initial begin
	  $monitor("Instruction: %0h, PC : %0h", data_out_instruction, address_inst);
  end

  computer dut (
  
    .clk(clk),
    .address_inst(address_inst),
    .data_out_instruction(data_out_instruction),
    .data_in(data_in),
    .data_out(data_out),
    .address_data(address_data),
    .MemRead(MemRead),
    .MemWrite(MemWrite),
	.reset(reset),
	.state(state),
	.ALUresult(ALUresult),
	.ALUop(ALUop)
  );
  
endmodule 




module tb_mem();

  wire [31:0] data_out_instruction;
  reg clk;
  reg [31:0] address;

  always #10ns clk = ~clk;

  initial begin
	  
	clk = 1;
    address = 32'b0;
	
	#10ns
	address = 32'b1; 
	
	$display ("ioerfjoierhf : %d\n", 1<<31);
	
    $monitor("instruction : 0x%x\n", data_out_instruction);
    #200ns $finish;
  end

  instructionMemory inst_mem(
	
		.data(data_out_instruction), 
		.clk(clk),
		.address(address)					  
	);
  
endmodule



module computer (
	
     input clk, reset,
     output signed [15:0] address_inst, data_out, address_data, data_in,
	 output [15:0] data_out_instruction, 
  	 output MemRead, MemWrite,
	 output [3:0] state,
	 output signed [15:0] ALUresult,
	 output [1:0] ALUop
  );

	
	instructionMemory inst_mem(
	
		.data(data_out_instruction), 
		.clk(clk),
		.address(address_inst)					  
	);
	
	dataMemory data_mem(
	
		.dataOut(data_out),
		.clk(clk),
		.address(address_data),
		.dataIn(data_in),
		.memRead(MemRead), 
		.memWrite(MemWrite)
	);
	
	
	CPU cpu(
	
        .data_out(data_out),
        .data_out_instruction(data_out_instruction), 
        .clk(clk),
        .data_in(data_in),
        .address_data(address_data),
        .address_instruction(address_inst),
        .MemWrite(MemWrite),
        .MemRead(MemRead),
		.reset(reset), 
		.state(state),
		.ALUresult(ALUresult),
		.ALUop(ALUop)
    );
	
endmodule


module datapath ( 
	
	input WrBack, RegSrc,
		  RegWrite,
		  IRwrite, PCwrite,
		   ExtOp , clk, reset,
		    RASrc, MemSelection,
		  
	input [15:0] data_out_instruction, data_out,				  
	input [1:0] PCsrc, ALUop, ALUsrcA, ALUsrcB,
	output [3:0] opcode, 
	output mode,
	output [15:0] address_data, data_in,
	output z, n, v,
	output signed [15:0] ALU_result, address_instruction
);

	wire [2:0] Rs1, Rs2, Rd;
    wire [15:0] pc_temp, newPC, Branch_address;

    assign pc_temp = address_instruction;
	assign newPC = PCtype + 16'h0002;

    wire [15:0] instruction, BUS1, BUS2, 
                jumpTargetAddress;

    wire signed [15:0] extended_immediate, MDR_out, PCtype, Register_result, ALU_result_buffer, B_operand, A_operand, ALU_operand1, ALU_operand2;
    wire [4:0] immediate;
	wire [7:0] SImmediate;

    wire [2:0] Rw, Rs1_regfile, Rs2_regfile;

    assign Branch_address = pc_temp + extended_immediate;
	
	assign address_data = ALU_result_buffer;

    assign opcode = instruction[15:12];
    // R-type
    assign Rd = (instruction[15:12] <= 4'b0010) ? instruction[11:9] : instruction[10:8];
    assign Rs1 = (instruction[15:12] <= 4'b0010 || instruction[15:12] == 4'b1111) ? instruction[8:6] : instruction[7:5];
    assign Rs2 = instruction[5:3];
    // I-type
    assign mode = instruction[11];
    assign immediate = instruction[4:0];
    // J-type
    assign jumpTargetAddress = {address_instruction[15:12], instruction[11:0]};
	// S-type
	assign SImmediate = instruction[8:1];
	wire signed [15:0] BusW;

    flop IR (.out(instruction), .clk(clk), .writeEn(IRwrite), .in(data_out_instruction), .reset(1'b1));
    flop A (.out(A_operand), .clk(clk), .writeEn(1'b1), .in(BUS1), .reset(1'b1));
    flop B (.out(B_operand), .clk(clk), .writeEn(1'b1), .in(BUS2), .reset(1'b1));
    flop pc (.out(address_instruction), .clk(clk), .writeEn(PCwrite), .in(PCtype), .reset(reset));
    flop ALUout (.in(ALU_result), .clk(clk), .writeEn(1'b1), .out(ALU_result_buffer), .reset(1'b1));
    flop DataOut (.out(MDR_out), .clk(clk), .writeEn(1'b1), .in(data_out), .reset(1'b1));
	flop RegRes (.in(Register_result), .clk(clk), .writeEn(1'b1), .out(BusW), .reset(1'b1));

    reg_file file (
        .clk(clk),
        .regWrite(RegWrite),
        .regDst(Rd),
        .regSrc1(Rs1_regfile),
        .regSrc2(Rs2_regfile),
        .bus_w(BusW),
        .out1(BUS1),
        .out2(BUS2)
    );

    ALU alu (
        .ALUop(ALUop),
        .a(ALU_operand1),
        .b(ALU_operand2),
        .zero(z),
        .overflow(v),
        .negative(n),
        .result(ALU_result)
    );
	
	Mux4x1 ALUsrcA_mux (.A(pc_temp), .B(A_operand), .C(16'h0000), .D(0), .selection(ALUsrcA), .out(ALU_operand1));
    Mux4x1 ALUsrcB_mux (.A(extended_immediate), .B(B_operand), .C(16'h0002), .D(0), .selection(ALUsrcB), .out(ALU_operand2));
    Mux4x1 pcMux (.A(ALU_result), .B(jumpTargetAddress), .C(Branch_address), .D(A_operand), .selection(PCsrc), .out(PCtype));
    Mux4x1 ALU_mux (.A(ALU_result_buffer), .B(MDR_out), .C(newPC), .D(16'h0000), .selection(WrBack), .out(Register_result));
    Mux2x1 #(3) RegFileSrc1_mux (.A(Rs1), .B(3'b111), .selection(RASrc), .out(Rs1_regfile));
	Mux2x1 #(3) RegFileSrc2_mux (.A(Rs2), .B(Rd), .selection(RegSrc), .out(Rs2_regfile));
	Mux2x1 data_in_mux (.A(B_operand), .B({{8{SImmediate[7]}}, SImmediate}), .selection(MemSelection), .out(data_in));
    Extender ext (.immediate(immediate), .ExtOp(ExtOp), .ExtImmediate(extended_immediate));
							  
endmodule




module datapath_tb;

    // Input signals
    reg WrBack, RegSrc, RegWrite, IRwrite, PCwrite, ExtOp, clk, reset, RASrc, MemSelection;
    reg [15:0] data_out_instruction, data_out;
    reg [1:0] PCsrc, ALUop, ALUsrcA, ALUsrcB;

    // Output signals
    wire [3:0] opcode;
    wire mode;
    wire [15:0] address_data, address_instruction, data_in;
    wire z, n, v;
    wire [15:0] ALU_result;

    // Instantiate the datapath module
    datapath uut (
        .WrBack(WrBack),
        .RegSrc(RegSrc),
        .RegWrite(RegWrite),
        .IRwrite(IRwrite),
        .PCwrite(PCwrite),
        .ExtOp(ExtOp),
		.MemSelection(MemSelection),
        .clk(clk),
        .reset(reset),
        .ALUsrcA(ALUsrcA),
		.ALUsrcB(ALUsrcB),
        .RASrc(RASrc),
        .data_out_instruction(data_out_instruction),
        .data_out(data_out),
        .PCsrc(PCsrc),
        .ALUop(ALUop),
        .opcode(opcode),
        .mode(mode),
        .address_data(address_data),
        .address_instruction(address_instruction),
        .data_in(data_in),
        .z(z),
        .n(n),
        .v(v),
        .ALU_result(ALU_result)
    );

    // Clock generation
    always #5 clk = ~clk;

    initial begin
        // Initialize signals
        clk = 0;
        reset = 0;
        WrBack = 2'b00;
        RegSrc = 0;
        RegWrite = 0;
        IRwrite = 0;
        PCwrite = 0;
        ExtOp = 0;
        ALUsrcA = 0;
		ALUsrcB = 0;
        RASrc = 0;
        PCsrc = 2'b00;
        ALUop = 2'b00;
        data_out_instruction = 16'h0000;
        data_out = 16'h0000;

        // Apply reset
        #1;
        reset = 1;
        #9;
        reset = 1;

        // Fetch instruction 1 (ADD R1, R2, R3)
        IRwrite = 1;
        PCwrite = 1;
        data_out_instruction = 16'b00011001_10000011; // Example instruction

        #10;
        IRwrite = 0;
        PCwrite = 0;
        // Decode
        #10;
        ALUsrcA = 2'b01;
		ALUsrcB = 2'b01;
        ExtOp = 0;
        RegSrc = 0;
        RegWrite = 1;
        ALUop = 2'b01;

        // Execute
        #10;
        // Assume ALU does its job here
        // Memory Access
        // Write Back
        WrBack = 2'b00;
        RegWrite = 1;

        #10;
        // Fetch instruction 2 (SUB R4, R5, R6)
        IRwrite = 1;
        PCwrite = 1;
        data_out_instruction = 16'b01000101_01100011; // Example instruction

        #10;
        IRwrite = 0;
        PCwrite = 0;
        // Decode
        #10;
        ALUsrcA = 2'b01;
		ALUsrcB = 2'b00;
        ExtOp = 0;
        RegSrc = 0;
        RegWrite = 1;
        ALUop = 2'b00;

        // Execute
        #10;
        // Assume ALU does its job here
        // Memory Access
        // Write Back
        WrBack = 2'b00;
        RegWrite = 1;

        #10;
        // Fetch instruction 3 (ADDI R1, R2, #5)
        IRwrite = 1;
        PCwrite = 1;
        data_out_instruction = 16'b00110001_00000101; // Example instruction

        #10;
        IRwrite = 0;
        PCwrite = 0;
        // Decode
        #10;
        ALUsrcA = 2'b01;
		ALUsrcB = 2'b00;
        ExtOp = 1;
        RegSrc = 0;
        RegWrite = 1;
        ALUop = 2'b01;

        // Execute
        #10;
        // Assume ALU does its job here
        // Memory Access
        // Write Back
        WrBack = 2'b00;
        RegWrite = 1;

        #10;
        // Fetch instruction 4 (ANDI R3, R4, #3)
        IRwrite = 1;
        PCwrite = 1;
        data_out_instruction = 16'b01000011_00000011; // Example instruction

        #10;
        IRwrite = 0;
        PCwrite = 0;
        // Decode
        #10;
        ALUsrcA = 2'b01;
		ALUsrcB = 2'b00;
        ExtOp = 0;
        RegSrc = 0;
        RegWrite = 1;
        ALUop = 2'b00;

        // Execute
        #10;
        // Assume ALU does its job here
        // Memory Access
        // Write Back
        WrBack = 2'b00;
        RegWrite = 1;

        // Continue similarly for other instructions...

        // End the simulation
        #500;
        $finish;
    end

    // Monitor output signals
    initial begin
        forever #30 $display("Time: %0d, Opcode: %b, Mode: %b, PC: %0h, Instruction: %0h, ALU_result: %0h, z: %b, n: %b, v: %b, A_operand: %0h, B_operand: %0h, instruction: %0h, ALU_Operand2: %0h, extended_immediate: %0h, BUS1: %0d, BUS2: %0d, Rs1_regfile: %0h, Rd: %0h, Rs2: %0h, Rs1: %0h, PCtype: %0h",
                 $time, opcode, mode, address_instruction, data_out_instruction, ALU_result, z, n, v, uut.A_operand, uut.B_operand, uut.instruction, uut.ALU_operand2, uut.extended_immediate, uut.BUS1, uut.BUS2, uut.Rs1_regfile, uut.Rd, uut.Rs2, uut.Rs1, uut.PCtype);
    end

endmodule




module CPU (
	
	input [15:0] data_out, data_out_instruction, 
	input clk, reset,
	output signed [15:0] data_in, address_data,
				address_instruction,
	
	
	output MemWrite, MemRead,
	output [3:0] state,
	output [15:0] ALUresult,
	output [1:0] ALUop
);

	wire [3:0] opcode;
	wire mode;
	
	wire   MemReg, RegSrc,
		   RegWrite, RegWrite2,
		   IRwrite, PCwriteUncond, RASrc,
		   sign_ext, PCwrite, MemSelection;
		   
	wire [1:0] WrBack, ALUsrcA, ALUsrcB,
	PCsrc , ALUctrl;
	
	wire z, n, v;

	datapath d1 ( 
	
	 	.WrBack(WrBack),
		 .RegSrc(RegSrc),
		.RegWrite(RegWrite),
		.IRwrite(IRwrite),
		.PCwrite(PCwrite),
		.ExtOp(sign_ext),
		.clk(clk),
		.RASrc(RASrc),
		.MemSelection(MemSelection),
		.data_out_instruction(data_out_instruction),
		.data_out(data_out),
					  
		.ALUsrcA(ALUsrcA),
		.ALUsrcB(ALUsrcB),
		.PCsrc(PCsrc),
		.ALUop(ALUop), .reset(reset),
	
		.opcode(opcode), 
		.mode(mode), 
		.address_data(address_data), 
		.address_instruction(address_instruction),
		.data_in(data_in),
		.z(z), .n(n), .v(v),
		.ALU_result(ALUresult)
	);
	
	
	mainController control(
		 .RegSrc(RegSrc),
		.RegWrite(RegWrite),
		.IRwrite(IRwrite),
		.PCwriteUncond(PCwriteUncond),
		.sign_ext(sign_ext),
		.MemWrite(MemWrite),
		.MemRead(MemRead),
		.MemSelection(MemSelection),
		   
		   
		.ALUsrcA(ALUsrcA),
		.ALUsrcB(ALUsrcB),
		.PCsrc(PCsrc),
		.ALUctrl(ALUctrl),  
	
		.clk(clk),
		.opcode(opcode),
		.mode(mode),
		.reset(reset),
		.state(state),
		.RASrc(RASrc),
		.WrBack(WrBack)
	);
	
	
	PCcontrol pc_control(

		.opcode(opcode), 
		.z(z), .v(v), .n(n),
		.pcWriteUncond(PCwriteUncond),
		.writeEn(PCwrite), 
		.state(state),
		.RASrc(RASrc)
	);
	
	
	ALUcontrol ALU_control(
	
		.ALUop(ALUop),
		.opcode(opcode),
		.ALUctrl(ALUctrl)
	); 	
	
endmodule


module CPU_tb;

    // Input signals
    reg [15:0] data_out, data_out_instruction;
    reg clk, reset;
    
    // Output signals
    wire [15:0] data_in, address_data, address_instruction;
    wire MemWrite, MemRead;
    wire [3:0] state;
    wire [15:0] ALUresult;
    wire [1:0] ALUop;

    // Instantiate the CPU module
    CPU uut (
        .data_out(data_out),
        .data_out_instruction(data_out_instruction),
        .clk(clk),
        .reset(reset),
        .data_in(data_in),
        .address_data(address_data),
        .address_instruction(address_instruction),
        .MemWrite(MemWrite),
        .MemRead(MemRead),
        .state(state),
        .ALUresult(ALUresult),
        .ALUop(ALUop)
    );

    // Clock generation
    always #5 clk = ~clk;

    // Test sequence
    initial begin
        // Initialize inputs
        clk = 0;
        reset = 0;
        data_out = 16'h0000;
        data_out_instruction = 16'h0000;

        // Apply reset
        #10;
        reset = 1;

        // Test scenario 1: Basic instruction fetch
        #20;
        data_out_instruction = 16'h1234; // Sample instruction

        // Test scenario 2: ALU operation - ADD
        #20;
        data_out = 16'h5678;
        data_out_instruction = 16'h9ABC; // Sample ALU instruction

        // Test scenario 3: Memory Read
        #20;
        data_out = 16'h1111;
        data_out_instruction = 16'h2222; // Sample memory read instruction

        // Test scenario 4: Memory Write
        #20;
        data_out = 16'h3333;
        data_out_instruction = 16'h4444; // Sample memory write instruction

        // Test scenario 5: Branching
        #20;
        data_out_instruction = 16'h5555; // Sample branch instruction

        // End the simulation
        #200;
        $finish;
    end

    // Monitor output signals
    initial begin
        $monitor("Time: %0d | State: %0d | PC: %0h | Instruction: %0h | ALUresult: %0h | MemWrite: %b | MemRead: %b", 
                 $time, state, address_instruction, data_out_instruction, ALUresult, MemWrite, MemRead);
    end

endmodule



// Extender module
module Extender #(parameter n = 5)
	(
	input [n-1:0] immediate,
	input ExtOp,
	output reg signed [15:0] ExtImmediate
	);
	
	always @(ExtOp or immediate) begin
        case (ExtOp)
            1'b0 :    ExtImmediate = {11'b00000000000, immediate};
            1'b1 :    ExtImmediate = {{11{immediate[n-1]}}, immediate};
        endcase
    end
		
endmodule


module mainController (	 
	
	output reg RegSrc,
		 	  RegWrite, MemRead,
		   	  IRwrite, PCwriteUncond,
		        sign_ext, MemWrite,
			   RASrc, MemSelection,
		   
		   
	output reg [1:0]  WrBack, ALUsrcB, ALUsrcA,
	PCsrc, ALUctrl,
	
	output reg [3:0] state,
	
	input clk, reset, 
	input [3:0] opcode,
	input mode
);		 						 
	reg jmp, branch;
	wire logical;
	
	parameter InstructionFetch = 0, 
		  	  InstructionDecode = 1, 
		  	  AddressComputation = 2, 
		  	  LoadAccess = 3, 
		  	  StoreAccess = 4,
			  ALUcomputationR = 5,
		  	  ResultStore = 6,
			  BranchCompletion = 7,
			  ALUcomputationI = 8,
			  ResultStoreMem = 9,
			  Call = 10,
			  Return = 11,
			  AddressForLB = 12;
				
	parameter AND = 4'b0000, ADD = 4'b0001, SUB = 4'b0010, ADDI = 4'b0011, ANDI = 4'b0100, LW = 4'b0101, LBu = 4'b0110, LBs = 4'b0110, SW = 4'b0111, BGT = 4'b1000, BGTZ = 4'b1000,
				BLT = 4'b1001, BLTZ = 4'b1001, BEQ = 4'b1010, BEQZ = 4'b1010, BNE = 4'b1011, BNEZ = 4'b1011, JMP = 4'b1100, CALL = 4'b1101,
				RET = 4'b1110, Sv = 4'b1111; 
			  

			 
			  
	always @* begin 
		
		case (opcode) 
			JMP, CALL, RET: begin
				
			   jmp = 1;
			   branch = 0;
			end
			
			BGT, BLT,
			BEQ, BNE
			 :  begin
				jmp = 0;
				branch=1;
			end
			
			default : begin 
				jmp = 0;
				branch = 0;
			end
		endcase
	end
	
	assign logical = (opcode == ANDI) ? 1 : 0; 
	
	
		
	always @(posedge clk or negedge reset) begin
		
		if (!reset)
			state <= InstructionFetch;
		
		else begin
			
		case (state)  
			
			InstructionFetch : state <= InstructionDecode;
			
			InstructionDecode :  begin
				
				case (opcode)
						
					AND, ADD, SUB: state <= ALUcomputationR;   //R-type			  
					LW, SW,  Sv : state <= AddressComputation; //memory access
					
					LBu, LBs : state <= AddressForLB;
					
					BEQ, BNE, BGT, BLT : state <= BranchCompletion; //Branch instruction
									
					JMP : state <= InstructionFetch; //jmp instruction	
					ANDI, ADDI: state <= ALUcomputationI;  // I-type
					CALL : state <= Call;
					RET : state <= Return;
				endcase
			 end

			AddressForLB: begin
				state <= LoadAccess;
			end
			
		  	AddressComputation : begin
				  
				case (opcode)
					LW : state <= LoadAccess; 
					SW, Sv : state <= StoreAccess;
				endcase
			end
					
			LoadAccess : begin
				
				 state <= ResultStoreMem;
				
			end

		  	StoreAccess, ResultStoreMem,
		 	ResultStore, 
		 	BranchCompletion, Return : state <= InstructionFetch;
			ALUcomputationI, ALUcomputationR, Call: state <= ResultStore;
			
		endcase	
		end
	
	end
	
	always @(*) begin
		RegSrc = 0;
		RegWrite = 0;
		MemRead = 0;
		MemWrite = 0;
		IRwrite = 0;
		PCwriteUncond = 0; 
		sign_ext = 0;
		PCsrc = 0;
		ALUctrl = 0;
		ALUsrcA = 0;
		ALUsrcB = 0;
		RASrc = 0;
		WrBack = 0;
		MemSelection = 0;
			
		case (state) 
			
			InstructionFetch : begin
				IRwrite = 2'b1;
  				PCsrc = 0;
 				PCwriteUncond = 1;
 				ALUsrcA=2'b00;
 				ALUsrcB=2'b10;
  				ALUctrl=2'b01;
			end
			
		  	InstructionDecode : begin
				sign_ext = 1;
				PCsrc = 2'b00;
				PCwriteUncond = jmp;
			end

		  	AddressComputation : begin 
				sign_ext = 1;
				ALUsrcA = 2'b01;
				ALUsrcB = 2'b00;
				ALUctrl = 2'b01;
				RegSrc = 1;
			end
			 
			LoadAccess : MemRead = 2'b1;
	 		StoreAccess : begin
			 	MemWrite = 2'b1;
				 case(opcode)
					SW: MemSelection = 0;
					Sv: MemSelection = 1;
				 endcase
			 end
			 
			ALUcomputationR :	begin
				 
				ALUsrcA = 2'b01;
				ALUsrcB = 2'b01;
				ALUctrl = 2'b00;
			end
				  
			ResultStore : RegWrite = 2'b1;
			
			AddressForLB: begin
				case(mode)
					1'b0: begin
						sign_ext = 0;
						ALUsrcA = 2'b01;
						ALUsrcB = 2'b00;
						ALUctrl = 2'b01;
					end
					
					1'b1:begin
						sign_ext = 1;
						ALUsrcA = 2'b01;
						ALUsrcB = 2'b00;
						ALUctrl = 2'b01;
					end
				endcase
			end
				
			ResultStoreMem : begin
				
				WrBack = 2'b01;
				RegWrite = 1;
			end
				  
			BranchCompletion : begin
				RegSrc = 1;
				PCsrc = 2'b10;
				ALUsrcA = 2'b01;
				ALUsrcB = 2'b01;
				ALUctrl= 2'b11;
			end
			  
			ALUcomputationI : begin
				  
				ALUsrcA = 2'b01;
				ALUsrcB = 2'b00;
				ALUctrl = 2'b00;
				sign_ext = !logical;
			end
			
			Call : begin
				PCsrc = 2'b01;
				ALUctrl = 2'b01;
				PCwriteUncond = 1;
			end
			
			Return : begin
				RASrc = 1;
				PCsrc = 2'b11;
				PCwriteUncond = 1;
				
			end
		
		endcase
		
	end

endmodule




module ALUcontrol (
	
	input [1:0] ALUctrl,
	input [3:0] opcode,
	output reg [1:0] ALUop
); 

parameter  	   
			  AND = 0'b00,
			  ADD = 2'b01,
			  SUB = 2'b10,
			  RSB = 2'b11;
			  

	always @* begin 
		case (ALUctrl)
		
			RSB, ADD, SUB : ALUop = ALUctrl; //the operation is generated by the main control
			2'b00 : begin
				case (opcode) //determine the operation based on the opcode 
					4'b0001, 4'b0011 : ALUop = ADD; //opcode = ADDI, ADD (R-TYPE)
					4'b0010 : ALUop = SUB; //opcode = SUB R-TYPE
					4'b0000, 4'b0100 : ALUop = AND; //opcode = AND, ANDI
				endcase
			end
		endcase
	end
	
endmodule


module PCcontrol (

	input [3:0] opcode, 
	input z, v, n,
	input pcWriteUncond,
	input RASrc,
	input [3:0] state,
	output writeEn
);
 
	parameter BNE = 4'b1011,
			  BNEZ = 4'b1011,
			  BEQ = 4'b1010,
			  BEQZ = 4'b1010,
			  BLT = 4'b1001,
			  BLTZ = 4'b1001,
			  BGT = 4'b1000,
			  BGTZ = 4'b1000;	 
			  
	wire branch;
	
	assign branch = ((opcode == BNE) && !z) || 
					((opcode == BEQ) && z) || 
					((opcode ==BGT) && (!z && !(n ^ v))) || 
					((opcode == BLT) && (n ^ v)); 

					
	assign writeEn = ((state == 7) && branch) || pcWriteUncond || RASrc;

endmodule 





module ALU (
	
	input [1:0] ALUop, 
	input signed [15:0] a, [15:0] b,
	output zero, overflow, negative,
	output reg signed [15:0] result 
);

	reg [1:0] carry;

	assign zero = (result == 0);
	assign negative = result[15];
	assign overflow = carry[1] ^ carry[0];
	
	
	always @(*) begin
		
		case (ALUop)
			
			2'b00 : result = a & b;
			2'b01 : begin 
					{carry[0], result[14:0]} = a[14:0] + b[14:0];
					{carry[1], result[15]} = a[15] + b[15] + carry[0];	
				end	
				
			2'b10 : result = a - b;
			2'b11 : result = b - a;
		endcase
	end
endmodule



module reg_file (
	input clk,
	input regWrite,
	input [2:0] regDst, regSrc1,
	input [2:0] regSrc2,
	input [15:0] bus_w,
	output reg [15:0] out1, out2
	);	
	reg [15:0] regArray [0:7] = {0,		// R0
								4198,	// R1
								5596,	// R2
								14426,	// R3
								7612,	// R4
								6638,	// R5
								10040,	// R6
								155};	// R7
	

	always @(*) begin //The output is taken asynchronously										  
			out1 = regArray[regSrc1];
			out2 = regArray[regSrc2];	
	end
	always @(posedge clk) begin	
		
		if (regWrite && regDst !== 3'b000)
			regArray[regDst] <= bus_w;
	end
	
endmodule



module instructionMemory #(
    parameter WIDTH = 16
) (
    output reg [15:0] data,
    input clk,
    input [WIDTH-1:0] address
);

    reg [7:0] memory [0:65535];

    initial begin
		
		integer i;
		for(i = 0; i < 65536 ; i = i + 1) begin
			memory[i] = 0;
		end
       // Initialize memory with specific instructions
    // S-type: Sv Mem[Reg(R2)] <-- 10
    memory[0] = 8'b00010100; // (lower byte)
    memory[1] = 8'b11110000; // (upper byte)

    // any instruction
    memory[12] = 8'b00000000; // (lower byte)
    memory[13] = 8'b10010000; // (upper byte)
	
	// any Instruction 
    memory[20] = 8'b00000110; // (lower byte)
    memory[21] = 8'b00111000; // (upper byte)

    end

    assign data = {memory[address + 1], memory[address]};

endmodule
	



module dataMemory #(parameter WIDTH = 256)(
	output reg [15:0] dataOut,
	input clk,
	input [15:0] address,
	input [15:0] dataIn,
	input memRead, memWrite
);
reg [7:0] mem [0:65535];
initial begin 
	
	integer i;
	for(i = 0; i < 65536; i = i + 1) begin
		mem[i] = 0;	
	end
	mem[10] = 5;
	mem[11] = 7;
	mem[700] = 8;
	mem[701] = 44;
	mem[121] = 32;
	mem[122] = -122;
	mem[400] = -200;
	mem[401] = -2;
	mem[402] = 211;
	mem[403] = 12;
	mem[4201] = 10;
	mem[4202] = 11;
	mem[16'hFFFA] = 10;
	mem[16'hFFFA + 1] = 11;
	mem[16'h001A] = 3;
	mem[16'h001A + 1] = 99;
end
	always @(posedge clk) begin
		if (memWrite) 
			{mem[address + 1], mem[address]} <= dataIn;
	end
	always @* begin
		if (memRead)
			dataOut = {mem[address + 1], mem[address]};
	end	
endmodule




	
module flop (  
	
	output reg [15:0] out,
	input clk,
	input writeEn,
	input [15:0] in,
	input reset
);
	always @(posedge clk or negedge reset) begin
	
		if (!reset)
			out <= 0;
		
		else if (writeEn)
			out <= in; 
	end
	
endmodule
 



// Mux 2 to 1 module
module Mux2x1 #(parameter n = 16)
	(input [n-1:0] A, B,
	 input selection,
	 output reg [n-1:0] out
	);
	
	always @(*) begin
		case (selection)
			1'b0 : out = A;
			1'b1 : out = B;
		endcase
	end
	
endmodule


// Mux 4 to 1 module
module Mux4x1 #(parameter n = 16)
	(input [n-1:0] A, B, C, D,
	 input [1:0] selection,
	 output reg [n-1:0] out
	);
	
	always @(*) begin
		case (selection)
			2'b00 : out = A;
			2'b01 : out = B;
			2'b10 : out = C;
			2'b11 : out = D;
		endcase
	end
	
endmodule	
		

	  
