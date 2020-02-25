// Testbench for the control module of the LEGLite
//  The control is a combinational circuit
//
//  This testbench will input different
//  instructions into the Controller.  Note
//  the only input to the Control Circuit
//  is the opcode field instr[15:13]
//
//  The output of the Control Circuit are the
//  control signals to the datapath.
//  The testbench will display these signals:
//
// 		Instr[opcode]  opcode = opcode value
//		PC[branch]     branch bit
//		ALU[alusrc,aluselect]   Signals into ALU
//		Reg[regdst,regwrite]    Signals into Reg file
//
module testbenchControl;

wire reg2loc;
wire regwrite;
wire uncondbranch;
wire branch;
wire memread;
wire memwrite;
wire alusrc;
wire [2:0] alu_select;
reg [15:0] instr;  

// Instantiation of control circuit

Control Control_Circuit(
		reg2loc,
		uncondbranch,
		branch,
		memread,
		memtoreg,
		alu_select,
		memwrite,
		alusrc,
		regwrite,
		instr[15:12]
		);

//
// We'll input different instructions and check the
// output of the controller
//
// For example, if the instruction is "add" then
// the output should be
// reg2loc = 0
// uncondbranch = 0
// branch = 0
// memread = 0
// memtoreg = 0
// alu_select = 0
// memwrite = 0
// alusrc = 0
// regwrite = 1
//
initial
	begin
	$display("Instr[opcode] br[uncondbranch,branch] ALU[alusrc,aluselect]");
	$display("        Reg[reg2loc,regwrite] Mem[memread,memwrite,memtoreg]");
	#6
	$display("ADDI  X2,XZR,#3 :");
	instr={4'd8, 6'd3, 3'd7, 3'd2};
	#4
	$display("ADD   X4,XZR,XZR :");
	instr={4'd0, 3'd7,3'd0,3'd7,3'd4};
	#4
	$display("CBZ   X2,L0 :");
	instr={4'd7, 6'b111110,3'd2,3'd0};
	#4
	$display("ADDI  X4,X4,#5 :");
	instr={4'd8, 6'd5, 3'd4,3'd4};
	#4
	$display("ADDI  X2,X2,#-1 :");
	instr={4'd8, 6'b111111,3'd2,3'd2};
	#4
	$display("CBZ   XZR,L1 :");
	instr={4'd7, 6'b111011,3'd7,3'd0};
	#4
	$finish;
	end

initial
	$monitor("Instr[%b],PC[%b,%b] ALU[%b,%b] Reg[%b,%b] Mem[%b,%b,%b]",
		instr[15:12],
           uncondbranch,
		branch,
		alusrc,
		alu_select,
		reg2loc,
		regwrite,
		memread,
		memwrite,
		memtoreg
		);
		
endmodule