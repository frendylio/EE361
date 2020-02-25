// Testbench for LEGLite PC Logic

module testbenchPCLogic;

wire [15:0] pc;
reg clock;
reg [15:0] signext;
reg uncondbranch;
reg branch;
reg alu_zero;
reg reset;

// Instantiation

PCLogic PC_Circuit(
				// Outputs
		pc,	// Program counter (pc)
				// Inputs
		clock,
		signext,	//   sign extension from 7 bit constant
		uncondbranch,  // unconditional branch
		branch,	//   CBZ instruction
		alu_zero,	//   zero from ALU, used in cond branch
		reset		//   reset input
		);

// clock signal generation
initial clock=0;
always #1 clock = ~clock;

initial
	begin
	uncondbranch=0;
	branch=0; // Initialize the inputs to the PC control
	alu_zero=0;
	signext=6;  // Signext = 6 will be relevant later
	reset=1;  // Reset PC to 0
	#2
	reset=0;  // Disable reset so PC should start changing
	#6	     // Next 3 clock cycles, PC should increment
	branch=1; // branch instruction but won't branch
                // since alu_zero =0
	#2
	alu_zero=1;   //This will cause a branch forward by 12
	#2
	branch = 0;  // This will disable the branch
	#2
	alu_zero=0;  // This will also disable the branch 
	#2
	branch=1;  // Does not cause a branch because alu_zero=0
	#6		// PC will increment as usual
	signext = -3;  // This causes a backward branch by -6
	branch=1;
	alu_zero=1;  
	#2
	branch=0;    // PC increments
	alu_zero=0;
	#6
	
	$finish;
	end


initial
	$monitor("pc=%d signext=%d br=%b ucondbr=%b aluz=%b reset=%b clk=%b",
		pc,
		signext,
		branch,
		uncondbranch,
		alu_zero,
		reset,
		clock
		);

endmodule

