// EE 361 Testbench 
// Testbench for data memory (128 words)
// and I/O ports
//     0xfff0:  output port 7 segment display
//     0xfffa:  input port two switches s1,s0

module testbench;

reg clock;

wire [15:0] dm_rdata; // Read data
wire [6:0] iodisplay; // Output port to 7 segment display
reg [15:0] dm_addr;  // Data memory address
reg [15:0] dm_wdata; // Data memory write data
reg dm_write;        // Write enable
reg dm_read;	     // Read enable
reg io_sw0;          // I/O input from switch 0
reg io_sw1;          // I/O input from switch 1

// Instantiation
DMemory_IO DMem_Circuit(
	dm_rdata,	// read data
	iodisplay,	// IO port connected to 7 segment display
	clock,		// clock
	dm_addr,	// address
	dm_wdata,	// write data
	dm_write,	// write enable
	dm_read,	// read enable
	io_sw0,		// IO port from sliding switch 0
	io_sw1		// IO port from sliding switch 1
	);


// Clock signal
initial clock = 0;
always #1 clock = ~clock;

initial
	begin
	dm_addr=6;
	dm_wdata=5;
	dm_write=0;
	dm_read=0;
	io_sw0=0;
	io_sw1=1;
	#4
	dm_write=1;  // Memory[6] = 5
	dm_read=1;   // Read contents of Memory[6]
	#2
	dm_write=0;  // Turn write off
	#2
	dm_read=0;   // Turn read off -- output = default
	#2
	dm_wdata=9;
	dm_write=1;  // Memory[6] = 9
	#2
	dm_read=1;   // Turn read on
	#2
	dm_write=0;  // Turn write off
	#4
	dm_wdata=8;  // Change data to 8
	dm_addr=22;  // Change address to 22
	#4
	dm_write=1;  // Memory[22] = 8
	#2
	dm_write=0;
	#4
	dm_addr=16'hfffa;
	dm_wdata=14;
	#4
	dm_write=1;  // Output port = 14 = 0001110
	#2
	dm_write=0;
	#4
	dm_addr=16'hfff0;  // Read input port
                        // Note that dm_read is still 1
	#4
	io_sw0=1;  // Reading the two switches
	io_sw1=0;
	#4
	$finish;
	end


initial
	begin
	$display("Adr=address Rd=Read data Wr=Write data we=write enable, re=read enable dsp=I/O display sw =I/O input switches cl=Clock");
	$monitor("Adr=%b Rd=%b Wr=%b we=%b re=%b dsp[%b] sw[%b,%b] clk=%b",
		dm_addr,
		dm_rdata,
		dm_wdata,
		dm_write,
		dm_read,
		iodisplay,
		io_sw1,
		io_sw0,
		clock
		);
	end

endmodule