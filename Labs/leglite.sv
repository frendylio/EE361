// Your Name: Spencer & KEENAN
// Date: 6 December 2019

typedef enum logic [3:0] {
  ADD = 0,
  SUB = 1,
  AND = 2,
  OR = 3,
  ADDI = 4,
  SUBI = 5,
  ANDI = 6,
  ORI = 7,
  ST = 8,
  LD = 9,
  CBZ = 10,
  CBNZ = 11,
  BU = 12, // Unconditional branch 'B'
  BL = 13,
  NOP = 14,
  BR = 15
} Opcode;

typedef struct packed { // Instruction format
Opcode op;
logic [11:0] fields;
} IFormat;

typedef enum logic [2:0] {
  aluADD = 0,
  aluSUB = 1,
  aluAND = 2,
  aluOR = 3,
  aluPASS = 4
} AluSel;

typedef enum logic { // Mux to the ALU
  aluSrcReg,
  aluSrcOffset
} AluSrcSel;

typedef enum logic [15:0] {  // Mux to Reg File--data
  AluData,
  MemData  
} RegWriteDataSel;

typedef enum logic {  // Mux to Reg file--index
  RegRType,  
  RegNotRType
} RegWriteIndexSel;

typedef struct packed {
  //-------------------------------Control I/O-----------------------------------------
  logic branchUncond;
  logic branchCond;
  AluSrcSel aluSrcSel;
  AluSel aluSel;
  logic memRead;
  logic memWrite;
  RegWriteDataSel regWriteDataSel;
  RegWriteIndexSel regWriteIndexSel;
  logic regWrite;
  logic linkRegister;
  logic branchLink;

  logic [15:0] aluInput2;

  logic [15:0] regWriteData;

  //-------------------------------------------------------------------------------------
  IFormat instr;
  logic [15:0] regReadData1;
  logic [15:0] regReadData2;
  logic [15:0] branchAddr;
  logic aluZero;
  logic [15:0] aluResult;
  logic [2:0] regWriteIndex;
  logic [15:0] memReadData;
  logic [15:0] pc;
} Stage;

//---------- Computer ----------
//   including data memory

module Computer(
  output logic [15:0] imAddress, // To instr memory
  input IFormat instr,  // From instr memory
  input logic reset,
  input clock
);
  
  logic [2:0] count;
	logic write;
  
Stage control;
Stage ifid;
Stage idex;
Stage exmem;
Stage memwb;

// Control signals from controller
/*logic branchUncond;
logic branchCond;
AluSrcSel aluSrcSel;
AluSel aluSel;
logic memRead;
logic memWrite;
RegWriteDataSel regWriteDataSel;
RegWriteIndexSel regWriteIndexSel;
logic regWrite;

logic linkRegister;
logic branchLink;

  // Other signals
logic [15:0] memReadData;
  
logic [15:0] aluResult;
logic aluZero;
logic [15:0] aluInput2;
  
logic [2:0] regReadIndex1;
logic [2:0] regReadIndex2;

logic [15:0] regWriteData 
// PC register
*/

logic [15:0] pc;

// PC update logic
  always_ff @(posedge clock)
    if (reset==1)
      begin
      	pc <= 0;
      end
  else if (count > 0)
      begin
    		pc <= pc;
      end
  	else if (exmem.branchCond == 1 && exmem.aluSel == 5)
      pc <= exmem.aluResult;
  	else if (exmem.branchCond == 1 && exmem.aluZero == 1)
      pc <= exmem.branchAddr;
  	else if (exmem.branchUncond == 1)
      pc <= exmem.branchAddr;

    else
      pc <= pc+2;

assign imAddress = pc;
  
Control contl(
  write,
  control.branchUncond,
  control.branchCond,
  control.aluSrcSel,
  control.aluSel,
  control.memRead,
  control.memWrite,
  control.regWriteDataSel,
  control.regWriteIndexSel,
  control.regWrite,
  control.linkRegister,
  control.branchLink,
  ifid.instr.op,
  count
);
  
  
//-----------------------------STAGE 1!!!!!!!!1-------------------------------



// inputs: pc, imAddress (instruction from instr. memory)
// outputs: ifid.pc -> idex.pc, ifid.instr -> idex.instr
//          ifid.instr.fields[2:0] -> readReg1, ifid.instr.fields[5:3] -> readReg2
//          readData1 -> idex.readData1, readData2 -> idex.readData2

  
IFID instructionFetch(
  ifid.pc,
  ifid.instr,
  clock,
  reset,
  pc,
  instr,
  write
);

  always_ff @ (posedge clock)
  begin
    if ((ifid.instr.op == CBZ || ifid.instr.op == BU || ifid.instr.op == BL || ifid.instr.op == BR) && count == 0) //If branch, insert NOPs
      begin
    		count <= 3;
        write <= 0;
      end
    else if((ifid.instr.fields[2:0] == idex.regWriteIndex || ifid.instr.fields[5:3] == idex.regWriteIndex) && idex.regWrite == 1 && count == 0) //If write is read, insert NOPs
      begin
        count <= 2;
        write <= 0;
      end
    else if((ifid.instr.fields[2:0] == exmem.regWriteIndex || ifid.instr.fields[5:3] == exmem.regWriteIndex) && exmem.regWrite == 1 && count == 0) //If write is read, insert NOPs
      begin
        count <= 1;
        write <= 0;
      end
    if(count > 0)
      count = count - 1;
    else
      write <= 1;
  end
  
RegFile rf1(
  ifid.regReadData1,
  ifid.regReadData2,
  clock,
  ifid.instr.fields[2:0],
  ifid.instr.fields[5:3],
  memwb.regWriteData,
  memwb.regWriteIndex,
  memwb.regWrite,
  control.branchLink,
  control.linkRegister,
  pc
);
  
//----------------------------STAGE 2!!!!!!!!!----------------------------------

IDEX instructionDecode(
  idex.branchUncond,
  idex.branchCond,
  idex.aluSrcSel,
  idex.aluSel,
  idex.memRead,
  idex.memWrite,
  idex.regWriteDataSel,
  idex.regWriteIndexSel,
  idex.regWrite,
  idex.branchLink,
  idex.linkRegister,

  //Inputs from Controller
  control.branchUncond,
  control.branchCond,
  control.aluSrcSel,
  control.aluSel,
  control.memRead,
  control.memWrite,
  control.regWriteDataSel,
  control.regWriteIndexSel,
  control.regWrite,
  control.branchLink,
  control.linkRegister,
  //------------------------
  idex.pc,
  idex.instr,
  idex.regReadData1,
  idex.regReadData2,
  clock,
  reset,
  ifid.pc,
  ifid.instr,
  ifid.regReadData1,
  ifid.regReadData2
);
  
//Mux for unconditional/conditional branch
always_comb //Unconditional Branch => signExt, right shift, and addition to PC counter
  begin
    if(idex.branchUncond == 1)
      idex.branchAddr <= idex.pc + ({{4{idex.instr.fields[11]}}, idex.instr.fields[11:0]} << 1);
    else
      idex.branchAddr <= idex.pc + ({{6{idex.instr.fields[11]}}, idex.instr.fields[11:3]} << 1);
  end

// Mux at the ALU
always_comb 
  case(idex.aluSrcSel)
    aluSrcReg:     idex.aluInput2 = idex.regReadData2;
    aluSrcOffset:  idex.aluInput2 = {{10{idex.instr.fields[11]}},idex.instr.fields[11:6]};
  endcase
  
Alu alu1(
  idex.aluResult,
  idex.aluZero,
  idex.regReadData1,
  idex.aluInput2,
  idex.aluSel
);

always_comb  // Mux for regWriteIndex
  begin
    if (idex.linkRegister == 1)
    	idex.regWriteIndex = 6;
  	else
      begin
    	case(idex.regWriteIndexSel)
      		RegNotRType: idex.regWriteIndex = idex.instr.fields[5:3];
      		RegRType: idex.regWriteIndex = idex.instr.fields[8:6];
    	endcase
      end
  end

//--------------------------STAGE 3!!!!!!!--------------------------------------

EXMEM execute(
  exmem.branchUncond,
  exmem.branchCond,
  exmem.memRead,
  exmem.memWrite,
  exmem.regWriteDataSel,
  exmem.regWrite,
  exmem.branchLink,
  exmem.linkRegister,
  exmem.regWriteIndex,
  
  idex.branchUncond,
  idex.branchCond,
  idex.memRead,
  idex.memWrite,
  idex.regWriteDataSel,
  idex.regWrite,
  idex.branchLink,
  idex.linkRegister,
  idex.regWriteIndex,

  exmem.branchAddr,
  exmem.aluZero,
  exmem.aluResult,
  exmem.regReadData2,

  clock,
  reset,
  
  idex.pc,
  idex.instr,
  idex.branchAddr,
  idex.aluZero,
  idex.aluResult,
  idex.regReadData2
);

DataMemory dMem(
  exmem.memReadData,
  clock,
  exmem.memRead,
  exmem.memWrite,
  exmem.aluResult,
  exmem.regReadData2
);

//--------------------------STAGE 4!!!!!!!--------------------------------------

MEMWB writeback(
  // Control
  memwb.regWriteDataSel,
  memwb.regWriteIndexSel,
  memwb.regWrite,
  memwb.branchLink,
  memwb.linkRegister,

  exmem.regWriteDataSel,
  exmem.regWriteIndexSel,
  exmem.regWrite,
  exmem.branchLink,
  exmem.linkRegister,

  // Outputs
	memwb.memReadData,
  memwb.regReadData2,
  memwb.regWriteIndex,
  memwb.aluResult,

  // Inputs
  clock,
  reset,
  
  exmem.memReadData,
  exmem.aluResult,
  exmem.regWriteIndex,
  exmem.aluResult
);

always_comb // Mux for regWriteData
  begin
  	case(memwb.regWriteDataSel)
    	AluData: memwb.regWriteData = memwb.aluResult;
    	MemData: memwb.regWriteData = memwb.memReadData;
  	endcase
  end

endmodule

//---------- Control ----------

module Control(
  input logic count,
  output logic branchUncond,
  output logic branchCond,
  output AluSrcSel aluSrcSel,
  output AluSel aluSel,
  output logic memRead,
  output logic memWrite,
  output RegWriteDataSel regWriteDataSel,
  output RegWriteIndexSel regWriteIndexSel,
  output logic regWrite,
  output logic branchLink, // Select if WriteData is sourced from program counter or alu/memory
  output logic linkRegister, // Select if WriteReg is selected from regWriteData or link reg
  input Opcode opcode
);
  
always_comb
  begin
    if(count > 0)
      begin
        branchUncond = 0;
         branchCond = 0;
         aluSrcSel = aluSrcOffset;
         aluSel = aluPASS;
         memRead = 0;
         memWrite = 0;
         regWriteDataSel = AluData;
         regWriteIndexSel = RegNotRType;
         regWrite = 0;
         linkRegister = 0;
         branchLink = 0;
      end
      else
          begin
          unique case(opcode)
             ADD: 
               begin
                 branchUncond = 0;
                 branchCond = 0;
                 aluSrcSel = aluSrcReg;
                 aluSel = aluADD;
                 memRead = 0;
                 memWrite = 0;
                 regWriteDataSel = AluData;
                 regWriteIndexSel = RegRType;
                 regWrite = 1;
                 linkRegister = 0;
                 branchLink = 0;
               end
             ADDI:
               begin
                 branchUncond = 0;
                 branchCond = 0;
                 aluSrcSel = aluSrcOffset;
                 aluSel = aluADD;
                 memRead = 0;
                 memWrite = 0;
                 regWriteDataSel = AluData;
                 regWriteIndexSel = RegNotRType;
                 regWrite = 1;
                 linkRegister = 0;
                 branchLink = 0;
               end
             BU:
               begin
                 branchUncond = 1;
                 branchCond = 0;
                 aluSrcSel = aluSrcOffset;
                 aluSel = aluADD;
                 memRead = 0;
                 memWrite = 0;
                 regWriteDataSel = AluData;
                 regWriteIndexSel = RegNotRType;
                 regWrite = 0;
                 linkRegister = 0;
                 branchLink = 0;
               end
             SUBI:
               begin
                 branchUncond = 0;
                 branchCond = 0;
                 aluSrcSel = aluSrcOffset;
                 aluSel = aluSUB;
                 memRead = 0;
                 memWrite = 0;
                 regWriteDataSel = AluData;
                 regWriteIndexSel = RegNotRType;
                 regWrite = 1;
                 linkRegister = 0;
                 branchLink = 0;
               end
             CBZ:
               begin
                 branchUncond = 0;
                 branchCond = 1;
                 aluSrcSel = aluSrcOffset;
                 aluSel = aluPASS;
                 memRead = 0;
                 memWrite = 0;
                 regWriteDataSel = AluData;
                 regWriteIndexSel = RegNotRType;
                 regWrite = 0;
                 linkRegister = 0;
                 branchLink = 0;
               end
            LD:
               begin
                 branchUncond = 0;
                 branchCond = 0;
                 aluSrcSel = aluSrcOffset;
                 aluSel = aluADD;
                 memRead = 1;
                 memWrite = 0;
                 regWriteDataSel = MemData;
                 regWriteIndexSel = RegNotRType;
                 regWrite = 1;
                 linkRegister = 0;
                 branchLink = 0;
               end
            ST:
               begin
                 branchUncond = 0;
                 branchCond = 0;
                 aluSrcSel = aluSrcOffset;
                 aluSel = aluADD;
                 memRead = 0;
                 memWrite = 1;
                 regWriteDataSel = AluData;
                 regWriteIndexSel = RegNotRType;
                 regWrite = 0;
                 linkRegister = 0;
                 branchLink = 0;
               end
            BL:
               begin
                 branchUncond = 1;
                 branchCond = 0;
                 aluSrcSel = aluSrcOffset;
                 aluSel = aluADD;
                 memRead = 0;
                 memWrite = 0;
                 regWriteDataSel = AluData;
                 regWriteIndexSel = RegNotRType;
                 regWrite = 1;
                 linkRegister = 1;
                 branchLink = 1;
               end
            BR:
               begin
                 branchUncond = 1;
                 branchCond = 0;
                 aluSrcSel = aluSrcOffset;
                 aluSel = aluPASS;
                 memRead = 0;
                 memWrite = 1;
                 regWriteDataSel = AluData;
                 regWriteIndexSel = RegNotRType;
                 regWrite = 0;
                 linkRegister = 0;
                 branchLink = 0;
               end
            NOP:
              begin
                 branchUncond = 0;
                 branchCond = 0;
                 aluSrcSel = aluSrcOffset;
                 aluSel = aluPASS;
                 memRead = 0;
                 memWrite = 0;
                 regWriteDataSel = AluData;
                 regWriteIndexSel = RegNotRType;
                 regWrite = 0;
                 linkRegister = 0;
                 branchLink = 0;
              end
           endcase
          end
  end
endmodule

//---------- Register File ----------

module RegFile(
  output logic [15:0] readData1,
  output logic [15:0] readData2,
  input logic clock,
  input logic [2:0] readReg1,
  input logic [2:0] readReg2,
  input logic [15:0] writeData,
  input logic [2:0] writeIndex,
  input logic write,
  input logic branchLink,
  input logic linkRegister,
  input logic [15:0] pc
);
  
logic [15:0] regState [7:0];

// WriteData
always_ff @(negedge clock)
   if (write == 1)
     if (branchLink == 1)
       begin
       regState[6] <= pc + 2;
       end
     else
     	regState[writeIndex] <= writeData;

always_comb
   if (readReg1==7)
      readData1 = 0;
   else
      readData1 = regState[readReg1];

always_comb
   if (readReg2==7)
      readData2 = 0;
   else
      readData2 = regState[readReg2];
  
endmodule

//---------- Data Memory ----------

module DataMemory(
  output logic [15:0] readData,
  input logic clock,
  input logic read,
  input logic write,
  input logic [15:0] addr,
  input logic [15:0] writeData
);

logic [15:0] mem [127:0];
  
always_ff @(posedge clock)
  if (write == 1)
    mem[addr[7:1]] <= writeData;
  
endmodule

//---------- ALU ----------

module Alu(
  output logic [15:0] aluResult,
  output logic aluZero,
  input logic [15:0] aluInput1,
  input logic [15:0] aluInput2,
  input AluSel aluSel
);
  
  always_comb
    unique case(aluSel)
      aluADD: aluResult = aluInput1+aluInput2;
      aluSUB: aluResult = aluInput1-aluInput2;
      aluAND: aluResult = aluInput1&aluInput2;
      aluOR:  aluResult = aluInput1|aluInput2;
      aluPASS: aluResult = aluInput1;    
      default: aluResult = 0;
    endcase
  
 assign aluZero = ~|aluResult;

endmodule

//-----------------------------Stage Registers---------------------------------------------

module IFID (
  output logic [15:0] pc_out,
  output logic [15:0] instr_out,
  input clock,
  input reset,
  input logic [15:0] pc,
  input logic [15:0] instr,
  input logic write
);
  always_ff@(posedge clock)
    begin
      if (reset == 1)
        begin
          pc_out <= 0;
          instr_out <= 0;
        end
      else
        begin
          if (write == 1) // for stalling CBZs
            begin
          		pc_out <= pc;
          		instr_out <= instr;
            end
          // else if(write == 0)
          //   instr_out <= 16'b1110000000000000;
        end
    end
endmodule

module IDEX (
  //-------------------------------Control I/O-----------------------------------------
  output logic branchUncond,
  output logic branchCond,
  output AluSrcSel aluSrcSel,
  output AluSel aluSel,
  output logic memRead,
  output logic memWrite,
  output RegWriteDataSel regWriteDataSel,
  output RegWriteIndexSel regWriteIndexSel,
  output logic regWrite,
  output logic branchLink, // Select if WriteData is sourced from program counter or alu/memory
  output logic linkRegister, // Select if WriteReg is selected from regWriteData or link reg

  input logic branchUncond_in,
  input logic branchCond_in,
  input AluSrcSel aluSrcSel_in,
  input AluSel aluSel_in,
  input logic memRead_in,
  input logic memWrite_in,
  input RegWriteDataSel regWriteDataSel_in,
  input RegWriteIndexSel regWriteIndexSel_in,
  input logic regWrite_in,
  input logic branchLink_in, // Select if WriteData is sourced from program counter or alu/memory
  input logic linkRegister_in, // Select if WriteReg is selected from regWriteData or link reg
  //----------------------------------------------------------------------------------------------
  
  output logic [15:0] pc_out,
  output logic [15:0] instr_out,
  output logic [15:0] regReadData1_out,
  output logic [15:0] regReadData2_out,
  input clock,
  input reset,
  input logic [15:0] pc,
  input logic [15:0] instr,
  input logic [15:0] regReadData1,
  input logic [15:0] regReadData2
);
  always_ff@(posedge clock)
    begin
      if (reset == 1)
        begin
          branchUncond <= 0;
          branchCond <= 0;
          aluSrcSel <= aluSrcReg;
          aluSel <= aluPASS;
          memRead <= 0;
          memWrite <= 0;
          regWriteDataSel <= AluData;
          regWriteIndexSel <= RegRType;
          regWrite <= 0;
          branchLink <= 0;
          linkRegister <= 0;

          pc_out <= 0;
          instr_out <= 0;
          regReadData1_out <= 0;
          regReadData2_out <= 0;
          
        end
      else
        begin
          pc_out <= pc;
          instr_out <= instr;
          regReadData1_out <= regReadData1;
          regReadData2_out <= regReadData2;

          //Controller logic
          branchUncond <= branchUncond_in;
          branchCond <= branchCond_in;
          aluSrcSel <= aluSrcSel_in;
          aluSel <= aluSel_in;
          memRead <= memRead_in;
          memWrite <= memWrite_in;
          regWriteDataSel <= regWriteDataSel_in;
          regWriteIndexSel <= regWriteIndexSel_in;
          regWrite <= regWrite_in;
          branchLink <= branchLink_in;
          linkRegister <= linkRegister_in;
        end
    end
endmodule

module EXMEM (
  //-------------------------------Control I/O-----------------------------------------
  output logic branchUncond,
  output logic branchCond,
  output logic memRead,
  output logic memWrite,
  output RegWriteDataSel regWriteDataSel,
  output logic regWrite,
  output logic branchLink, // Select if WriteData is sourced from program counter or alu/memory
  output logic linkRegister, // Select if WriteReg is selected from regWriteData or link reg
  output logic [2:0] regWriteIndex,

  input logic branchUncond_in,
  input logic branchCond_in,
  input logic memRead_in,
  input logic memWrite_in,
  input RegWriteDataSel regWriteDataSel_in,
  input logic regWrite_in,
  input logic branchLink_in, // Select if WriteData is sourced from program counter or alu/memory
  input logic linkRegister_in, // Select if WriteReg is selected from regWriteData or link reg
  input logic [2:0] regWriteIndex_in,
  
  //----------------------------------------------------------------------------------------------
  output logic [15:0] branchAddr_out,
  output logic aluZero_out,
  output logic [15:0] aluResult_out,
  output logic [15:0] regReadData2_out,
  
  input clock,
  input reset,
  
  input logic [15:0] pc,
  input logic [15:0] instr,
  input logic [15:0] branchAddr,
  input logic aluZero,
  input logic [15:0] aluResult,
  input logic [15:0] regReadData2
);
  always_ff@(posedge clock)
    begin
      if (reset == 1)
        begin
          //control
          branchUncond <= 0;
          branchCond <= 0;
          memRead <= 0;
          memWrite <= 0;
          regWriteDataSel <= AluData;
          regWrite <= 0;
          branchLink <= 0;
          linkRegister <= 0;
          regWriteIndex <= 0;

          //other
          branchAddr_out <= 0;
          aluZero_out <= 0;
          aluResult_out <= 0;
          regReadData2_out <= 0;
        end
      else
        begin
          branchAddr_out <= branchAddr;
          aluZero_out <= aluZero;
          aluResult_out <= aluResult;
          regReadData2_out <= regReadData2;
          regWriteIndex <= regWriteIndex_in;

          //controller logic
          branchUncond <= branchUncond_in;
          branchCond <= branchCond_in;
          memRead <= memRead_in;
          memWrite <= memWrite_in;
          regWriteDataSel <= regWriteDataSel_in;
          regWrite <= regWrite_in;
          branchLink <= branchLink_in;
          linkRegister <= linkRegister_in;
        end
    end
endmodule

module MEMWB (
  //-------------------------------Control I/O-----------------------------------------
  output RegWriteDataSel regWriteDataSel,
  output RegWriteIndexSel regWriteIndexSel,
  output logic regWrite,
  output logic branchLink, // Select if WriteData is sourced from program counter or alu/memory
  output logic linkRegister, // Select if WriteReg is selected from regWriteData or link reg
  
  input RegWriteDataSel regWriteDataSel_in,
  input RegWriteIndexSel regWriteIndexSel_in,
  input logic regWrite_in,
  input logic branchLink_in, // Select if WriteData is sourced from program counter or alu/memory
  input logic linkRegister_in, // Select if WriteReg is selected from regWriteData or link reg
  
  //----------------------------------------------------------------------------------------------
  
  output logic [15:0] memReadData_out,
  output logic [15:0] regReadData2_out,
  output logic [2:0] regWriteIndex_out,
  output logic [15:0] aluResult,
  
  input clock,
  input reset,
  
  input logic [15:0] memReadData,
  input logic [15:0] regReadData2,
  input logic [2:0] regWriteIndex,
  input logic [15:0] aluResult_in
);
  always_ff@(posedge clock)
    begin
      if (reset == 1)
        begin
          regWriteDataSel <= AluData;
          regWriteIndexSel <= RegRType;
          regWrite <= 0;
          branchLink <= 0;
          linkRegister <= 0;
          memReadData_out <= 0;
          regReadData2_out <= 0;
          regWriteIndex_out <= 0;
          aluResult <= 0;
        end

      else
        begin
          //controller logic
          regWriteDataSel <= regWriteDataSel_in;
          regWriteIndexSel <= regWriteIndexSel_in;
          regWriteIndex_out <= regWriteIndex;
          regWrite <= regWrite_in;
          branchLink <= branchLink_in;
          linkRegister <= linkRegister_in;
          aluResult <= aluResult_in;
        end
      
    end
endmodule
