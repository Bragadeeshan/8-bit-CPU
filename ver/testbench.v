`timescale 1ns/100ps
module testbenchcpu;

    reg CLK, RESET;
    wire [31:0] PC;
    wire [31:0] INSTRUCTION;

//intializing the memory registers
 reg[7:0] instr_mem[0:1023];
wire[7:0] address,readdata,writedata;
wire[31:0] memreaddata,memwritedata;
wire[5:0] memaddress,ADDRESS;
wire[127:0] READDATA;

// instruction_cache(clock,
// reset,
// pc,
// read,
//readdata,
// instruction,
// address,
// busywait);

// module instruction_memory(
//     clock,
//     read,
//     address,
//     readdata,
//     busywait
// );

//data_cache(clock,reset,readin,writein,addressin,writedatain,readdatacpu,busywaitcpu,readout,writeout,addressout,writedataout,readdatadc,busywaitdc);
  
//cpu(PC,INSTRUCTION,CLK,RESET,READDATA,BUSYWAIT,WRITE,READ,WRITEDATA,ADDRESS);
instruction_memory myins(CLK,READ,ADDRESS,READDATA,BUSYWAIT);
instruction_cache myincache(CLK,RESET,PC,READ,READDATA,INSTRUCTION,ADDRESS,BUSYWAIT);

data_memory mymem(CLK,RESET,memread,memwrite,memaddress,memwritedata,memreaddata,membusywait);

data_cache mycache(CLK,RESET,read,write,address,writedata,readdata,busywait,memread,memwrite,memaddress,memwritedata,memreaddata,membusywait);

cpu mycpu(PC, INSTRUCTION, CLK, RESET,readdata,busywait,write,read,writedata,address);

    initial   
    begin
                //assigning some values for the registers
                

        
        // {instr_mem[0],instr_mem[1],instr_mem[2],instr_mem[3]} = 32'b00000000000000010000000000000111; //load 1 7
        // {instr_mem[4],instr_mem[5],instr_mem[6],instr_mem[7]} = 32'b00000000000000000000000000000011; //load 0 3
        // {instr_mem[8],instr_mem[9],instr_mem[10],instr_mem[11]} = 32'b00000000000001100000000000000111; //load 5 7
        // {instr_mem[12],instr_mem[13],instr_mem[14],instr_mem[15]} = 32'b00001010000000100000000000000011;//swd 2 3
        // {instr_mem[16],instr_mem[17],instr_mem[18],instr_mem[19]} = 32'b00000001000001000000000100000000;//add 4 1 0
        // {instr_mem[20],instr_mem[21],instr_mem[22],instr_mem[23]} = 32'b00001001000001000000000010001011;//lwi 4 0x8C 
        // {instr_mem[24],instr_mem[25],instr_mem[26],instr_mem[27]} = 32'b00000101000001110000001000000010;
        // {instr_mem[28],instr_mem[29],instr_mem[30],instr_mem[31]} = 32'b00001010000000110000001000000010;//swd 3 2
        // {instr_mem[32],instr_mem[33],instr_mem[34],instr_mem[35]} =  32'b00000001000000100000000100000000;//add 2 1 0
        // {instr_mem[36],instr_mem[37],instr_mem[38],instr_mem[39]} = 32'b00001000000001000000000000000010;//lwd 4 2
        // {instr_mem[40],instr_mem[41],instr_mem[42],instr_mem[43]} = 32'b00000100000001010000000100000100;
        // {instr_mem[44],instr_mem[45],instr_mem[46],instr_mem[47]} = 32'b00000010000000100000000100000000;//sub 2 1 0    
        // {instr_mem[48],instr_mem[49],instr_mem[50],instr_mem[51]} = 32'b00001011000000100000000000011111;//swi 2 0x1F
        // {instr_mem[52],instr_mem[53],instr_mem[54],instr_mem[55]} = 32'b00001010000000100000001000000011;//swd 2 3
        // {instr_mem[56],instr_mem[57],instr_mem[58],instr_mem[59]} = 32'b00000000000000110000000000000110; //load 3 6
    
    
    
        // generate files needed to plot the waveform using GTKWave
        $dumpfile("cpu_wavedata.vcd");
		$dumpvars(0, testbenchcpu);
        
        CLK = 1'b0;
        RESET = 1'b0;

        #10;
        RESET=1'b1;
        #30
        RESET=1'b0;

       
        
        // finish simulation after some time
        #250
        $finish;
        
    end


    assign #2 INSTRUCTION = {instr_mem[PC],instr_mem[PC + 32'd1],instr_mem[PC + 32'd2],instr_mem[PC + 32'd3]};
    // clock signal generation
    always
        #4 CLK = ~CLK;
        

endmodule