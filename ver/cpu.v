`timescale 1ns/100ps

//creating a multiplexer to use
module MUX_2x1 (in0,in1,sel,out);
//port declarations
input[7:0] in0,in1;
input sel;
output[7:0] out;

reg[7:0] out;

always @(in0,in1,sel)
begin 
    case(sel)
        1'b1 :
            out = in1;
        1'b0 :
            out = in0;
     endcase   
end
endmodule
module MUX32_2x1 (in0,in1,sel,out);
//port declarations
input[31:0] in0,in1;
input sel;
output[31:0] out;

reg[31:0] out;

always @(in0,in1,sel)
begin 
    case(sel)
        1'b1 :
            out = in1;
        1'b0 :
            out = in0;
     endcase   
end
endmodule

//creating an ALU to use
module alu(in1,in2,result,select,zero); 
    input[7:0] in1,in2; //8 bit operands
    input[2:0] select; //operation selection 
    output[7:0] result ;// 8 bit outputs
    output reg zero;
    reg result;
    always @(in1,in2,select)
    begin
        case(select)
            3'b000 : begin
                zero=0;
                #1 result=in2; // forwarding data
                end
            3'b001 :begin
                 zero=0;
                #2 result=in1+in2; // addition
                end
            3'b010 : begin
                zero=0;
                #1 result=in1 & in2; // and operation
                end
            3'b011 : begin
                zero=0;
                #1 result=in1 | in2; // or operation
                end
            3'b100 : begin
                #1 result=in1 ^~ in2;
                if(result==8'b11111111)begin
                    zero=1;
                end
                else begin
                    zero =0;
                end
            end
            

            default:begin

                result = 8'b00000000; // for future functional units 
                zero=0;
                end
        endcase  
    end
endmodule


//ceating a regfile to use
module reg_file(IN, OUT1, OUT2, INADDRESS, OUT1ADDRESS, OUT2ADDRESS, WRITE, CLK, RESET); 

input[7:0] IN;
input[2:0] INADDRESS,OUT1ADDRESS,OUT2ADDRESS;
input CLK,RESET,WRITE;
output[7:0] OUT1,OUT2;
reg[7:0] OUT1,OUT2;

reg [7:0] rfile[7:0];
integer i;
always @(RESET or posedge CLK ) begin
    if(RESET) 
    begin
        #2;
         for (i=0;i<=7;i=i+1)
             begin
             rfile[i]=0;
            end
    end

    else if(WRITE) begin
        #1 rfile[INADDRESS] = IN;
    end
             
end

always @(*) begin
    #2;
    OUT1 = rfile[OUT1ADDRESS];
    OUT2 = rfile[OUT2ADDRESS];
end


endmodule

module comp(out,in);
input [7:0] in;
output signed[7:0]out;
reg out;
always@(in)begin
#1 
out=-in;
end


endmodule


module data_memory(
	clock,
    reset,
    read,
    write,
    address,
    writedata,
    readdata,
	busywait
);
input				clock;
input           	reset;
input           	read;
input           	write;
input[5:0]      	address;
input[31:0]     	writedata;
output reg [31:0]	readdata;
output reg      	busywait;

//Declare memory array 256x8-bits 
reg [7:0] memory_array [255:0];

//Detecting an incoming memory access
reg readaccess, writeaccess;
always @(read, write)
begin
	busywait = (read || write)? 1 : 0;
	readaccess = (read && !write)? 1 : 0;
	writeaccess = (!read && write)? 1 : 0;
end

//Reading & writing
always @(posedge clock)
begin
	if(readaccess)
	begin
		readdata[7:0]   = #40 memory_array[{address,2'b00}];
		readdata[15:8]  = #40 memory_array[{address,2'b01}];
		readdata[23:16] = #40 memory_array[{address,2'b10}];
		readdata[31:24] = #40 memory_array[{address,2'b11}];
		busywait = 0;
		readaccess = 0;
	end
	if(writeaccess)
	begin
		memory_array[{address,2'b00}] = #40 writedata[7:0];
		memory_array[{address,2'b01}] = #40 writedata[15:8];
		memory_array[{address,2'b10}] = #40 writedata[23:16];
		memory_array[{address,2'b11}] = #40 writedata[31:24];
		busywait = 0;
		writeaccess = 0;
	end
end
integer i;
//Reset memory
always @(posedge reset)
begin
    if (reset)
    begin
        for (i=0;i<256; i=i+1)
            memory_array[i] = 0;
        
        busywait = 0;
		readaccess = 0;
		writeaccess = 0;
    end
end

endmodule

module instruction_cache(clock,
reset,
pc,
read,
readdata,
instruction,
address,
busywait);
input clock,reset;
input[127:0] readdata;
input[31:0] pc;
output reg[31:0] instruction;
output reg read;
output reg busywait;
output reg[5:0] address;

reg [31:0] cachemem[31:0];
reg [2:0] addressTag [7:0];

reg  vbit [7:0];
// reg [31:0] word;
reg [2:0] index;
reg [1:0] offset;
reg [24:0] tag;
//reg [127:0] block;
reg valid,hit;

always @ (*) begin

	 


		if(pc != -4) begin

			index = pc[9:7];
			tag = pc[6:4];
			offset = pc[3:2];

			valid = vbit[index]; 



			if((addressTag[index] == tag) && valid) begin // check the tag match and valid

				hit = 1; 
			end 
            else begin

				hit = 0; 
			end
		end
end





	always @(*) begin  

		if(hit) begin 
			
			instruction = cachemem[{index,offset}]; 

		end
	end




	integer i; 
	always @ (posedge reset) begin  

		if (reset) begin
			for(i=0; i<8; i++)begin

				addressTag[i] = 0;     
				vbit[i] = 0; 
				
			end
		end
	end


	parameter IDLE = 3'b000, MEM_READ = 3'b001, UPDATE_CACHE = 3'b010;
	reg [2:0] state, next_state;


    // combinational next state logic
    always @(*)
    begin
        case (state)
            IDLE: begin                     // IDLE state no memeory readings 
                if (!hit)  
                    next_state = MEM_READ;
                else
                	next_state = IDLE;
            end

            MEM_READ: begin                // When hit = 0 
            	if(!busywait)
            		next_state = UPDATE_CACHE;
            	else
            		next_state = MEM_READ;
            end

            UPDATE_CACHE: begin          // Updating the cache memory 
            		next_state = IDLE;
            end
        endcase
    end


    // combinational output logic
    always @(*)
    begin
        case(state)
            IDLE:
            begin
                read = 0;
                address = 6'd0;
                busywait = 0;
            end

            MEM_READ: 
            begin
                read = 1;
              
                address = {index, tag};
                busywait = 1;
            end

            UPDATE_CACHE: 
            begin
                read = 1;

                cachemem[{index,2'b00}] = readdata[31:0];
                cachemem[{index,2'b01}] = readdata[63:32];
                cachemem[{index,2'b10}] = readdata[95:64];
                cachemem[{index,2'b11}] = readdata[127:96];

                busywait = 1;
                vbit[index] = 1;
                addressTag[index] = tag;
                hit = 1;

            end
            
        endcase
    end


    //sequential logic for state transitioning 
    always @(posedge clock, reset)
    begin
        if(reset)
            state = IDLE;
        else
            state = next_state;
    end

endmodule


module instruction_memory(
    clock,
    read,
    address,
    readdata,
    busywait
);
input               clock;
input               read;
input[5:0]          address;
output reg [127:0]  readdata;
output reg          busywait;

reg readaccess;

//Declare memory array 1024x8-bits 
reg [7:0] memory_array [1023:0];

//Initialize instruction memory
initial
begin
    busywait = 0;
    readaccess = 0;

    // Sample program given below. You may hardcode your software program here, or load it from a file:
    {memory_array[10'd3],  memory_array[10'd2],  memory_array[10'd1],  memory_array[10'd0]}  = 32'b00000000000001000000000000011001; // loadi 4 #25
    {memory_array[10'd7],  memory_array[10'd6],  memory_array[10'd5],  memory_array[10'd4]}  = 32'b00000000000001010000000000100011; // loadi 5 #35
    {memory_array[10'd11], memory_array[10'd10], memory_array[10'd9],  memory_array[10'd8]}  = 32'b00000010000001100000010000000101; // add 6 4 5
    {memory_array[10'd15], memory_array[10'd14], memory_array[10'd13], memory_array[10'd12]} = 32'b00000000000000010000000001011010; // loadi 1 90
    {memory_array[10'd19], memory_array[10'd18], memory_array[10'd17], memory_array[10'd16]} = 32'b00000011000000010000000100000100; // sub 1 1 4
end

//Detecting an incoming memory access
always @(read)
begin
    busywait = (read)? 1 : 0;
    readaccess = (read)? 1 : 0;
end

//Reading
always @(posedge clock)
begin
    if(readaccess)
    begin
        readdata[7:0]     = #40 memory_array[{address,4'b0000}];
        readdata[15:8]    = #40 memory_array[{address,4'b0001}];
        readdata[23:16]   = #40 memory_array[{address,4'b0010}];
        readdata[31:24]   = #40 memory_array[{address,4'b0011}];
        readdata[39:32]   = #40 memory_array[{address,4'b0100}];
        readdata[47:40]   = #40 memory_array[{address,4'b0101}];
        readdata[55:48]   = #40 memory_array[{address,4'b0110}];
        readdata[63:56]   = #40 memory_array[{address,4'b0111}];
        readdata[71:64]   = #40 memory_array[{address,4'b1000}];
        readdata[79:72]   = #40 memory_array[{address,4'b1001}];
        readdata[87:80]   = #40 memory_array[{address,4'b1010}];
        readdata[95:88]   = #40 memory_array[{address,4'b1011}];
        readdata[103:96]  = #40 memory_array[{address,4'b1100}];
        readdata[111:104] = #40 memory_array[{address,4'b1101}];
        readdata[119:112] = #40 memory_array[{address,4'b1110}];
        readdata[127:120] = #40 memory_array[{address,4'b1111}];
        busywait = 0;
        readaccess = 0;
    end
end
 
endmodule
module data_cache(
    clock,
    reset,
    readin,
    writein,
    addressin,
    writedatain,
    readdatacpu,
    busywaitcpu,
    readout,
    writeout,
    addressout,
    writedataout,
    readdatadc,
    busywaitdc
);
input               clock;
input               reset;
input               readin;
input               writein;
input               busywaitdc;
input[7:0]          addressin;
input[7:0]          writedatain;
input[31:0]          readdatadc;
output reg             readout;
output reg            writeout;
output reg [5:0]    addressout;
output reg [31:0]    writedataout;
output reg [7:0]    readdatacpu;
output reg          busywaitcpu;


reg [31:0] cachemem[7:0];
reg [2:0] addressTag [7:0];
reg  dbit [7:0];
reg  vbit [7:0];
reg [7:0] word;
wire [2:0] index;
wire [1:0] offset;
wire [2:0] tag;
reg [31:0] block;
wire valid;
wire dirty;
wire operator;  //checking the tag
wire x1,x2,x3,out;
wire hit;

 //assigning
assign index=addressin[4:2]; 
assign offset=addressin[1:0];


always @ (addressin,readin,writein) 
 begin
    if(readin || writein)
	    busywaitcpu=1;
	else
		busywaitcpu=0;
    
 end

always @(posedge clock) begin
  if(hit) 
    busywaitcpu = 0 ;


end

always @ (addressin) begin
		#1 block=cachemem[index];
	end
	assign  #1 tag=addressTag[index];  
	assign  #1 valid=vbit[index];
	assign  #1 dirty=dbit[index];


	
	xnor xnor1(x1,tag[0],addressin[5]); 
	xnor xnor2(x2,tag[1],addressin[6]);
	xnor xnor3(x3,tag[2],addressin[7]);
	and and1(out,x1,x2,x3);

    assign #1 operator=out;

    and and2(hit,operator,valid);

    always @ (*)
	begin 
			
		case(offset)
			2'b00:
                 #1 word=block[31:24];
			2'b01: 
                #1 word=block[25:16];
			2'b10:
                 #1 word=block[15:8];
			2'b11:
                 #1 word=block[7:0];
		endcase
		
		
		if(hit) begin 
			readdatacpu=word;
		end
	end
    always @ (posedge clock,hit)
	begin 
		if (hit && writein) begin
			#1
			dbit[index]=1'd1;
			case(offset)
				2'b00:
                      cachemem[index][31:24]=writedatain;
				2'b01:
                      cachemem[index][25:16]=writedatain;
				2'b10:
                      cachemem[index][15:8]=writedatain;
				2'b11:
                      cachemem[index][7:0]=writedatain;
			endcase
		end
	end
	
    /* Cache Controller FSM Start */

    parameter IDLE = 3'b000, MEM_READ = 3'b001,WRITE_BACK=3'b010, CACHE_UPDATE=3'b011;
    reg [2:0] state, next_state;

    // combinational next state logic
    always @(*)
    begin
        case (state)
            IDLE:
                if ((readin || writein) && !dirty && !hit)  
                    next_state = MEM_READ;
                else if ((readin || writein) && dirty && !hit)
                    next_state = WRITE_BACK;
                else
                    next_state = IDLE;
            
            MEM_READ:
                if (!busywaitdc)
                    next_state = CACHE_UPDATE;
                else    
                    next_state = MEM_READ;
					
			WRITE_BACK:
                if (!busywaitdc)
                    next_state = MEM_READ;
                else    
                    next_state = WRITE_BACK;
					
			CACHE_UPDATE:
                next_state = IDLE;
            
        endcase
    end

    // combinational output logic
    always @(*)
    begin
        case (state)
        IDLE:
            begin
                readout = 0;
                writeout = 0;
                addressout = 6'dx;
                writedataout = 6'dx;
                busywaitcpu = 0;
            end
         
            MEM_READ: 
            begin
                readout = 1;
                writeout = 0;
                addressout = addressin[7:2];
                writedataout = 32'dx;
                busywaitcpu = 1;
            end
			
			WRITE_BACK:
			begin
                readout = 0;
                writeout = 1;
                addressout = {tag, index};
			    writedataout = block;
                busywaitcpu = 1;
            end
			
			CACHE_UPDATE:
			begin 
                readout = 0;
                writeout = 0;
                addressout = 6'dx;
                writedataout =32'dx;
				busywaitcpu = 1;
				#1 addressTag[index][2:0]=addressin[7:5];
				
				cachemem[index][7:0]=readdatadc[7:0];
				cachemem[index][15:8]=readdatadc[15:8];
				cachemem[index][23:16]=readdatadc[23:16];
				cachemem[index][31:24]=readdatadc[31:24];
				dbit[index]=1'b0;
				vbit[index]=1'b1;
                
            end
			
			
            
        endcase
    end

    // sequential logic for state transitioning 
    integer i;
	always @(reset)begin				
		if(reset)begin
			
			for (i=0; i<8; i=i+1)begin
				vbit[i] <= 0;
				dbit[i] <= 0;
				cachemem[i] <= 0;
				busywaitcpu=0;
			end	
			
		end
	end
    /* Cache Controller FSM End */

endmodule



        






















module cpu(PC,INSTRUCTION,CLK,RESET,READDATA,BUSYWAIT,WRITE,READ,WRITEDATA,ADDRESS);

	input [31:0] INSTRUCTION;
    input [7:0] READDATA;
	input CLK,RESET,BUSYWAIT;
	output reg [31:0] PC;
    output reg [7:0] WRITEDATA,ADDRESS;
    output reg READ,WRITE;

    wire zero;
	reg write;//control signal for writing
	reg selectim,selectneg,selectp,selectj,jump,branch,selectreg;   //control signals for the multiplexers
	reg [2:0] ALUOP; // ALU opreation control signal
	wire [7:0] IN,ALURESULT,OUT1,OUT2,COMP2,DATA2,OPERAND2; // inputs and out put of the units
	reg [31:0]	PCj,PCnext;
   reg signed[31:0] offset;
  //  wire[31:0] PCout
  //making sure that the off set is being extended
  
			
	always @ (INSTRUCTION) begin  
    READ=1'b0;
    WRITE=1'b0; 
    selectreg=1'b0;
		case(INSTRUCTION[31:24])
			8'b00000000 : #1 begin            //loading
						write=1'b1;
						ALUOP=3'b000;
						selectim=1'b0;
						selectneg=1'b0;
                          branch =1'b0;
						 jump=0;
					end
			
			8'b00000001 : #1 begin            //mov
						WRITE=1'b1;
						ALUOP=3'b000;
						selectim=1'b1;
						selectneg=1'b0;
                        branch =1'b0;
						 jump=0;
					end
					
			8'b00000010 : #1 begin            //add
						WRITE=1'b1;
						ALUOP=3'b001;
						selectim=1'b1;
						selectneg=1'b0;
                        branch =1'b0;
                         jump=0;
					end
					
			8'b00000011 : #1 begin            //sub
						WRITE=1'b1;
						ALUOP=3'b001;
						selectim=1'b1;
						selectneg=1'b1;
                       jump=0;
                       branch =1'b0;
					end
					
			8'b00000100 : #1 begin            //and
						WRITE=1'b1;
						ALUOP=3'b010;
						selectim=1'b1;
						selectneg=1'b0;
                        jump=0;
                        branch =1'b0;
					end
			
			8'b00000101 : #1 begin            //or
						WRITE=1'b1;
						ALUOP=3'b011;
						selectim=1'b1;
						selectneg=1'b0;
                        jump=0;
                        branch =1'b0;
                        
					end
            8'b00000110 : #1 begin
                        WRITE=1'b0;
                        ALUOP=3'b100;
                        branch =1'b1;
                        
                        end
                        
             8'b00000111 : #1 begin
                         branch =1'b0;
                         jump =1'b1;
                        end
            8'b00001000 : #1 begin
                        ALUOP = 3'b000;
                        READ=1;
                        selectim=1'b1;
                        selectreg=1'b1;
                        end

            8'b00001001 : #1 begin
                         ALUOP = 3'b000;
                         READ=1;
                        selectim=1'b0;
                        selectreg=1'b1;
                        end
            8'b00001010 : #1 begin
                         ALUOP = 3'b000;
                         WRITE=1;
                        selectim=1'b1;
                        end
            8'b00001011 : #1 begin
                         ALUOP = 3'b000;
                         WRITE=1;
                        selectim=1'b0;
                        end
			// default : #1 begin                // setting load as default for other opcodes 
            //         	WRITE=1'b1;
			// 			ALUOP=3'b000;
			// 			selectim=1'b0;
			// 			selectneg=1'b0;
            //         end	
			
		endcase	
	end

//always@(jump||branch) begin
   
//end

 
	
  //  always@(*)begin
    
   //end

	always @ (posedge CLK && !BUSYWAIT) begin
     WRITEDATA=OUT2;
    ADDRESS=ALURESULT;
		if(RESET==0) begin
            if(jump)begin

                 if(INSTRUCTION[23]==0) begin
                   offset[31:8] = 24'b000000000000000000000000;
                   offset[7:0] = INSTRUCTION[23:16];
                   offset = offset *4;//multiplying by 4
                 end
                 else begin
                  offset[31:8]=24'b111111111111111111111111;
                   offset[7:0]=INSTRUCTION[23:16];
                  offset = offset *4;//multiplying by 4
                 end      
              #3  PC= PC + offset; //pc update delay of #1 and adder delay #2
              end
             if(zero && branch)begin
             if(INSTRUCTION[23]==0) begin
                   offset[31:8] = 24'b000000000000000000000000;
                   offset[7:0] = INSTRUCTION[23:16];
                   offset = offset*4;//multiplying by 4
                 end
                 else begin
                  offset[31:8]=24'b111111111111111111111111;
                   offset[7:0]=INSTRUCTION[23:16];
                  offset = offset*4;//multiplying by 4
                 end      
             #3  PC= PC + offset; //pc update delay of #1 and adder delay #2
             end
            else begin
			 #3 PC=PC+ 8'd4;   //pc update delay of #1 and adder delay #2
             end
             //selectp = zero & branch;
            // selectj = selectp | jump;
            
		end
	end


	always@(BUSYWAIT)begin
    WRITEDATA=OUT2;
    ADDRESS=ALURESULT;
    #40 PC=PC+8'd4;
    end

	always @ (RESET) begin
         
        
		#3 PC=-32'd4;  //pc update delay  of #1 and adder delay #2
	end	

	
	reg_file myreg_file(IN, OUT1, OUT2,INSTRUCTION[18:16],INSTRUCTION[10:8],INSTRUCTION[2:0],WRITE,CLK,RESET);
	comp negated(COMP2,OUT2);
	MUX_2x1 muxcomp(OUT2,COMP2,selectneg,DATA2);
	MUX_2x1 mymux(INSTRUCTION[7:0],DATA2,selectim,OPERAND2);
    MUX_2x1 memmux(ALURESULT,READDATA,selectreg,IN);
	alu alu_1(OUT1,OPERAND2,ALURESULT,ALUOP,zero); 
   
//	MUX32_2x1 muxj(PCnext,PCj,selectj,PC);
	
   

			
			
		

endmodule