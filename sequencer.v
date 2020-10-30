// Module: sequencer: Sequences the inputs to the Fusion PEs based on the configuration read from the SRAM mem blocks. 
//       : 4x4bitBrick: Module name can be a misnomer because it is actually a 5x5 signed multiply. But for the sake of conceptual understanding
//                      it is maintained like this. That is, it handles the [3:0],[7:4](4 bits each) part of an 8 bit value.
//                      Just multiplies them and returns output in registers considering multiplication itself will have a high logic delay and 
//                      i would like to pipeline the shifting in the module which feeds from this
// Author: kputrev@ncsu.edu
module sequencer(
    input               clock, reset,
    input               go,
    output              busy,
    input   [15:0]      input_word, weight_word,
    output  reg [11:0]      input_read_addr,weight_read_addr, output_write_addr,
    output  reg            output_write_enable,
    output  reg signed [15:0]  output_write_data
     );

localparam  RESET_STATE = 3'b000,
            FIRST_WORD_READ = 3'b001,
            SECOND_WORD_READ_FIRST_AVAILABLE = 3'b010,
            SET_CONFIG_DONE = 3'b011,
            COMPUTE_SUMS = 3'b100,
            COMPUTE_SUMS_INITIATED = 3'b101;

reg [15:0] I,W;
reg signed [4:0] I0,I1,I2,I3,I4,I5,I6,I7, W0,W1,W2,W3,W4,W5,W6,W7;
wire signed [15:0] PS0,PS1,PS2,PS3,PS4,PS5,PS6,PS7;

reg bb0, bb1, bb2, bb3, bb4, bb5, bb6, bb7;
reg [3:0] shiftbb0, shiftbb1, shiftbb2, shiftbb3, shiftbb4, shiftbb5, shiftbb6, shiftbb7;
reg [1:0] servInput, servWeight;

reg o0,o1,o2,o3;
reg signed [15:0] O0,O1,O2,O3,O4,O5,O6,O7;
reg signed [15:0] O0_1,O1_1,O2_1,O3_1,O4_1,O5_1,O6_1,O7_1;

reg bb0_reg, bb1_reg, bb2_reg, bb3_reg, bb4_reg, bb5_reg, bb6_reg, bb7_reg; //Bit Brick Select registers configured at config time. THey are stable while config_done is high.
reg [3:0] shiftbb0_reg, shiftbb1_reg, shiftbb2_reg, shiftbb3_reg, shiftbb4_reg, shiftbb5_reg, shiftbb6_reg, shiftbb7_reg;  //Bit Brick shift registers that are configure at config time. They are stable while config_done is high.
reg [1:0] serv_inp_count, serv_weight_count; // Counters that determine when read requests are sent to the Input and weight SRAMs.
reg [2:0] inp_counter, w_counter, inp_counter_t1, w_counter_t1, inp_counter_t2, w_counter_t2;
reg [7:0] counter_partial, totalCount_reg, perOutIter;
reg [3:0] totalCountShift;
reg  config_done,config_read_done;
reg [3:0] inp_config,w_config;

reg [5:0] current_state_i;
reg [5:0] next_state_i, next_state_w;
reg [1:0] addr_sel,input_addr_sel,w_addr_sel, weight_addr_sel;
reg       first_word_in_set;
reg       second_word_in_set;
reg       config_read_done_sm;
reg       config_done_sm;
reg       clear_config;
reg       start_counter;
reg	  finish;
wire   dec_total_count, next_word_in_partial, last_partial_in_set, counter_last_word_in_partial, next_weight;
reg     write_one_output,set_done;

reg	state_is_compute_init, state_is_compute_init_t1, state_is_compute_init_t2,state_is_compute_init_t3,state_is_compute_init_t4;
//0: change state and send request to mem location 0 when you get go.
//1: save total inputs/weights to a reg. Also store base address of current set.  
always@(posedge clock)
begin
	if(reset)
	begin
		state_is_compute_init 	<= 1'b0;
		state_is_compute_init_t1<= 1'b0;
		state_is_compute_init_t2<= 1'b0;
		state_is_compute_init_t3<= 1'b0;
		state_is_compute_init_t4<= 1'b0;
	end
	else
	begin
		if(current_state_i == COMPUTE_SUMS_INITIATED) state_is_compute_init <= 1'b1; else state_is_compute_init <= 1'b0;
		state_is_compute_init_t1 <= state_is_compute_init;
		state_is_compute_init_t2 <= state_is_compute_init_t1;
		state_is_compute_init_t3 <= state_is_compute_init_t2;
		state_is_compute_init_t4 <= state_is_compute_init_t3;
	end
end

// Registering Inputs and Weights so they can be shifted and returned in non-single-cycle reads.
always@(posedge clock)
begin
    if(reset)
    begin
        I <= 16'b0;
    end
    else if(finish)
    begin
        I <= 16'b0;
    end
    //else if(current_state_i==COMPUTE_SUMS_INITIATED)
    else if(state_is_compute_init_t1)
    begin
        //Input MUXing 
        if(inp_counter_t2==2'b00) I <= input_word;
        else if((servInput==2'b01)&&(inp_counter_t2 == 2'b01)) I <= I>>8;
        else if((servInput==2'b11)&&(inp_counter_t2 == 2'b01)) I <= I>>4;
        else if((inp_counter_t2==2'b10)) I <= I>>4;
        else if(inp_counter_t2==2'b11) I <= I>>4;
        else I <= 16'hDEAD;
    end
end

//Shifting Input and Weight Counters by 2 cycles;
always @(posedge clock)
begin
	if(reset)
	begin
		inp_counter_t1 <= 2'b00; inp_counter_t2 <= 2'b00;
		w_counter_t1 <= 2'b00; w_counter_t2 <= 2'b00;
	end
	else
	begin
		inp_counter_t1 <= inp_counter;
		inp_counter_t2 <= inp_counter_t1;
		w_counter_t1 <= w_counter;
		w_counter_t2 <= w_counter_t1;
	end
end 

// Weights MUXing.
always@(posedge clock)
begin
    if(reset)
    begin
        W <= 16'b0;
    end
    else if(finish)
    begin
        W <= 16'b0;
    end
    else if(state_is_compute_init_t1)
    begin
        //Input MUXing 
        if(w_counter_t2==2'b00) W <= weight_word;
        else if((servWeight==2'b01)&&(w_counter_t2 == 2'b01)) W <= W>>8;
        else if((servWeight==2'b11)&&(w_counter_t2 == 2'b01)) W <= W>>4;
        else if((w_counter_t2==2'b10)) W <= W>>4;
        else if(w_counter_t2==2'b11) W <= W>>4;
        else W <= 16'hDEAD;
    end
end

// Basic State Machine switching.
always@(posedge clock)
begin
    if(reset)
    begin
        current_state_i <= RESET_STATE;
    end
    else
    begin
        current_state_i <= next_state_i;
    end
end

//Inputs State Machine
always@(*)
begin
    casex(current_state_i)
    RESET_STATE:
        begin
            addr_sel = 2'b00;
            first_word_in_set = 1'b0;
            second_word_in_set = 1'b0;
            config_read_done_sm = 1'b0;
            config_done_sm = 1'b0;
            clear_config = 1'b1;
            if(go)
            begin
                next_state_i = FIRST_WORD_READ;
                addr_sel = 2'b00; //Change in next cycle
                clear_config = 1'b0;
            end
            else
                next_state_i = RESET_STATE;
        end
    FIRST_WORD_READ:
        begin
            addr_sel = 2'b01;
            first_word_in_set = 1'b0;
            second_word_in_set = 1'b0;
            config_read_done_sm = 1'b0;
            config_done_sm = 1'b0;
            clear_config = 1'b0;
            second_word_in_set = 1'b0;
            next_state_i = SECOND_WORD_READ_FIRST_AVAILABLE;
        end
    SECOND_WORD_READ_FIRST_AVAILABLE:
        begin
            addr_sel = 2'b10;
            first_word_in_set = 1'b1;
            second_word_in_set = 1'b0;
            config_read_done_sm = 1'b0;
            config_done_sm = 1'b0; 
            clear_config = 1'b0;
            next_state_i = SET_CONFIG_DONE;
        end
    SET_CONFIG_DONE :
        begin
            addr_sel = 2'b10;
            first_word_in_set = 1'b0;
            second_word_in_set = 1'b1;
            config_read_done_sm = 1'b1;
            config_done_sm = 1'b0;
            clear_config = 1'b0;
	    if(busy)
            	next_state_i = COMPUTE_SUMS;
	    else
		next_state_i = RESET_STATE;
        end
    COMPUTE_SUMS:
        begin
            addr_sel = 2'b10;
            first_word_in_set = 1'b0;
            second_word_in_set = 1'b0;
            config_read_done_sm = 1'b0;
            config_done_sm = 1'b1;
            clear_config = 1'b0;
            next_state_i = COMPUTE_SUMS_INITIATED;
        end
    COMPUTE_SUMS_INITIATED:
        begin
            addr_sel = 2'b10;
            first_word_in_set = 1'b0;
            second_word_in_set = 1'b0;
            config_read_done_sm = 1'b0;
            config_done_sm = 1'b1;
            clear_config = 1'b0;
            if(set_done)
                next_state_i = FIRST_WORD_READ;
	    else 
		next_state_i = COMPUTE_SUMS_INITIATED;
	  
        end
     default:
	begin
	    addr_sel = 2'b00;
	    config_read_done_sm = 1'b0;
	    config_done_sm = 1'b0;
	    first_word_in_set = 1'b0;
	    second_word_in_set = 1'b0;
	    clear_config = 1'b0;
	    next_state_i = RESET_STATE;
	end
    endcase
end

// Ccounter Logic to decide when to rollback the addr line to get next partial sum and when to go to next set of values.
always@(posedge clock)
begin
    if(reset)
    begin
        counter_partial <= 8'b0;
    end
    else if((current_state_i==COMPUTE_SUMS_INITIATED) && next_word_in_partial)
    begin
        if(counter_partial == (perOutIter))
        begin
            counter_partial <= 8'b1;            
        end
	else
	begin
        counter_partial <= counter_partial + 8'b1;
	end
    end
    else 
    begin
        counter_partial <= 8'b0;
    end
end
assign counter_last_word_in_partial = (counter_partial==(perOutIter)) ? 1'b1 : 1'b0;
assign dec_total_count = (counter_partial==(perOutIter)) ? 1'b1 : 1'b0;


//Serve Input Counter
always@(posedge clock)
begin
    if(reset)
    begin
        inp_counter <= 2'b00;
    end
    else if(current_state_i == COMPUTE_SUMS_INITIATED)
    begin
        if(inp_counter == serv_inp_count)
        begin
            inp_counter <= 2'b00;
        end
	else
	begin
        inp_counter <= inp_counter + 2'b01; 
	end
    end
    else
    begin
        inp_counter <= 2'b00;
    end
end
assign next_word_in_partial = (inp_counter==2'b00) ? 1'b1 : 1'b0;

//Serve Weight Counter
always@(posedge clock)
begin
    if(reset)
    begin
        w_counter <= 2'b00;
    end
    else if(current_state_i == COMPUTE_SUMS_INITIATED)
    begin
        if(w_counter == serv_weight_count )
        begin
            w_counter <= 2'b00;
        end
	else
	begin
        w_counter <= w_counter + 2'b01;
	end
    end
    else 
    begin
        w_counter <= 2'b00;
    end
end
assign next_weight = (w_counter==2'b00) ? 1'b1 : 1'b0;

//Set Counter Values for each configuration///////////////////////////////////////////////////////////////////////////
always@(*)
begin
   casex(inp_config)
   4'b0010: totalCountShift = 2'b11;
   4'b0100: totalCountShift = 2'b10;
   4'b1000: totalCountShift = 2'b01;
   default: totalCountShift = 2'b00;
   endcase 
end


// Input Read Address Master Muxer
always@(*)
begin
if(current_state_i == COMPUTE_SUMS_INITIATED)
begin
    if(last_partial_in_set && next_word_in_partial && counter_last_word_in_partial)  
    begin
        input_addr_sel = 2'b01; 
        set_done = 1'b1;
        write_one_output = 1'b1;
    end
    else if(counter_last_word_in_partial && next_word_in_partial)
    begin
        input_addr_sel = 2'b11;
        write_one_output = 1'b1;
        set_done = 1'b0;
    end
    else if(next_word_in_partial)
    begin
        input_addr_sel = 2'b01;
        set_done = 1'b0;
        write_one_output = 1'b0;
    end
    else
    begin
        input_addr_sel = 2'b10;
        set_done = 1'b0;
        write_one_output = 1'b0;
    end
end
else    
begin
    input_addr_sel = addr_sel;
    set_done = 1'b0;
    write_one_output = 1'b0;
end
end

// Input Read Address Selection/Incrmentation Logic
always@(posedge clock)
begin
    if(reset)
        input_read_addr <= 12'b0;
    else if(input_addr_sel == 2'b00)
        input_read_addr <= 12'b0;
    else if(input_addr_sel == 2'b01)
        input_read_addr <= input_read_addr + 12'b1;
    else if(input_addr_sel == 2'b10)
        input_read_addr <= input_read_addr;
    else if(input_addr_sel == 2'b11)
        input_read_addr <= input_read_addr - (perOutIter-12'b1);
    else
        input_read_addr <= 12'hFFF;
end

always@(*)
begin
    if(current_state_i == COMPUTE_SUMS_INITIATED)
    begin
        if(next_weight)
        weight_addr_sel = 2'b01;
        else
        weight_addr_sel = 2'b10;
    end
    else
    begin
        weight_addr_sel = addr_sel;
    end
end

// Weight Read Address Selecetion/ Incrementing Logic.
always@(posedge clock)
begin
    if(reset)
        weight_read_addr <= 12'b0;
    else if(weight_addr_sel == 2'b00)
        weight_read_addr <= 12'b0;
    else if(weight_addr_sel == 2'b01)
        weight_read_addr <= weight_read_addr + 12'b1;
    else if(weight_addr_sel == 2'b10)
        weight_read_addr <= weight_read_addr;
    else
        weight_read_addr <= 12'hFFF;
end

// FF to store the number of interations for one Output. = (Total Inputs >> totalCountShift(1 if 8, 2 if 4, 3 if 2))
always@(posedge clock)
begin
    if(reset)
    begin
        perOutIter <= 8'h00;
        config_done <= 1'b0;
    end
    else if(config_read_done)
    begin
        perOutIter <= totalCount_reg >> totalCountShift;
        config_done <= config_done_sm;
    end

    else if(clear_config)
    begin
        perOutIter <= 8'h00;
        config_done <= 1'b0;
    end
end

//FF to store total Count
always@(posedge clock)
begin
    if(reset)
    begin
        totalCount_reg <= 8'h00;
        finish <= 1'b1;
    end
    else if(first_word_in_set)
    begin
	finish <= 1'b0;
        totalCount_reg <= input_word;
        if(input_word==16'hFF)
            finish <= 1'b1;
    end
    else if(config_done && dec_total_count)
    begin
        totalCount_reg <= totalCount_reg - 8'b1;
    end
    else if(clear_config)
    begin
        totalCount_reg <= 8'h00;
    end
end
assign last_partial_in_set = (totalCount_reg==8'h01) ? 1'b1 : 1'b0;
assign busy = !finish;

//FF to store input and weight config
always@(posedge clock)
begin
   if(reset)
   begin
       inp_config <= 4'b0000;
       w_config   <= 4'b0000;
       config_read_done <= 1'b0;
   end
   else if(second_word_in_set)
   begin
       inp_config <= input_word;
       w_config   <= weight_word;
       config_read_done <= config_read_done_sm;
   end
   else if(clear_config)
   begin
       inp_config <= 4'b0000;
       w_config   <= 4'b0000;
       config_read_done <= 1'b0;
   end
   else
	begin
		config_read_done <= 1'b0;
	end
end

//Configuring select lines and counters based on config.
always@(*)
begin
    casex({inp_config,w_config})
    8'b00100010: begin servInput=2'b00; servWeight=2'b00; bb0=1'b1; bb1=1'b1; bb2=1'b1; bb3=1'b1; bb4=1'b1; bb5=1'b1; bb6=1'b1; bb7=1'b1; shiftbb0=4'h0; shiftbb1=4'h0; shiftbb2=4'h0; shiftbb3=4'h0; shiftbb4=4'h0; shiftbb5=4'h0; shiftbb6=4'h0; shiftbb7=4'h0; end
    8'b00100100: begin servInput=2'b01; servWeight=2'b00; bb0=1'b1; bb1=1'b1; bb2=1'b0; bb3=1'b0; bb4=1'b1; bb5=1'b1; bb6=1'b0; bb7=1'b0; shiftbb0=4'h0; shiftbb1=4'h0; shiftbb2=4'h0; shiftbb3=4'h0; shiftbb4=4'h0; shiftbb5=4'h0; shiftbb6=4'h0; shiftbb7=4'h0; end
    8'b00101000: begin servInput=2'b11; servWeight=2'b00; bb0=1'b1; bb1=1'b1; bb2=1'b0; bb3=1'b0; bb4=1'b1; bb5=1'b1; bb6=1'b0; bb7=1'b0; shiftbb0=4'h0; shiftbb1=4'h4; shiftbb2=4'h0; shiftbb3=4'h0; shiftbb4=4'h0; shiftbb5=4'h4; shiftbb6=4'h0; shiftbb7=4'h0; end
    8'b01000010: begin servInput=2'b00; servWeight=2'b01; bb0=1'b1; bb1=1'b1; bb2=1'b0; bb3=1'b0; bb4=1'b1; bb5=1'b1; bb6=1'b0; bb7=1'b0; shiftbb0=4'h0; shiftbb1=4'h0; shiftbb2=4'h0; shiftbb3=4'h0; shiftbb4=4'h0; shiftbb5=4'h0; shiftbb6=4'h0; shiftbb7=4'h0; end
    8'b01000100: begin servInput=2'b00; servWeight=2'b00; bb0=1'b1; bb1=1'b1; bb2=1'b0; bb3=1'b0; bb4=1'b1; bb5=1'b1; bb6=1'b0; bb7=1'b0; shiftbb0=4'h0; shiftbb1=4'h0; shiftbb2=4'h0; shiftbb3=4'h0; shiftbb4=4'h0; shiftbb5=4'h0; shiftbb6=4'h0; shiftbb7=4'h0; end
    8'b01001000: begin servInput=2'b01; servWeight=2'b00; bb0=1'b1; bb1=1'b1; bb2=1'b0; bb3=1'b0; bb4=1'b1; bb5=1'b1; bb6=1'b0; bb7=1'b0; shiftbb0=4'h0; shiftbb1=4'h4; shiftbb2=4'h0; shiftbb3=4'h0; shiftbb4=4'h0; shiftbb5=4'h4; shiftbb6=4'h0; shiftbb7=4'h0; end
    8'b10000010: begin servInput=2'b00; servWeight=2'b11; bb0=1'b1; bb1=1'b1; bb2=1'b0; bb3=1'b0; bb4=1'b1; bb5=1'b1; bb6=1'b0; bb7=1'b0; shiftbb0=4'h0; shiftbb1=4'h4; shiftbb2=4'h0; shiftbb3=4'h0; shiftbb4=4'h0; shiftbb5=4'h4; shiftbb6=4'h0; shiftbb7=4'h0; end
    8'b10000100: begin servInput=2'b00; servWeight=2'b01; bb0=1'b1; bb1=1'b1; bb2=1'b0; bb3=1'b0; bb4=1'b1; bb5=1'b1; bb6=1'b0; bb7=1'b0; shiftbb0=4'h0; shiftbb1=4'h4; shiftbb2=4'h0; shiftbb3=4'h0; shiftbb4=4'h0; shiftbb5=4'h4; shiftbb6=4'h0; shiftbb7=4'h0; end
    8'b10001000: begin servInput=2'b00; servWeight=2'b00; bb0=1'b1; bb1=1'b1; bb2=1'b1; bb3=1'b1; bb4=1'b1; bb5=1'b1; bb6=1'b1; bb7=1'b1; shiftbb0=4'h0; shiftbb1=4'h4; shiftbb2=4'h4; shiftbb3=4'h8; shiftbb4=4'h0; shiftbb5=4'h4; shiftbb6=4'h4; shiftbb7=4'h8; end
    default:     begin servInput=2'b00; servWeight=2'b00; bb0=1'b0; bb1=1'b0; bb2=1'b0; bb3=1'b0; bb4=1'b0; bb5=1'b0; bb6=1'b0; bb7=1'b0; shiftbb0=4'h0; shiftbb1=4'h0; shiftbb2=4'h0; shiftbb3=4'h0; shiftbb4=4'h0; shiftbb5=4'h0; shiftbb6=4'h0; shiftbb7=4'h0; end
    endcase
end

always@(posedge clock)
begin
    if(reset)
    begin
        serv_inp_count <= 2'b00; serv_weight_count <= 2'b00;
        bb0_reg <= 1'b0;       bb1_reg <= 1'b0;       bb2_reg <= 1'b0;       bb3_reg <= 1'b0;       bb4_reg <= 1'b0;       bb5_reg <= 1'b0;       bb6_reg <= 1'b0;        bb7_reg <= 1'b0;
        shiftbb0_reg <= 4'b0000;       shiftbb1_reg <= 4'b0000;       shiftbb2_reg <= 4'b0000;       shiftbb3_reg <= 4'b0000;       shiftbb4_reg <= 4'b0000;       shiftbb5_reg <= 4'b0000;       shiftbb6_reg <= 4'b0000;       shiftbb7_reg <= 4'b0000;
    end
    if(config_read_done)
    begin
        serv_inp_count <= servInput; serv_weight_count <= servWeight;
        bb0_reg <= bb0;
        bb1_reg <= bb1;
        bb2_reg <= bb2;
        bb3_reg <= bb3;
        bb4_reg <= bb4;
        bb5_reg <= bb5;
        bb6_reg <= bb6;
        bb7_reg <= bb7;
        shiftbb0_reg <= shiftbb0;
        shiftbb1_reg <= shiftbb1;
        shiftbb2_reg <= shiftbb2;
        shiftbb3_reg <= shiftbb3;
        shiftbb4_reg <= shiftbb4;
        shiftbb5_reg <= shiftbb5;
        shiftbb6_reg <= shiftbb6;
        shiftbb7_reg <= shiftbb7;
    end
    else if(clear_config)
    begin
        serv_inp_count     <= 2'b00; serv_weight_count <= 2'b00;
        bb0_reg      <= 1'b0;
        bb1_reg      <= 1'b0;
        bb2_reg      <= 1'b0;
        bb3_reg      <= 1'b0;
        bb4_reg      <= 1'b0;
        bb5_reg      <= 1'b0;
        bb6_reg      <= 1'b0;
        bb7_reg      <= 1'b0;
        shiftbb0_reg <= 4'b0;
        shiftbb1_reg <= 4'b0;
        shiftbb2_reg <= 4'b0;
        shiftbb3_reg <= 4'b0;
        shiftbb4_reg <= 4'b0;
        shiftbb5_reg <= 4'b0;
        shiftbb6_reg <= 4'b0;
        shiftbb7_reg <= 4'b0;
    end
end
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//datapath muxing.
always@(*)
begin
    casex({inp_config,w_config})
    8'b00100010 : //2,2
    begin
        I0=$signed(I[1:0]);   W0=$signed(W[1:0]); 
        I1=$signed(I[3:2]);   W1=$signed(W[3:2]);
        I2=$signed(I[5:4]);   W2=$signed(W[5:4]);
        I3=$signed(I[7:6]);   W3=$signed(W[7:6]);
        I4=$signed(I[9:8]);   W4=$signed(W[9:8]);
        I5=$signed(I[11:10]); W5=$signed(W[11:10]);
        I6=$signed(I[13:12]); W6=$signed(W[13:12]);
        I7=$signed(I[15:14]); W7=$signed(W[15:14]);
    end
    8'b00100100 : //2,4
    begin
        I0= $signed(I[1:0]);  W0=$signed(W[3:0]) ;
        I1= $signed(I[3:2]);  W1=$signed(W[7:4]) 	;
        I2= 0;                W2=0;
        I3= 0;                W3=0;
        I4= $signed(I[5:4]);  W4=$signed(W[11:8]) ;
        I5= $signed(I[7:6]);  W5=$signed(W[15:12]);
        I6= 0;                W6=0;
        I7= 0;                W7=0;
    end 
    8'b00101000 ://2,8
    begin
        I0=$signed(I[1:0]);W0=$signed({1'b0,W[3:0]});
        I1=$signed(I[1:0]);W1=$signed(W[7:4]) ;
        I2=0;W2=0;
        I3=0;W3=0;
        I4=$signed(I[3:2]);W4=$signed({1'b0,W[11:8]}) 	;
        I5=$signed(I[3:2]);W5=$signed(W[15:12])	;
        I6=0;W6=0;	
        I7=0;W7=0;

    end
    8'b01000010 ://4,2
    begin
        I0=$signed(I[3:0]);W0=$signed(W[1:0]);
        I1=$signed(I[7:4]);W1=$signed(W[3:2]) ;
        I2=0;W2=0;
        I3=0;W3=0;
        I4=$signed(I[11:8]);W4=$signed(W[5:4]) 	;
        I5=$signed(I[15:12]);W5=$signed(W[7:6])	;
        I6=0;W6=0;	
        I7=0;W7=0;
    end
    8'b01000100 ://4,4
    begin
        I0=$signed(I[3:0]);W0=$signed(W[3:0]);
        I1=$signed(I[7:4]);W1=$signed(W[7:4]) ;
        I2=0;W2=0;
        I3=0;W3=0; 
        I4=$signed(I[11:8]);W4=$signed(W[11:8]) 	;
        I5=$signed(I[15:12]);W5=$signed(W[15:12])	;
        I6=0;W6=0;	
        I7=0;W7=0;
    end 
    8'b01001000 ://4,8
    begin
        I0=$signed(I[3:0]);W0=$signed({1'b0,W[3:0]});
        I1=$signed(I[3:0]);W1=$signed(W[7:4]) ;
        I2=0;W2=0;
        I3=0;W3=0;
        I4=$signed(I[7:4]);W4=$signed({1'b0,W[11:8]}) 	;
        I5=$signed(I[7:4]);W5=$signed(W[15:12])	;
        I6=0;W6=0;	
        I7=0;W7=0;
    end
    8'b10000010 ://8,2
    begin
        W0=$signed(W[1:0]);I0=$signed({1'b0,I[3:0]});
        W1=$signed(W[1:0]);I1=$signed(I[7:4]) ;
        W2=0;I2=0;
        W3=0;I3=0;
        W4=$signed(W[3:2]);I4=$signed({1'b0,I[11:8]}) 	;
        W5=$signed(W[3:2]);I5=$signed(I[15:12])	;
        W6=0;I6=0;	
        W7=0;I7=0;
    end
    8'b10000100 ://8,4
    begin
        W0=$signed(W[3:0]);I0=$signed({1'b0,I[3:0]});
        W1=$signed(W[3:0]);I1=$signed(I[7:4]) ;
        W2=0;I2=0;
        W3=0;I3=0;
        W4=$signed(W[7:4]);I4=$signed({1'b0,I[11:8]}) 	;
        W5=$signed(W[7:4]);I5=$signed(I[15:12])	;
        W6=0;I6=0;	
        W7=0;I7=0;
    end
    8'b10001000: //8,8
    begin
        I0=$signed({1'b0,I[3:0]}); W0=$signed({1'b0,W[3:0]});
        I1=$signed({1'b0,I[3:0]}); W1=$signed(W[7:4]);
        I2=$signed(I[7:4]);        W2=$signed({1'b0,W[3:0]});
        I3=$signed(I[7:4]);        W3=$signed(W[7:4]);
        I4=$signed({1'b0,I[11:8]}); W4=$signed({1'b0,W[11:8]});
        I5=$signed({1'b0,I[11:8]}); W5=$signed(W[15:12]);
        I6=$signed(I[15:12]);        W6=$signed({1'b0,W[11:8]});
        I7=$signed(I[15:12]);        W7=$signed(W[15:12]);
    end
    default: 
    begin
    	I0=0;I1=0;I2=0;I3=0;I4=0;I5=0;I6=0;I7=0;
    	W0=0;W1=0;W2=0;W3=0;W4=0;W5=0;W6=0;W7=0;
    end
    endcase
end
assign PS0 = bb0_reg ? I0*W0 : $signed(16'b0);
assign PS1 = bb1_reg ? I1*W1 : $signed(16'b0);
assign PS2 = bb2_reg ? I2*W2 : $signed(16'b0);
assign PS3 = bb3_reg ? I3*W3 : $signed(16'b0);
assign PS4 = bb4_reg ? I4*W4 : $signed(16'b0);
assign PS5 = bb5_reg ? I5*W5 : $signed(16'b0);
assign PS6 = bb6_reg ? I6*W6 : $signed(16'b0);
assign PS7 = bb7_reg ? I7*W7 : $signed(16'b0);
///////End of Input and Weights Muxing to respective 4x4 bit brick blocks. /////////////////////////////////////////////////////////////////////

always@(posedge clock)
begin
    if(reset)
    begin        
        O0 <= 16'b0;
        O1 <= 16'b0;
        O2 <= 16'b0;
        O3 <= 16'b0; 
        O4 <= 16'b0;
        O5 <= 16'b0;
        O6 <= 16'b0;
        O7 <= 16'b0;
        O0_1 <= 16'b0;
        O1_1 <= 16'b0;
        O2_1 <= 16'b0;
        O3_1 <= 16'b0; 
        O4_1 <= 16'b0;
        O5_1 <= 16'b0;
        O6_1 <= 16'b0;
        O7_1 <= 16'b0;
    end
    else if(state_is_compute_init_t2)
    begin
        O0 <= PS0;
        O1 <= PS1;
        O2 <= PS2;
        O3 <= PS3; 
        O4 <= PS4;
        O5 <= PS5;
        O6 <= PS6;
        O7 <= PS7;
        O0_1 <= O0<<shiftbb0_reg;
        O1_1 <= O1<<shiftbb1_reg;
        O2_1 <= O2<<shiftbb2_reg;
        O3_1 <= O3<<shiftbb3_reg; 
        O4_1 <= O4<<shiftbb4_reg;
        O5_1 <= O5<<shiftbb5_reg;
        O6_1 <= O6<<shiftbb6_reg;
        O7_1 <= O7<<shiftbb7_reg;
    end
    else
    begin
	O0 <= 16'b0;
        O1 <= 16'b0;
        O2 <= 16'b0;
        O3 <= 16'b0;
        O4 <= 16'b0;
        O5 <= 16'b0;
        O6 <= 16'b0;
        O7 <= 16'b0;
    end
end

//Ouput Sum from Partial Sums.
always@(posedge clock)
begin
    if(reset)
    begin
        output_write_data <= 16'b0;
        output_write_addr <= 12'b0;
    end
    else
    begin
	if(~state_is_compute_init_t4 && finish)
	begin
		output_write_addr <= 12'b0;
		output_write_data <= 16'b0;
	end
	else if(~state_is_compute_init_t4)
	begin
	output_write_data <= 16'b0;
	//output_write_addr <= 12'b0;
	end
	else if(output_write_enable)
	begin
		output_write_data <= O0_1 + O1_1 + O2_1 + O3_1 + O4_1 + O5_1 + O6_1 + O7_1;
		output_write_addr <= output_write_addr + 12'b1;
	end
	else
        output_write_data <= output_write_data + O0_1 + O1_1 + O2_1 + O3_1 + O4_1 + O5_1 + O6_1 + O7_1;
	
    end
end

//OUtput Control
always@(posedge clock)
begin
    if(reset)
    begin
        
        output_write_enable <= 1'b0;
        o0 <= 1'b0; o1 <= 1'b0; o2 <= 1'b0; o3 <= 1'b0; 
    end
    else
    begin
        o0 <= write_one_output;
        o1 <= o0;
        o2 <= o1;
        o3 <= o2;
        output_write_enable <= o3;
    end

end
endmodule
