module ALU #(
    parameter DATA_W = 32
)
(
    input                       i_clk,   // clock
    input                       i_rst_n, // reset

    input                       i_valid, // input valid signal
    input [DATA_W - 1 : 0]      i_A,     // input operand A
    input [DATA_W - 1 : 0]      i_B,     // input operand B
    input [         2 : 0]      i_inst,  // instruction

    output [2*DATA_W - 1 : 0]   o_data,  // output value
    output                      o_done   // output valid signal
);
// Do not Modify the above part !!!

// Parameters
    // ======== choose your FSM style ==========
    // 1. FSM based on operation cycles
    // parameter S_IDLE           = 2'd0;
    // parameter S_ONE_CYCLE_OP   = 2'd1;
    // parameter S_MULTI_CYCLE_OP = 2'd2;
    // 2. FSM based on operation modes
    parameter S_IDLE = 4'd0;
    parameter S_ADD  = 4'd1;
    parameter S_SUB  = 4'd2;
    parameter S_AND  = 4'd3;
    parameter S_OR   = 4'd4;
    parameter S_SLT  = 4'd5;
    parameter S_SRA  = 4'd6;
    parameter S_MUL  = 4'd7;
    parameter S_DIV  = 4'd8;
    parameter S_OUT  = 4'd9;

// Wires & Regs
    // Todo
    // state
    reg  [         3: 0] state, state_nxt; // remember to expand the bit width if you want to add more states!
    // load input
    reg  [  DATA_W-1: 0] operand_a, operand_a_nxt;
    reg  [  DATA_W-1: 0] operand_b, operand_b_nxt;
    reg  [         2: 0] inst, inst_nxt;

    reg  [         4: 0] counter, counter_nxt; // counter
    reg                  done, done_nxt;    //check whether valid output data
    reg  [2*DATA_W-1: 0] shifted_reg, shifted_reg_nxt; //Shift Register
    reg  [    DATA_W: 0] alu_out;   
    //reg                  division_check;    //use to check the case of division

// Wire Assignments
    // Todo
    assign o_done = done;
    assign o_data = shifted_reg;
    
// Always Combination
    // load input
    always @(*) begin
        if (i_valid) begin
            operand_a_nxt = i_A;
            operand_b_nxt = i_B;
            inst_nxt      = i_inst;
        end
        else begin
            operand_a_nxt = operand_a;
            operand_b_nxt = operand_b;
            inst_nxt      = inst;
        end
    end
    // Todo: FSM
    always @(*) begin
        case(state)
            S_IDLE           : begin
                if(!i_valid) state_nxt = S_IDLE;
                else begin
                    case(i_inst)
                        3'd0: state_nxt = S_ADD;
                        3'd1: state_nxt = S_SUB;
                        3'd2: state_nxt = S_AND;
                        3'd3: state_nxt = S_OR;
                        3'd4: state_nxt = S_SLT;
                        3'd5: state_nxt = S_SRA;
                        3'd6: state_nxt = S_MUL;
                        3'd7: state_nxt = S_DIV;
                        default: state_nxt = S_IDLE;
                    endcase
                end
            end
            // S_ONE_CYCLE_OP   :
            // S_MULTI_CYCLE_OP :
            S_ADD            :  state_nxt = S_OUT;
            S_SUB            :  state_nxt = S_OUT;
            S_AND            :  state_nxt = S_OUT;
            S_OR             :  state_nxt = S_OUT;
            S_SLT            :  state_nxt = S_OUT;
            S_SRA            :  state_nxt = S_OUT;
            S_MUL            :  begin
                if(counter == 5'd31) state_nxt = S_OUT;
                else state_nxt = S_MUL;
            end
            S_DIV            : begin
                if(counter == 5'd31) state_nxt = S_OUT;
                else state_nxt = S_DIV;
            end
            S_OUT            :  state_nxt = S_IDLE;
            default: state_nxt = state;
        endcase
    end
    // Todo: Counter
    always @(*) begin
        if(state == S_MUL || state == S_DIV) begin
            counter_nxt = counter + 1;
        end
        else counter_nxt = 0;
    end

    // Todo: ALU output
    always @(*) begin
        case(state)
            S_ADD: begin
                alu_out = {operand_a[31], shifted_reg[31:0]} + {operand_b[31], operand_b}; 
            end
            S_SUB: begin
                alu_out = {operand_a[31], shifted_reg[31:0]} - {operand_b[31], operand_b};
            end
            S_AND: alu_out = shifted_reg[31:0] & operand_b;
            S_OR : alu_out = shifted_reg[31:0] | operand_b;
            S_SLT: begin
                alu_out = {operand_a[31], shifted_reg[31:0]} - {operand_b[31], operand_b};
            end
            S_SRA: alu_out = $signed(shifted_reg[31:0]) >>> operand_b;
            S_MUL: begin
                if(shifted_reg[0]) alu_out = shifted_reg[63:32] + operand_b;
                else alu_out = shifted_reg[63:32];
            end
            S_DIV: begin
                //division_check = (shifted_reg[63:32] >= operand_b);
                if(shifted_reg[63:32] >= operand_b) alu_out = shifted_reg[63:32] - operand_b;
                else alu_out = shifted_reg[63:32];
            end
            default: begin
                alu_out = 0;
                //division_check = 0;
            end
        endcase
    end

    //shift register
    always @(*) begin
        case(state)
            S_IDLE: begin
                if(!i_valid) shifted_reg_nxt = 0; 
                else begin
                    if(i_inst == 3'd7) begin //DIV iteration 0 
                        shifted_reg_nxt = {63'b0, i_A, 1'b0};
                    end
                    else shifted_reg_nxt = {{32{1'b0}}, i_A};
                end
            end
            S_ADD: begin
                if(alu_out[32]==0 && alu_out[31]==1) begin //positive overflow
                    shifted_reg_nxt = {{33{1'b0}}, {31{1'b1}}};
                end
                else if(alu_out[32]==1 && alu_out[31]==0) begin
                    shifted_reg_nxt = {{32{1'b0}}, 1'b1, {31{1'b0}}};
                end
                else shifted_reg_nxt = {32'b0, alu_out[31:0]};
            end
            S_SUB: begin
                if(alu_out[32]==0 && alu_out[31]==1) begin //positive overflow
                    shifted_reg_nxt = {{33{1'b0}}, {31{1'b1}}};
                end
                else if(alu_out[32]==1 && alu_out[31]==0) begin
                    shifted_reg_nxt = {{32{1'b0}}, 1'b1, {31{1'b0}}};
                end
                else shifted_reg_nxt = {32'b0, alu_out[31:0]};
            end
            S_AND: shifted_reg_nxt = {32'b0, alu_out[31:0]};
            S_OR : shifted_reg_nxt = {32'b0, alu_out[31:0]};
            S_SLT: begin
                if(alu_out[32]==1) shifted_reg_nxt = {63'b0, 1'b1};
                else shifted_reg_nxt = 0;
            end
            S_SRA: shifted_reg_nxt = {32'b0, alu_out[31:0]};
            S_MUL: shifted_reg_nxt = {alu_out, shifted_reg[31:1]};
            S_DIV: begin
                if(counter == 31) begin
                    if(shifted_reg[63:32] >= operand_b) begin
                        shifted_reg_nxt = {alu_out[31:0], shifted_reg[30:0], 1'b1};
                    end
                    else begin
                        shifted_reg_nxt = {alu_out[31:0], shifted_reg[30:0], 1'b0};
                    end
                end
                else begin
                    if(shifted_reg[63:32] >= operand_b) begin
                        shifted_reg_nxt = {alu_out[30:0], shifted_reg[31:0], 1'b1};
                    end
                    else begin
                        shifted_reg_nxt = {alu_out[30:0], shifted_reg[31:0], 1'b0};
                    end
                end
            end
            S_OUT: shifted_reg_nxt = shifted_reg;
            default: begin
                shifted_reg_nxt = shifted_reg;
                //division_check = 0;
            end
        endcase
    end
    
    // Todo: output valid signal
    always @(*) begin
        case(state)
            S_IDLE: done_nxt = 0;
            S_ADD : done_nxt = 1;
            S_SUB : done_nxt = 1;
            S_AND : done_nxt = 1;
            S_OR  : done_nxt = 1;
            S_SLT : done_nxt = 1;
            S_SRA : done_nxt = 1;
            S_MUL : begin
                if(counter == 5'd31) done_nxt = 1;
                else done_nxt = 0;
            end
            S_DIV : begin
                if(counter == 5'd31) done_nxt = 1;
                else done_nxt = 0;
            end
            S_OUT : done_nxt = 0;
            default: done_nxt = 0;
        endcase
    end

    // Todo: Sequential always block
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            state       <= S_IDLE;
            operand_a   <= 0;
            operand_b   <= 0;
            inst        <= 0;
            counter     <= 0;
            done        <= 0;
            shifted_reg <= 0;
        end
        else begin
            state       <= state_nxt;
            operand_a   <= operand_a_nxt;
            operand_b   <= operand_b_nxt;
            inst        <= inst_nxt;
            counter     <= counter_nxt;
            done        <= done_nxt;
            shifted_reg <= shifted_reg_nxt;
        end
    end

endmodule