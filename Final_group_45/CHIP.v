//----------------------------- DO NOT MODIFY THE I/O INTERFACE!! ------------------------------//
module CHIP #(                                                                                  //
    parameter BIT_W = 32                                                                        //
)(                                                                                              //
    // clock                                                                                    //
        input               i_clk,                                                              //
        input               i_rst_n,                                                            //
    // instruction memory                                                                       //
        input  [BIT_W-1:0]  i_IMEM_data,                                                        //
        output [BIT_W-1:0]  o_IMEM_addr,                                                        //
        output              o_IMEM_cen,                                                         //
    // data memory                                                                              //
        input               i_DMEM_stall,                                                       //
        input  [BIT_W-1:0]  i_DMEM_rdata,                                                       //
        output              o_DMEM_cen,                                                         //
        output              o_DMEM_wen,                                                         //
        output [BIT_W-1:0]  o_DMEM_addr,                                                        //
        output [BIT_W-1:0]  o_DMEM_wdata,                                                       //
    // finnish procedure                                                                        //
        output              o_finish,                                                           //
    // cache                                                                                    //
        input               i_cache_finish,                                                     //
        output              o_proc_finish                                                       //
);                                                                                              //
//----------------------------- DO NOT MODIFY THE I/O INTERFACE!! ------------------------------//

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Parameters
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // opcode
    // R-type
    // add, sub, and, xor, mul
    localparam R_type = 7'b0110011;
    // I-type
    localparam JALR  = 7'b1100111;
    localparam ADDI  = 7'b0010011;
    localparam SLLI  = 7'b0010011;
    localparam SLTI  = 7'b0010011;
    localparam SRAI  = 7'b0010011;
    localparam LW    = 7'b0000011;
    localparam ECALL = 7'b1110011;
    // B-type
    localparam BEQ   = 7'b1100011;
    localparam BGE   = 7'b1100011;
    localparam BLT   = 7'b1100011;
    localparam BNE   = 7'b1100011;
    // S-type
    localparam SW    = 7'b0100011;
    // J-type
    localparam JAL   = 7'b1101111;
    // U-type
    localparam AUIPC = 7'b0010111;

    // funct3
    localparam JALR_FUNC3 = 3'b000;
    localparam ADD_FUNC3  = 3'b000;
    localparam SUB_FUNC3  = 3'b000;
    localparam AND_FUNC3  = 3'b111;
    localparam XOR_FUNC3  = 3'b100;
    localparam ADDI_FUNC3 = 3'b000;
    localparam SLLI_FUNC3 = 3'b001;
    localparam SLTI_FUNC3 = 3'b010;
    localparam SRAI_FUNC3 = 3'b101;
    localparam MUL_FUNC3  = 3'b000;
    localparam BEQ_FUNC3  = 3'b000;
    localparam BGE_FUNC3  = 3'b101;
    localparam BLT_FUNC3  = 3'b100;
    localparam BNE_FUNC3  = 3'b001;

    // funct7
    localparam ADD_FUNC7 = 7'b0000000;
    localparam SUB_FUNC7 = 7'b0100000;
    localparam AND_FUNC7 = 7'b0000000;
    localparam XOR_FUNC7 = 7'b0000000;
    localparam MUL_FUNC7 = 7'b0000001;

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Wires and Registers
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // TODO: any declaration
        reg [BIT_W-1:0] PC, next_PC;

        //change wire to reg
        reg  mem_cen, mem_wen; //cen for memory func, wen for read/write
        reg [BIT_W-1:0] mem_addr, mem_wdata, mem_rdata; //data memory

        wire mem_stall;

        reg [2*BIT_W-1:0] extout;
        wire [BIT_W-1:0] temp, t0, t1;
        wire [2*BIT_W-1:0] imm64;

        reg  [      4:0] rs1, rs2, rd;
        wire [BIT_W-1:0] rs1_data, rs2_data;
        reg  [BIT_W-1:0] rd_data;
        reg              RegWrite; 

        reg  [      6:0] opcode;
        reg  [     31:0] instruction;
        reg  [      2:0] func3; 
        reg  [      6:0] func7;
        reg  [BIT_W-1:0] imm32;
        reg              finish;
        reg              DMEM_cen;
        wire  [     31:0] mul_data;
        reg [1:0] state, next_state;
        reg [1:0] m_state, m_next_state;
        reg valid;
        wire ready;

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Continuous Assignment
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any wire assignment
    assign o_IMEM_addr = PC;
    assign o_DMEM_addr = mem_addr;
    assign o_DMEM_wdata = mem_wdata;
    assign o_DMEM_wen = mem_wen;
    assign o_finish = finish;
    assign o_DMEM_cen = DMEM_cen;
    wire Branch, ALUSrc;
    // wire RegWrite;
    wire [3:0] alu_ctr;
    wire zero;
    // wire [1:0] ALUOp;
    wire [31:0] alu2;
    wire B0;

    // Imm Gen
    assign temp = (i_IMEM_data[6:0] == 7'b011011) ? 32'd0 : 
                  (i_IMEM_data[6:0] == 7'b1100111 || i_IMEM_data[6:0] == 7'b0000011 || i_IMEM_data[6:0] == 7'b0010011) ? {{20{i_IMEM_data[31]}},i_IMEM_data[31:20]} :
                  (i_IMEM_data[6:0] == 7'b0100011) ? {{20{i_IMEM_data[31]}}, i_IMEM_data[31:25], i_IMEM_data[11:7]} :
                  (i_IMEM_data[6:0] == 7'b1100011) ? {{20{i_IMEM_data[31]}}, i_IMEM_data[7], i_IMEM_data[30:25], i_IMEM_data[11:8], 0} :
                  (i_IMEM_data[6:0] == 7'b0010111) ? {{12{i_IMEM_data[31]}}, i_IMEM_data[31:12]} :
                  (i_IMEM_data[6:0] == 7'b1101111 || i_IMEM_data[6:0] == 7'b1110011) ? {{12{i_IMEM_data[31]}}, i_IMEM_data[19:12], i_IMEM_data[20], i_IMEM_data[30:21]} :
                  32'd0;
    assign o_IMEM_cen = 1'b1; //set high to load instruction

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Submoddules
    MULDIV_unit mul(.rs1_data(rs1_data), .rs2_data(rs2_data), .valid(valid), .mul_data(mul_data), .ready(ready), .i_clk(i_clk));
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: Reg_file wire connection
    Reg_file reg0(               
        .i_clk  (i_clk),             
        .i_rst_n(i_rst_n),         
        .wen    (RegWrite),   // 0:read | 1:write   
        .rs1    (rs1),                
        .rs2    (rs2),                
        .rd     (rd),                 
        .wdata  (rd_data),             
        .rdata1 (rs1_data),           
        .rdata2 (rs2_data)
    );

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Always Blocks
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // Todo: any combinational/sequential circuit
    always @(*) begin
        instruction = i_IMEM_data;
        opcode = instruction[6:0];
        func3 = instruction[14:12];
        func7 = instruction[31:25];
        rs1 = instruction[19:15];
        rs2 = instruction[24:20];
        rd = instruction[11:7];
        rd_data = 0;
        imm32 = 0;
        mem_addr = 0;
        mem_wdata = 0;
        mem_wen = 0;
        RegWrite = 1'd0;
        next_state = 2'd0;
        DMEM_cen = 1'd0;
        finish = 1'b0;
        next_PC = PC + 4;
        valid = 0;
        m_next_state = 2'd0;
        case(opcode)
            ECALL: finish = 1'b1;
            AUIPC: begin
                RegWrite = 1'b1;
                imm32[31:12] = instruction[31:12];
                rd_data = PC + imm32;
                next_PC = PC + 3'd4;
            end
            JAL: begin
                RegWrite = 1'b1;
                imm32[20:0] = {instruction[31], instruction[19:12], instruction[20], instruction[30:21], 1'b0};
                next_PC = $signed({1'b0, PC}) + $signed(imm32[20:0]);
                rd_data = PC + 3'd4;
            end
            JALR: begin
                RegWrite = 1'b1;
                imm32[11:0] = instruction[31:20];
                next_PC = $signed({1'b0, rs1_data}) + $signed(imm32[11:0]);
                rd_data = PC + 3'd4;
            end
            7'b0010011: begin //ADDI, SLLI, SLTI, SRAI
                RegWrite = 1'b1;
                imm32[11:0] = instruction[31:20];
                case(func3)
                    ADDI_FUNC3: rd_data = $signed(rs1_data) + $signed(imm32[11:0]);
                    SLLI_FUNC3: rd_data = rs1_data << rs2;
                    SLTI_FUNC3: begin
                        if($signed(rs1_data) < $signed(imm32[11:0])) rd_data = 32'd1;
                        else                                         rd_data = 32'd0;
                    end
                    SRAI_FUNC3: rd_data = $signed(rs1_data) >>> rs2;
                endcase
                next_PC = PC + 4;
            end
            LW: begin
                mem_wen = 1'b0; //low for read
                RegWrite = 1'b1;
                imm32[11:0] = instruction[31:20];
                mem_addr = rs1_data + imm32[11:0];
                rd_data = i_DMEM_rdata;
                case (state) 
                    2'd0 : begin
                        next_state = 2'd1;
                        DMEM_cen = 1;
                        next_PC = PC;
                    end
                    2'd1 : begin
                        next_state = i_DMEM_stall ? 2'd1 : 2'd0;
                        DMEM_cen = 0;
                        next_PC = i_DMEM_stall ? PC : PC + 3'd4;
                    end
                    // 2'd1 : begin
                    //     next_state = 2'd2;
                    //     DMEM_cen = 0;
                    //     next_PC = PC;
                    // end
                    // 2'd2 : begin
                    //     next_state = i_DMEM_stall ? 2'd2 : 2'd0;
                    //     DMEM_cen = 0;
                    //     next_PC = i_DMEM_stall ? PC : PC + 3'd4;
                    // end
                endcase
            end
            SW: begin
                mem_wen = 1'b1; //high for write
                RegWrite = 1'b0;
                imm32[4:0] = instruction[11:7];
                imm32[11:5] = instruction[31:25];
                mem_addr = rs1_data + imm32[11:0];
                mem_wdata = rs2_data;
                case (state) 
                    2'd0 : begin
                        next_state = 2'd1;
                        DMEM_cen = 1;
                        next_PC = PC;
                    end
                    2'd1 : begin
                        next_state = i_DMEM_stall ? 2'd1 : 2'd0;
                        DMEM_cen = 0;
                        next_PC = i_DMEM_stall ? PC : PC + 3'd4;
                    end
                    // 2'd1 : begin
                    //     next_state = 2'd2;
                    //     DMEM_cen = 0;
                    //     next_PC = PC;
                    // end
                    // 2'd2 : begin
                    //     next_state = i_DMEM_stall ? 2'd2 : 2'd0;
                    //     DMEM_cen = 0;
                    //     next_PC = i_DMEM_stall ? PC : PC + 3'd4;
                    // end
                endcase
            end
            7'b1100011: begin //B-type
                imm32[12:0] = {instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};
                case(func3)
                    BEQ_FUNC3: begin
                        RegWrite = 1'b0;
                        if(rs1_data == rs2_data) next_PC = $signed({1'b0, PC}) + $signed(imm32[12:0]);
                        else                     next_PC = PC + 3'd4;
                    end
                    BGE_FUNC3: begin
                        RegWrite = 1'b0;
                        if($signed(rs1_data) >= $signed(rs2_data)) next_PC = $signed({1'b0, PC}) + $signed(imm32[12:0]);
                        else                     next_PC = PC + 3'd4;
                    end
                    BLT_FUNC3: begin
                        RegWrite = 1'b0;
                        if($signed(rs1_data) < $signed(rs2_data)) next_PC = $signed({1'b0, PC}) + $signed(imm32[12:0]);
                        else                     next_PC = PC + 3'd4;
                    end
                    BNE_FUNC3: begin
                        RegWrite = 1'b0;
                        if(rs1_data != rs2_data) next_PC = $signed({1'b0, PC}) + $signed(imm32[12:0]);
                        else                     next_PC = PC + 3'd4;
                    end
                endcase
            end
            R_type: begin
                case(func3)
                    3'b000: begin
                        case(func7)
                            ADD_FUNC7: begin
                                RegWrite = 1'b1;
                                rd_data = rs1_data + rs2_data;
                                next_PC = PC + 3'd4;
                            end
                            SUB_FUNC7: begin
                                RegWrite = 1'b1;
                                rd_data = rs1_data - rs2_data;
                                next_PC = PC + 3'd4;
                            end
                            MUL_FUNC7: begin
                                valid = (m_state == 2'd0) ? 1'b1 : 1'b0;
                                rd_data = mul_data;
                                case (m_state) 
                                    2'd0 : begin
                                        m_next_state = 2'd1;
                                        valid = 1;
                                        RegWrite = 0;
                                        next_PC = PC;
                                    end
                                    2'd1 : begin
                                        m_next_state = ready ? 2'd0 : 2'd1;
                                        valid = 0;
                                        RegWrite = ready ? 1 : 0;
                                        next_PC = ready ? PC + 4 : PC;
                                    end
                                endcase
                            end
                        endcase
                    end
                    AND_FUNC3: begin
                        rd_data = rs1_data & rs2_data;
                        next_PC = PC + 3'd4;
                    end
                    XOR_FUNC3: begin
                        rd_data = rs1_data ^ rs2_data;
                        next_PC = PC + 3'd4;
                    end
                endcase
            end
        endcase
    end

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
            state <= 2'd0;
            m_state <= 2'd0;
        end
        else begin
            PC <= next_PC;
            state <= next_state;
            m_state <= m_next_state;
        end
    end
endmodule

module Reg_file(i_clk, i_rst_n, wen, rs1, rs2, rd, wdata, rdata1, rdata2);
   
    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth
    
    input i_clk, i_rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] wdata;
    input [addr_width-1:0] rs1, rs2, rd;

    output [BITS-1:0] rdata1, rdata2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign rdata1 = mem[rs1];
    assign rdata2 = mem[rs2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (rd == i)) ? wdata : mem[i];
    end

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end       
    end
endmodule

module MULDIV_unit(rs1_data, rs2_data, valid, mul_data, ready, i_clk
    // TODO: port declaration
    );
    input i_clk;
    input [31:0] rs1_data, rs2_data;
    input valid;
    output reg [31:0] mul_data;
    output ready;

    reg [1:0] mul_state, mul_next_state;
    reg [4:0] counter;
    reg [63:0] product;
    reg [32:0] partial_sum;

    wire [31:0] rs2_mul;

    assign ready = (mul_state == 2'd2);
    assign rs2_mul = rs2_data & {32{product[0]}};

    always @(*) begin
        case(mul_state)
            2'd0: mul_next_state = (valid == 0) ? 2'd0 : 2'd1;
            2'd1: mul_next_state = (counter == 0) ? 2'd2 : 2'd1;
            2'd2: mul_next_state = 2'd0;
            default : mul_next_state = 0;
        endcase
        mul_data = product[31:0];
    end

    always @(posedge i_clk) begin
        if (mul_state == 2'd0 & counter == 4'd0) begin
            product = 64'd0;
            product[31:0] = product[31:0] + rs1_data;
        end
        else begin
            partial_sum = product[63:32] + rs2_mul;
            product = product >> 1;
            product[63:31] = partial_sum[32:0];
        end
    end

    always @(posedge i_clk) begin
        mul_state <= mul_next_state;
    end

    always @(negedge i_clk) begin
        if (mul_state == 2'd1) begin
            if (counter == 5'd31) // If counter reaches 31, reset to 0
                counter <= 5'd0;
            else
                counter <= counter + 1; // Increment counter
        end
        else begin
            counter <= 5'd0; // Reset counter for other states
        end
    end   
endmodule

module Cache#(
        parameter BIT_W = 32,
        parameter ADDR_W = 32
    )(
        input i_clk,
        input i_rst_n,
        // processor interface
            input i_proc_cen,
            input i_proc_wen,
            input [ADDR_W-1:0] i_proc_addr,
            input [BIT_W-1:0]  i_proc_wdata,
            output [BIT_W-1:0] o_proc_rdata,
            output o_proc_stall,
            input i_proc_finish,
            output o_cache_finish,
        // memory interface
            output o_mem_cen,
            output o_mem_wen,
            output [ADDR_W-1:0] o_mem_addr,
            output [BIT_W*4-1:0]  o_mem_wdata,
            input [BIT_W*4-1:0] i_mem_rdata,
            input i_mem_stall,
            output o_cache_available,
        // others
        input  [ADDR_W-1: 0] i_offset
    );

    integer i;

    reg [152:0] cache[0:15], cache_nxt[0:15];
    reg cen, wen;
    reg [31:0] addr;
    reg [127:0] wdata;
    reg [1:0] state, next_state;
    reg [1:0] hit_state, hit_next_state;
    reg proc_stall;
    reg mem_cen, mem_wen;
    reg [127:0] write_through, cache_update;

    wire [31:0] addr_mem;
    wire [127:0] data;
    wire [1:0] word_offset;
    wire valid, hit;
    wire [23:0] tag;
    wire [3:0] index;

    // assign addr_mem = (addr >= 32'hbffffff0 - 32*4) ? addr - 32'hbffffff0 + 32*4 : addr - i_offset;
    assign addr_mem = addr + 16 - i_offset[3:0];
    assign valid = cache[addr_mem[7:4]][152];
    assign tag = addr_mem[31:8];
    assign index = addr_mem[7:4];
    assign hit = valid & (tag == cache[index][151:128]);
    assign word_offset = addr_mem[3:2];
    assign o_proc_rdata = hit ? cache[index][32*(word_offset)+:32] : i_mem_stall ? cache[index][32*(word_offset)+:32] : i_mem_rdata[32*(word_offset)+:32];
    assign o_proc_stall = proc_stall;
    assign o_mem_cen = mem_cen;
    assign o_mem_wen = i_proc_wen;
    // assign o_mem_addr = i_proc_addr;
    // assign o_mem_addr = {i_proc_addr[31:4], i_offset[3:0]};
    assign o_mem_addr = (i_proc_addr >= i_offset + 15*16) ? i_proc_addr :
                        (i_proc_addr >= i_offset + 15*16) ? i_offset + 15*16 :
                        (i_proc_addr >= i_offset + 14*16) ? i_offset + 14*16 :
                        (i_proc_addr >= i_offset + 13*16) ? i_offset + 13*16 :
                        (i_proc_addr >= i_offset + 12*16) ? i_offset + 12*16 :
                        (i_proc_addr >= i_offset + 11*16) ? i_offset + 11*16 :
                        (i_proc_addr >= i_offset + 10*16) ? i_offset + 10*16 :
                        (i_proc_addr >= i_offset + 9*16) ? i_offset + 9*16 :
                        (i_proc_addr >= i_offset + 8*16) ? i_offset + 8*16 :
                        (i_proc_addr >= i_offset + 7*16) ? i_offset + 7*16 :
                        (i_proc_addr >= i_offset + 6*16) ? i_offset + 6*16 :
                        (i_proc_addr >= i_offset + 5*16) ? i_offset + 5*16 :
                        (i_proc_addr >= i_offset + 4*16) ? i_offset + 4*16 :
                        (i_proc_addr >= i_offset + 3*16) ? i_offset + 3*16 :
                        (i_proc_addr >= i_offset + 2*16) ? i_offset + 2*16 :
                        (i_proc_addr >= i_offset + 1*16) ? i_offset + 1*16 : i_offset;


    

    assign o_mem_wdata = (word_offset == 2'd0) ? {cache[index][127:32], i_proc_wdata} :
                         (word_offset == 2'd1) ? {cache[index][127:64], i_proc_wdata, cache[index][31:0]} :
                         (word_offset == 2'd2) ? {cache[index][127:96], i_proc_wdata, cache[index][63:0]} :
                         {i_proc_wdata, cache[index][95:0]};
    

    always @(*) begin
        mem_cen = 0;
        proc_stall = 0;
        next_state = 0;
        hit_next_state = 0;
        write_through = (word_offset == 2'd0) ? {cache[index][127:32], i_proc_wdata} :
                        (word_offset == 2'd1) ? {cache[index][127:64], i_proc_wdata, cache[index][31:0]} :
                        (word_offset == 2'd2) ? {cache[index][127:96], i_proc_wdata, cache[index][63:0]} :
                        {i_proc_wdata, cache[index][95:0]};
        cache_update  = (word_offset == 2'd0) ? i_mem_rdata :
                        (word_offset == 2'd1) ? {i_mem_rdata[95:0], cache[index][31:0]} :
                        (word_offset == 2'd2) ? {i_mem_rdata[63:0], cache[index][63:0]} :
                        {i_mem_rdata[31:0], cache[index][95:0]};

        for (i=0; i<16; i=i+1) begin
            cache_nxt[i] = cache[i];
        end
        if (cen) begin
            if (hit & (hit_state == 0) & !wen) begin
                        mem_cen = 0;
                        proc_stall = 0;
            end
            else begin
                proc_stall = i_mem_stall;
                case (state) 
                    2'd0 : begin
                        next_state = 2'd1;
                        mem_cen = 1;
                        mem_wen = i_proc_wen;
                        hit_next_state = 1;
                        // if (wen) begin 
                        //     cache_nxt[index] = {1'b1,i_proc_addr[31:8],cache[index][127:32], i_proc_wdata};
                        // end
                    end
                    2'd1 : begin
                        next_state = i_mem_stall ? 2'd1 : 2'd0;
                        mem_cen = 0;
                        mem_wen = i_proc_wen;
                        hit_next_state = i_mem_stall ? 1 : 0;
                        if (!wen) begin
                            cache_nxt[index] = i_mem_stall ? cache[index] : {1'b1,tag,cache_update};
                        end
                        else begin 
                            cache_nxt[index] = i_mem_stall ? cache[index] : {1'b1,tag,write_through};
                        end
                    end
                endcase
            end
        end
    end

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            cen <= 0;
            wen <= 0;
            addr <= 0;
            wdata <= 0;
            state <= 0;
            hit_state <= 0;
        end
        else
            if (i_proc_cen) begin
                cen <= i_proc_cen;
                wen <= i_proc_wen;
                addr <= i_proc_addr;
                wdata <= i_proc_wdata;
                state <= next_state;
                hit_state <= hit_next_state;      
            end
            else if (cen & (i_mem_stall == 0)) begin
                cen <= 0;
                wen <= 0;
                addr <= 0;
                wdata <= 0;
                state <= 0;
                hit_state <= 0;
            end
            else if (cen) begin
                state <= next_state;
                hit_state <= hit_next_state;
            end
    end

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            for (i=0; i<16; i=i+1) begin
                cache[i] <= 0;
            end
        end
        else begin
            for (i=0; i<16; i=i+1) begin
                cache[i] <= cache_nxt[i];
            end
        end
    end

    assign o_cache_available = 1; // change this value to 1 if the cache is implemented

    //------------------------------------------//
    //          default connection              //
    // assign o_mem_cen = i_proc_cen;              //
    // assign o_mem_wen = i_proc_wen;              //
    // assign o_mem_addr = i_proc_addr;            //
    // assign o_mem_wdata = i_proc_wdata;          //
    // assign o_proc_rdata = i_mem_rdata[0+:BIT_W];//
    // assign o_proc_stall = i_mem_stall;          //
    //------------------------------------------//

    // Todo: BONUS

endmodule