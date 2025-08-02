`timescale 1ns / 1ps

module CPUSystem(
    input Clock,
    input Reset,
    output reg [11:0] T,
    output reg T_Reset,
    output wire [5:0] Opcode,
    output wire [1:0] RegSel,
    output wire [7:0] Address,
    output wire [2:0] DestReg,
    output wire [2:0] SrcReg1,
    output wire [2:0] SrcReg2
);

    // Internal signals for connecting to ALUSys
    reg [2:0] RF_OutASel;
    reg [2:0] RF_OutBSel;
    reg [2:0] RF_FunSel;
    reg [3:0] RF_RegSel;
    reg [3:0] RF_ScrSel;
    reg [4:0] ALU_FunSel;
    reg ALU_WF;
    reg [1:0] ARF_OutCSel;
    reg [1:0] ARF_OutDSel;
    reg [1:0] ARF_FunSel;
    reg [2:0] ARF_RegSel;
    reg IR_LH;
    reg IR_Write;
    reg Mem_WR;
    reg Mem_CS;
    reg [1:0] MuxASel;
    reg [1:0] MuxBSel;
    reg [1:0] MuxCSel;
    reg MuxDSel;
    reg DR_E;
    reg [1:0] DR_FunSel;

    // Instantiate the ArithmeticLogicUnitSystem
    ArithmeticLogicUnitSystem ALUSys(
        .RF_OutASel(RF_OutASel),
        .RF_OutBSel(RF_OutBSel),
        .RF_FunSel(RF_FunSel),
        .RF_RegSel(RF_RegSel),
        .RF_ScrSel(RF_ScrSel),
        .ALU_FunSel(ALU_FunSel),
        .ALU_WF(ALU_WF),
        .ARF_OutCSel(ARF_OutCSel),
        .ARF_OutDSel(ARF_OutDSel),
        .ARF_FunSel(ARF_FunSel),
        .ARF_RegSel(ARF_RegSel),
        .IR_LH(IR_LH),
        .IR_Write(IR_Write),
        .Mem_WR(Mem_WR),
        .Mem_CS(Mem_CS),
        .MuxASel(MuxASel),
        .MuxBSel(MuxBSel),
        .MuxCSel(MuxCSel),
        .MuxDSel(MuxDSel),
        .DR_E(DR_E),
        .DR_FunSel(DR_FunSel),
        .Clock(Clock),
        .Reset(Reset)
    );

    // Control Unit States
    parameter S_FETCH_LSB = 0,
              S_FETCH_MSB = 1,
              S_EXECUTE = 2,
              S_EXECUTE2 = 3,
              S_EXECUTE3 = 4,
              S_EXECUTE4 = 5;

    reg [2:0] current_state;
    reg [2:0] next_state;

    // Instruction Register Output
    wire [15:0] instruction = ALUSys.IR.IROut;

    // Assign outputs for simulation    
    assign Opcode = instruction[15:10];
    assign RegSel = instruction[9:8];
    assign Address = instruction[7:0];
    assign DestReg = instruction[9:7];
    assign SrcReg1 = instruction[6:4];
    assign SrcReg2 = instruction[3:1];

    always @(posedge Clock or negedge Reset) begin
        if (!Reset) begin
            T <= 12'b0;
            T_Reset <= 1'b1;
            current_state <= S_FETCH_LSB;
            next_state <= S_FETCH_LSB;
            
            // Reset all control signals
            RF_OutASel <= 3'b0;
            RF_OutBSel <= 3'b0;
            RF_FunSel <= 3'b0;
            RF_RegSel <= 4'b0;
            RF_ScrSel <= 4'b0;
            ALU_FunSel <= 5'b0;
            ALU_WF <= 1'b0;
            ARF_OutCSel <= 2'b0;
            ARF_OutDSel <= 2'b0;
            ARF_FunSel <= 2'b0;
            ARF_RegSel <= 3'b0;
            IR_LH <= 1'b0;
            IR_Write <= 1'b0;
            Mem_WR <= 1'b0;
            Mem_CS <= 1'b1;
            MuxASel <= 2'b0;
            MuxBSel <= 2'b0;
            MuxCSel <= 2'b0;
            MuxDSel <= 1'b0;
            DR_E <= 1'b0;
            DR_FunSel <= 2'b0;
        end else begin
            if (T_Reset) begin
                T <= 12'b0;
                T_Reset <= 1'b0;
            end else begin
                T <= T + 1;
            end
            
            current_state <= next_state;
            
            // Default assignments
            RF_OutASel = 3'b0;
            RF_OutBSel = 3'b0;
            RF_FunSel = 3'b0;
            RF_RegSel = 4'b0;
            RF_ScrSel = 4'b0;
            ALU_FunSel = 5'b0;
            ALU_WF = 1'b0;
            ARF_OutCSel = 2'b0;
            ARF_OutDSel = 2'b0;
            ARF_FunSel = 2'b0;
            ARF_RegSel = 3'b0;
            IR_LH = 1'b0;
            IR_Write = 1'b0;
            Mem_WR = 1'b0;
            Mem_CS = 1'b1;
            MuxASel = 2'b0;
            MuxBSel = 2'b0;
            MuxCSel = 2'b0;
            MuxDSel = 1'b0;
            DR_E = 1'b0;
            DR_FunSel = 2'b0;

            case (current_state)
                S_FETCH_LSB: begin
                    // Fetch LSB of instruction from PC address
                    ARF_OutDSel = 2'b00; // Select PC for address
                    Mem_CS = 1'b0; // Enable memory
                    Mem_WR = 1'b0; // Read
                    IR_Write = 1'b1; // Enable IR write
                    IR_LH = 1'b0; // Write to LSB
                    next_state = S_FETCH_MSB;
                end
                
                S_FETCH_MSB: begin
                    // Increment PC and fetch MSB
                    ARF_RegSel = 3'b100; // Select PC
                    ARF_FunSel = 2'b01; // Increment PC
                    ARF_OutDSel = 2'b00; // Select PC for address
                    Mem_CS = 1'b0; // Enable memory
                    Mem_WR = 1'b0; // Read
                    IR_Write = 1'b1; // Enable IR write
                    IR_LH = 1'b1; // Write to MSB
                    next_state = S_EXECUTE;
                end
                
                S_EXECUTE: begin
                    // Increment PC for next instruction (default behavior)
                    ARF_RegSel = 3'b100; // Select PC
                    ARF_FunSel = 2'b01; // Increment PC
                    next_state = S_FETCH_LSB; // Default next state
                    
                    // Execute based on opcode
                    case (Opcode)
                        6'h00: begin // BRA PC <- VALUE
                            ARF_RegSel = 3'b100; // Select PC
                            ARF_FunSel = 2'b10; // Load PC
                            MuxBSel = 2'b11; // Select IR address field
                        end
                        
                        6'h01: begin // BNE IF Z=0 THEN PC <- VALUE
                            if (ALUSys.ALU.FlagsOut[3] == 1'b0) begin // Z=0
                                ARF_RegSel = 3'b100; // Select PC
                                ARF_FunSel = 2'b10; // Load PC
                                MuxBSel = 2'b11; // Select IR address field
                            end
                        end
                        
                        6'h02: begin // BEQ IF Z=1 THEN PC <- VALUE
                            if (ALUSys.ALU.FlagsOut[3] == 1'b1) begin // Z=1
                                ARF_RegSel = 3'b100; // Select PC
                                ARF_FunSel = 2'b10; // Load PC
                                MuxBSel = 2'b11; // Select IR address field
                            end
                        end
                        
                        6'h07: begin // CALL M[SP] <- PC, SP <- SP – 1, PC <- VALUE
                            // First cycle: Store PC to memory at SP
                            ARF_OutDSel = 2'b01; // Select SP for address
                            Mem_CS = 1'b0; // Enable memory
                            Mem_WR = 1'b1; // Write
                            ARF_OutCSel = 2'b00; // PC for data
                            MuxCSel = 2'b01; // PC lower byte
                            next_state = S_EXECUTE2;
                        end
                        
                        6'h08: begin // RET SP <- SP + 1, PC <- M[SP]
                            // First cycle: Increment SP
                            ARF_RegSel = 3'b010; // Select SP
                            ARF_FunSel = 2'b01; // Increment SP
                            next_state = S_EXECUTE2;
                        end
                        
                        6'h09: begin // INC DSTREG <- SREG1 + 1
                            case (DestReg)
                                3'b000: begin // PC
                                    ARF_RegSel = 3'b100;
                                    ARF_FunSel = 2'b01; // Increment
                                end
                                3'b001: begin // SP
                                    ARF_RegSel = 3'b010;
                                    ARF_FunSel = 2'b01; // Increment
                                end
                                3'b010, 3'b011: begin // AR
                                    ARF_RegSel = 3'b001;
                                    ARF_FunSel = 2'b01; // Increment
                                end
                                3'b100: begin // R1
                                    RF_RegSel = 4'b1000;
                                    RF_FunSel = 3'b001; // Increment
                                end
                                3'b101: begin // R2
                                    RF_RegSel = 4'b0100;
                                    RF_FunSel = 3'b001; // Increment
                                end
                                3'b110: begin // R3
                                    RF_RegSel = 4'b0010;
                                    RF_FunSel = 3'b001; // Increment
                                end
                                3'b111: begin // R4
                                    RF_RegSel = 4'b0001;
                                    RF_FunSel = 3'b001; // Increment
                                end
                            endcase
                            ALU_WF = 1'b1; // Update flags
                        end
                        
                        6'h0A: begin // DEC DSTREG <- SREG1 - 1
                            case (DestReg)
                                3'b000: begin // PC
                                    ARF_RegSel = 3'b100;
                                    ARF_FunSel = 2'b00; // Decrement
                                end
                                3'b001: begin // SP
                                    ARF_RegSel = 3'b010;
                                    ARF_FunSel = 2'b00; // Decrement
                                end
                                3'b010, 3'b011: begin // AR
                                    ARF_RegSel = 3'b001;
                                    ARF_FunSel = 2'b00; // Decrement
                                end
                                3'b100: begin // R1
                                    RF_RegSel = 4'b1000;
                                    RF_FunSel = 3'b000; // Decrement
                                end
                                3'b101: begin // R2
                                    RF_RegSel = 4'b0100;
                                    RF_FunSel = 3'b000; // Decrement
                                end
                                3'b110: begin // R3
                                    RF_RegSel = 4'b0010;
                                    RF_FunSel = 3'b000; // Decrement
                                end
                                3'b111: begin // R4
                                    RF_RegSel = 4'b0001;
                                    RF_FunSel = 3'b000; // Decrement
                                end
                            endcase
                            ALU_WF = 1'b1; // Update flags
                        end
                        
                        6'h0B: begin // LSL DSTREG <- LSL SREG1
                            RF_OutASel = SrcReg1;
                            ALU_FunSel = 5'b11011; // 32-bit LSL
                            ALU_WF = 1'b1;
                            case (DestReg)
                                3'b100: begin RF_RegSel = 4'b1000; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b101: begin RF_RegSel = 4'b0100; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b110: begin RF_RegSel = 4'b0010; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b111: begin RF_RegSel = 4'b0001; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                            endcase
                        end
                        
                        6'h0C: begin // LSR DSTREG <- LSR SREG1
                            RF_OutASel = SrcReg1;
                            ALU_FunSel = 5'b11100; // 32-bit LSR
                            ALU_WF = 1'b1;
                            case (DestReg)
                                3'b100: begin RF_RegSel = 4'b1000; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b101: begin RF_RegSel = 4'b0100; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b110: begin RF_RegSel = 4'b0010; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b111: begin RF_RegSel = 4'b0001; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                            endcase
                        end
                        
                        6'h0D: begin // ASR DSTREG <- ASR SREG1
                            RF_OutASel = SrcReg1;
                            ALU_FunSel = 5'b11101; // 32-bit ASR
                            ALU_WF = 1'b1;
                            case (DestReg)
                                3'b100: begin RF_RegSel = 4'b1000; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b101: begin RF_RegSel = 4'b0100; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b110: begin RF_RegSel = 4'b0010; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b111: begin RF_RegSel = 4'b0001; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                            endcase
                        end
                        
                        6'h0E: begin // CSL DSTREG <- CSL SREG1
                            RF_OutASel = SrcReg1;
                            ALU_FunSel = 5'b11110; // 32-bit CSL
                            ALU_WF = 1'b1;
                            case (DestReg)
                                3'b100: begin RF_RegSel = 4'b1000; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b101: begin RF_RegSel = 4'b0100; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b110: begin RF_RegSel = 4'b0010; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b111: begin RF_RegSel = 4'b0001; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                            endcase
                        end
                        
                        6'h0F: begin // CSR DSTREG <- CSR SREG1
                            RF_OutASel = SrcReg1;
                            ALU_FunSel = 5'b11111; // 32-bit CSR
                            ALU_WF = 1'b1;
                            case (DestReg)
                                3'b100: begin RF_RegSel = 4'b1000; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b101: begin RF_RegSel = 4'b0100; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b110: begin RF_RegSel = 4'b0010; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b111: begin RF_RegSel = 4'b0001; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                            endcase
                        end
                        
                        6'h10: begin // NOT DSTREG <- NOT SREG1
                            RF_OutASel = SrcReg1;
                            ALU_FunSel = 5'b10010; // 32-bit NOT A
                            ALU_WF = 1'b1;
                            case (DestReg)
                                3'b100: begin RF_RegSel = 4'b1000; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b101: begin RF_RegSel = 4'b0100; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b110: begin RF_RegSel = 4'b0010; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b111: begin RF_RegSel = 4'b0001; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                            endcase
                        end
                        
                        6'h11: begin // AND DSTREG <- SREG1 AND SREG2
                            RF_OutASel = SrcReg1;
                            RF_OutBSel = SrcReg2;
                            ALU_FunSel = 5'b10111; // 32-bit AND
                            ALU_WF = 1'b1;
                            case (DestReg)
                                3'b100: begin RF_RegSel = 4'b1000; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b101: begin RF_RegSel = 4'b0100; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b110: begin RF_RegSel = 4'b0010; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b111: begin RF_RegSel = 4'b0001; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                            endcase
                        end
                        
                        6'h12: begin // ORR DSTREG <- SREG1 OR SREG2
                            RF_OutASel = SrcReg1;
                            RF_OutBSel = SrcReg2;
                            ALU_FunSel = 5'b11000; // 32-bit OR
                            ALU_WF = 1'b1;
                            case (DestReg)
                                3'b100: begin RF_RegSel = 4'b1000; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b101: begin RF_RegSel = 4'b0100; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b110: begin RF_RegSel = 4'b0010; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b111: begin RF_RegSel = 4'b0001; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                            endcase
                        end
                        
                        6'h13: begin // XOR DSTREG <- SREG1 XOR SREG2
                            RF_OutASel = SrcReg1;
                            RF_OutBSel = SrcReg2;
                            ALU_FunSel = 5'b11001; // 32-bit XOR
                            ALU_WF = 1'b1;
                            case (DestReg)
                                3'b100: begin RF_RegSel = 4'b1000; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b101: begin RF_RegSel = 4'b0100; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b110: begin RF_RegSel = 4'b0010; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b111: begin RF_RegSel = 4'b0001; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                            endcase
                        end
                        
                        6'h14: begin // NAND DSTREG <- SREG1 NAND SREG2
                            RF_OutASel = SrcReg1;
                            RF_OutBSel = SrcReg2;
                            ALU_FunSel = 5'b11010; // 32-bit NAND
                            ALU_WF = 1'b1;
                            case (DestReg)
                                3'b100: begin RF_RegSel = 4'b1000; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b101: begin RF_RegSel = 4'b0100; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b110: begin RF_RegSel = 4'b0010; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b111: begin RF_RegSel = 4'b0001; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                            endcase
                        end
                        
                        6'h15: begin // ADD DSTREG <- SREG1 + SREG2
                            RF_OutASel = SrcReg1;
                            RF_OutBSel = SrcReg2;
                            ALU_FunSel = 5'b10100; // 32-bit ADD
                            ALU_WF = 1'b1;
                            case (DestReg)
                                3'b000: begin ARF_RegSel = 3'b100; ARF_FunSel = 2'b10; MuxBSel = 2'b00; end
                                3'b001: begin ARF_RegSel = 3'b010; ARF_FunSel = 2'b10; MuxBSel = 2'b00; end
                                3'b010, 3'b011: begin ARF_RegSel = 3'b001; ARF_FunSel = 2'b10; MuxBSel = 2'b00; end
                                3'b100: begin RF_RegSel = 4'b1000; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b101: begin RF_RegSel = 4'b0100; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b110: begin RF_RegSel = 4'b0010; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b111: begin RF_RegSel = 4'b0001; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                            endcase
                        end
                        
                        6'h16: begin // ADC DSTREG <- SREG1 + SREG2 + CARRY
                            RF_OutASel = SrcReg1;
                            RF_OutBSel = SrcReg2;
                            ALU_FunSel = 5'b10101; // 32-bit ADD with carry
                            ALU_WF = 1'b1;
                            case (DestReg)
                                3'b100: begin RF_RegSel = 4'b1000; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b101: begin RF_RegSel = 4'b0100; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b110: begin RF_RegSel = 4'b0010; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b111: begin RF_RegSel = 4'b0001; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                            endcase
                        end
                        
                        6'h17: begin // SUB DSTREG <- SREG1 - SREG2
                            RF_OutASel = SrcReg1;
                            RF_OutBSel = SrcReg2;
                            ALU_FunSel = 5'b10110; // 32-bit SUB
                            ALU_WF = 1'b1;
                            case (DestReg)
                                3'b100: begin RF_RegSel = 4'b1000; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b101: begin RF_RegSel = 4'b0100; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b110: begin RF_RegSel = 4'b0010; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b111: begin RF_RegSel = 4'b0001; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                            endcase
                        end
                        
                        6'h18: begin // MOV DSTREG <- SREG1
                            RF_OutASel = SrcReg1;
                            ALU_FunSel = 5'b10000; // Pass through A
                            ALU_WF = 1'b1; // Update flags for MOV
                            case (DestReg)
                                3'b000: begin ARF_RegSel = 3'b100; ARF_FunSel = 2'b10; MuxBSel = 2'b00; end
                                3'b001: begin ARF_RegSel = 3'b010; ARF_FunSel = 2'b10; MuxBSel = 2'b00; end
                                3'b010, 3'b011: begin ARF_RegSel = 3'b001; ARF_FunSel = 2'b10; MuxBSel = 2'b00; end
                                3'b100: begin RF_RegSel = 4'b1000; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b101: begin RF_RegSel = 4'b0100; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b110: begin RF_RegSel = 4'b0010; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                                3'b111: begin RF_RegSel = 4'b0001; RF_FunSel = 3'b010; MuxASel = 2'b00; end
                            endcase
                        end
                        
                        6'h19: begin // MOVL Rx[7:0] <- IMMEDIATE
                            case (RegSel)
                                2'b00: RF_RegSel = 4'b1000; // R1
                                2'b01: RF_RegSel = 4'b0100; // R2
                                2'b10: RF_RegSel = 4'b0010; // R3
                                2'b11: RF_RegSel = 4'b0001; // R4
                            endcase
                            RF_FunSel = 3'b100; // Load lower 8 bits, clear upper 24
                            MuxASel = 2'b11; // Select IR address field (immediate)
                        end
                        
                        6'h1A: begin // MOVSH Rx[31-8] <- Rx[23-0], Rx[7-0] <- IMMEDIATE
                            case (RegSel)
                                2'b00: RF_RegSel = 4'b1000; // R1
                                2'b01: RF_RegSel = 4'b0100; // R2
                                2'b10: RF_RegSel = 4'b0010; // R3
                                2'b11: RF_RegSel = 4'b0001; // R4
                            endcase
                            RF_FunSel = 3'b110; // 8-bit left shift and load lower 8 bits
                            MuxASel = 2'b11; // IR address field (immediate)
                        end
                        
                        6'h1B: begin // LDARL DSTREG <- M[AR] (16-bit)
                            // Load address into AR, then read
                            ARF_OutDSel = 2'b10; // Select AR for address
                            Mem_CS = 1'b0; // Enable memory
                            Mem_WR = 1'b0; // Read
                            case (DestReg)
                                3'b100: begin RF_RegSel = 4'b1000; RF_FunSel = 3'b101; MuxASel = 2'b10; end
                                3'b101: begin RF_RegSel = 4'b0100; RF_FunSel = 3'b101; MuxASel = 2'b10; end
                                3'b110: begin RF_RegSel = 4'b0010; RF_FunSel = 3'b101; MuxASel = 2'b10; end
                                3'b111: begin RF_RegSel = 4'b0001; RF_FunSel = 3'b101; MuxASel = 2'b10; end
                            endcase
                        end
                        
                        6'h1C: begin // LDARH DSTREG <- M[AR] (32-bit)
                            // Multi-cycle 32-bit memory read
                            ARF_OutDSel = 2'b10; // Select AR for address
                            Mem_CS = 1'b0; // Enable memory
                            Mem_WR = 1'b0; // Read
                            DR_E = 1'b1; // Enable DR
                            DR_FunSel = 2'b01; // Clear upper, load lower 8
                            next_state = S_EXECUTE2;
                        end
                        
                        6'h1D: begin // STAR M[AR] <- SREG1
                            ARF_OutDSel = 2'b10; // Select AR for address
                            Mem_CS = 1'b0; // Enable memory
                            Mem_WR = 1'b1; // Write
                            RF_OutASel = SrcReg1;
                            ALU_FunSel = 5'b10000; // Pass through A
                            MuxCSel = 2'b00; // Lower 8 bits of ALU output
                            next_state = S_EXECUTE2; // Multi-cycle for 32-bit write
                        end
                        
                        6'h1E: begin // LDAL Rx <- M[ADDRESS] (16-bit)
                            // Load address into AR first
                            ARF_RegSel = 3'b001; // Select AR
                            ARF_FunSel = 2'b10; // Load AR
                            MuxBSel = 2'b11; // IR address field
                            next_state = S_EXECUTE2;
                        end
                        
                        6'h1F: begin // LDAH Rx <- M[ADDRESS] (32-bit)
                            // Load address into AR first
                            ARF_RegSel = 3'b001; // Select AR
                            ARF_FunSel = 2'b10; // Load AR
                            MuxBSel = 2'b11; // IR address field
                            next_state = S_EXECUTE2;
                        end
                        
                        6'h20: begin // STA M[ADDRESS] <- Rx
                            // Load address into AR first
                            ARF_RegSel = 3'b001; // Select AR
                            ARF_FunSel = 2'b10; // Load AR
                            MuxBSel = 2'b11; // IR address field
                            next_state = S_EXECUTE2;
                        end
                        
                        6'h03: begin // POPL SP <- SP + 1, Rx <- M[SP] (16-bit)
                            ARF_RegSel = 3'b010; // Select SP
                            ARF_FunSel = 2'b01; // Increment SP
                            next_state = S_EXECUTE2;
                        end
                        
                        6'h04: begin // PSHL M[SP] <- Rx, SP <- SP – 1 (16-bit)
                            ARF_OutDSel = 2'b01; // Select SP for address
                            Mem_CS = 1'b0; // Enable memory
                            Mem_WR = 1'b1; // Write
                            case (RegSel)
                                2'b00: RF_OutASel = 3'b100; // R1
                                2'b01: RF_OutASel = 3'b101; // R2
                                2'b10: RF_OutASel = 3'b110; // R3
                                2'b11: RF_OutASel = 3'b111; // R4
                            endcase
                            ALU_FunSel = 5'b10000; // Pass through A
                            MuxCSel = 2'b00; // Lower 8 bits
                            next_state = S_EXECUTE2;
                        end
                        
                        6'h05: begin // POPH SP <- SP + 1, Rx <- M[SP] (32-bit)
                            ARF_RegSel = 3'b010; // Select SP
                            ARF_FunSel = 2'b01; // Increment SP
                            next_state = S_EXECUTE2;
                        end
                        
                        6'h06: begin // PSHH M[SP] <- Rx, SP <- SP – 1 (32-bit)
                            ARF_OutDSel = 2'b01; // Select SP for address
                            Mem_CS = 1'b0; // Enable memory
                            Mem_WR = 1'b1; // Write
                            case (RegSel)
                                2'b00: RF_OutASel = 3'b100; // R1
                                2'b01: RF_OutASel = 3'b101; // R2
                                2'b10: RF_OutASel = 3'b110; // R3
                                2'b11: RF_OutASel = 3'b111; // R4
                            endcase
                            ALU_FunSel = 5'b10000; // Pass through A
                            MuxCSel = 2'b00; // Lower 8 bits
                            next_state = S_EXECUTE2;
                        end
                        
                        default: begin
                            // Unknown instruction - just continue
                            next_state = S_FETCH_LSB;
                        end
                    endcase
                end
                
                S_EXECUTE2: begin
                    next_state = S_FETCH_LSB; // Default next state
                    
                    case (Opcode)
                        6'h07: begin // CALL - Second cycle: Store PC high byte and decrement SP
                            ARF_OutDSel = 2'b01; // Select SP for address
                            Mem_CS = 1'b0; // Enable memory
                            Mem_WR = 1'b1; // Write
                            ARF_OutCSel = 2'b00; // PC for data
                            MuxCSel = 2'b01; // PC upper byte
                            ARF_RegSel = 3'b010; // Select SP
                            ARF_FunSel = 2'b00; // Decrement SP
                            next_state = S_EXECUTE3;
                        end
                        
                        6'h08: begin // RET - Second cycle: Read PC from memory
                            ARF_OutDSel = 2'b01; // Select SP for address
                            Mem_CS = 1'b0; // Enable memory
                            Mem_WR = 1'b0; // Read
                            ARF_RegSel = 3'b100; // Select PC
                            ARF_FunSel = 2'b10; // Load PC
                            MuxBSel = 2'b10; // Memory output
                        end
                        
                        6'h1C: begin // LDARH - Continue 32-bit read
                            ARF_RegSel = 3'b001; // Select AR
                            ARF_FunSel = 2'b01; // Increment AR
                            ARF_OutDSel = 2'b10; // Select AR for address
                            Mem_CS = 1'b0; // Enable memory
                            Mem_WR = 1'b0; // Read
                            DR_E = 1'b1; // Enable DR
                            DR_FunSel = 2'b10; // Left shift and load
                            next_state = S_EXECUTE3;
                        end
                        
                        6'h1D: begin // STAR - Continue multi-byte write
                            ARF_RegSel = 3'b001; // Select AR
                            ARF_FunSel = 2'b01; // Increment AR
                            ARF_OutDSel = 2'b10; // Select AR for address
                            Mem_CS = 1'b0; // Enable memory
                            Mem_WR = 1'b1; // Write
                            MuxCSel = 2'b01; // Next 8 bits
                            next_state = S_EXECUTE3;
                        end
                        
                        6'h1E: begin // LDAL - Read from memory at loaded address
                            ARF_OutDSel = 2'b10; // Select AR for address
                            Mem_CS = 1'b0; // Enable memory
                            Mem_WR = 1'b0; // Read
                            case (RegSel)
                                2'b00: begin RF_RegSel = 4'b1000; RF_FunSel = 3'b101; MuxASel = 2'b10; end
                                2'b01: begin RF_RegSel = 4'b0100; RF_FunSel = 3'b101; MuxASel = 2'b10; end
                                2'b10: begin RF_RegSel = 4'b0010; RF_FunSel = 3'b101; MuxASel = 2'b10; end
                                2'b11: begin RF_RegSel = 4'b0001; RF_FunSel = 3'b101; MuxASel = 2'b10; end
                            endcase
                        end
                        
                        6'h1F: begin // LDAH - Start 32-bit read
                            ARF_OutDSel = 2'b10; // Select AR for address
                            Mem_CS = 1'b0; // Enable memory
                            Mem_WR = 1'b0; // Read
                            DR_E = 1'b1; // Enable DR
                            DR_FunSel = 2'b01; // Clear upper, load lower 8
                            next_state = S_EXECUTE3;
                        end
                        
                        6'h20: begin // STA - Write to memory at loaded address
                            ARF_OutDSel = 2'b10; // Select AR for address
                            Mem_CS = 1'b0; // Enable memory
                            Mem_WR = 1'b1; // Write
                            case (RegSel)
                                2'b00: RF_OutASel = 3'b100; // R1
                                2'b01: RF_OutASel = 3'b101; // R2
                                2'b10: RF_OutASel = 3'b110; // R3
                                2'b11: RF_OutASel = 3'b111; // R4
                            endcase
                            ALU_FunSel = 5'b10000; // Pass through A
                            MuxCSel = 2'b00; // Lower 8 bits
                            next_state = S_EXECUTE3;
                        end
                        
                        6'h03: begin // POPL - Read from memory at SP
                            ARF_OutDSel = 2'b01; // Select SP for address
                            Mem_CS = 1'b0; // Enable memory
                            Mem_WR = 1'b0; // Read
                            case (RegSel)
                                2'b00: begin RF_RegSel = 4'b1000; RF_FunSel = 3'b101; MuxASel = 2'b10; end
                                2'b01: begin RF_RegSel = 4'b0100; RF_FunSel = 3'b101; MuxASel = 2'b10; end
                                2'b10: begin RF_RegSel = 4'b0010; RF_FunSel = 3'b101; MuxASel = 2'b10; end
                                2'b11: begin RF_RegSel = 4'b0001; RF_FunSel = 3'b101; MuxASel = 2'b10; end
                            endcase
                        end
                        
                        6'h04: begin // PSHL - Continue 16-bit push, decrement SP
                            ARF_RegSel = 3'b010; // Select SP
                            ARF_FunSel = 2'b00; // Decrement SP
                            ARF_OutDSel = 2'b01; // Select SP for address
                            Mem_CS = 1'b0; // Enable memory
                            Mem_WR = 1'b1; // Write
                            MuxCSel = 2'b01; // Upper 8 bits
                        end
                        
                        6'h05: begin // POPH - Start 32-bit pop
                            ARF_OutDSel = 2'b01; // Select SP for address
                            Mem_CS = 1'b0; // Enable memory
                            Mem_WR = 1'b0; // Read
                            DR_E = 1'b1; // Enable DR
                            DR_FunSel = 2'b01; // Clear upper, load lower 8
                            next_state = S_EXECUTE3;
                        end
                        
                        6'h06: begin // PSHH - Continue 32-bit push
                            ARF_RegSel = 3'b001; // Select AR (or continue with SP logic)
                            ARF_FunSel = 2'b01; // Increment for next byte
                            ARF_OutDSel = 2'b01; // Select SP for address
                            Mem_CS = 1'b0; // Enable memory
                            Mem_WR = 1'b1; // Write
                            MuxCSel = 2'b01; // Next 8 bits
                            next_state = S_EXECUTE3;
                        end
                    endcase
                end
                
                S_EXECUTE3: begin
                    next_state = S_FETCH_LSB; // Default next state
                    
                    case (Opcode)
                        6'h07: begin // CALL - Third cycle: Load new PC
                            ARF_RegSel = 3'b100; // Select PC
                            ARF_FunSel = 2'b10; // Load PC
                            MuxBSel = 2'b11; // IR address field
                        end
                        
                        6'h1C: begin // LDARH - Continue reading bytes
                            ARF_RegSel = 3'b001; // Select AR
                            ARF_FunSel = 2'b01; // Increment AR
                            ARF_OutDSel = 2'b10; // Select AR for address
                            Mem_CS = 1'b0; // Enable memory
                            Mem_WR = 1'b0; // Read
                            DR_E = 1'b1; // Enable DR
                            DR_FunSel = 2'b10; // Left shift and load
                            next_state = S_EXECUTE4;
                        end
                        
                        6'h1D: begin // STAR - Continue writing bytes
                            ARF_RegSel = 3'b001; // Select AR
                            ARF_FunSel = 2'b01; // Increment AR
                            ARF_OutDSel = 2'b10; // Select AR for address
                            Mem_CS = 1'b0; // Enable memory
                            Mem_WR = 1'b1; // Write
                            MuxCSel = 2'b10; // Next 8 bits
                            next_state = S_EXECUTE4;
                        end
                        
                        6'h1F: begin // LDAH - Continue 32-bit read
                            ARF_RegSel = 3'b001; // Select AR
                            ARF_FunSel = 2'b01; // Increment AR
                            ARF_OutDSel = 2'b10; // Select AR for address
                            Mem_CS = 1'b0; // Enable memory
                            Mem_WR = 1'b0; // Read
                            DR_E = 1'b1; // Enable DR
                            DR_FunSel = 2'b10; // Left shift and load
                            next_state = S_EXECUTE4;
                        end
                        
                        6'h20: begin // STA - Continue writing bytes
                            ARF_RegSel = 3'b001; // Select AR
                            ARF_FunSel = 2'b01; // Increment AR
                            ARF_OutDSel = 2'b10; // Select AR for address
                            Mem_CS = 1'b0; // Enable memory
                            Mem_WR = 1'b1; // Write
                            MuxCSel = 2'b01; // Next 8 bits
                            next_state = S_EXECUTE4;
                        end
                        
                        6'h05: begin // POPH - Continue 32-bit pop
                            ARF_RegSel = 3'b010; // Select SP
                            ARF_FunSel = 2'b01; // Increment SP
                            ARF_OutDSel = 2'b01; // Select SP for address
                            Mem_CS = 1'b0; // Enable memory
                            Mem_WR = 1'b0; // Read
                            DR_E = 1'b1; // Enable DR
                            DR_FunSel = 2'b10; // Left shift and load
                            next_state = S_EXECUTE4;
                        end
                        
                        6'h06: begin // PSHH - Continue 32-bit push
                            ARF_RegSel = 3'b010; // Select SP
                            ARF_FunSel = 2'b00; // Decrement SP
                            ARF_OutDSel = 2'b01; // Select SP for address
                            Mem_CS = 1'b0; // Enable memory
                            Mem_WR = 1'b1; // Write
                            MuxCSel = 2'b10; // Next 8 bits
                            next_state = S_EXECUTE4;
                        end
                    endcase
                end
                
                S_EXECUTE4: begin
                    next_state = S_FETCH_LSB; // Always return to fetch after this
                    
                    case (Opcode)
                        6'h1C: begin // LDARH - Final cycle: transfer DR to destination
                            case (DestReg)
                                3'b100: begin RF_RegSel = 4'b1000; RF_FunSel = 3'b010; MuxASel = 2'b10; end
                                3'b101: begin RF_RegSel = 4'b0100; RF_FunSel = 3'b010; MuxASel = 2'b10; end
                                3'b110: begin RF_RegSel = 4'b0010; RF_FunSel = 3'b010; MuxASel = 2'b10; end
                                3'b111: begin RF_RegSel = 4'b0001; RF_FunSel = 3'b010; MuxASel = 2'b10; end
                            endcase
                        end
                        
                        6'h1D: begin // STAR - Final byte write
                            ARF_OutDSel = 2'b10; // Select AR for address
                            Mem_CS = 1'b0; // Enable memory
                            Mem_WR = 1'b1; // Write
                            MuxCSel = 2'b11; // Final 8 bits
                        end
                        
                        6'h1F: begin // LDAH - Final cycle: transfer DR to destination
                            case (RegSel)
                                2'b00: begin RF_RegSel = 4'b1000; RF_FunSel = 3'b010; MuxASel = 2'b10; end
                                2'b01: begin RF_RegSel = 4'b0100; RF_FunSel = 3'b010; MuxASel = 2'b10; end
                                2'b10: begin RF_RegSel = 4'b0010; RF_FunSel = 3'b010; MuxASel = 2'b10; end
                                2'b11: begin RF_RegSel = 4'b0001; RF_FunSel = 3'b010; MuxASel = 2'b10; end
                            endcase
                        end
                        
                        6'h20: begin // STA - Final byte write
                            ARF_OutDSel = 2'b10; // Select AR for address
                            Mem_CS = 1'b0; // Enable memory
                            Mem_WR = 1'b1; // Write
                            MuxCSel = 2'b11; // Final 8 bits
                        end
                        
                        6'h05: begin // POPH - Final cycle: transfer DR to destination
                            case (RegSel)
                                2'b00: begin RF_RegSel = 4'b1000; RF_FunSel = 3'b010; MuxASel = 2'b10; end
                                2'b01: begin RF_RegSel = 4'b0100; RF_FunSel = 3'b010; MuxASel = 2'b10; end
                                2'b10: begin RF_RegSel = 4'b0010; RF_FunSel = 3'b010; MuxASel = 2'b10; end
                                2'b11: begin RF_RegSel = 4'b0001; RF_FunSel = 3'b010; MuxASel = 2'b10; end
                            endcase
                        end
                        
                        6'h06: begin // PSHH - Final byte push and decrement SP
                            ARF_RegSel = 3'b010; // Select SP
                            ARF_FunSel = 2'b00; // Decrement SP
                            ARF_OutDSel = 2'b01; // Select SP for address
                            Mem_CS = 1'b0; // Enable memory
                            Mem_WR = 1'b1; // Write
                            MuxCSel = 2'b11; // Final 8 bits
                        end
                    endcase
                end
            endcase
        end
    end
endmodule