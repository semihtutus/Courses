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
    S_EXECUTE = 2;

    reg [1:0] current_state;

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
        T <= T + 1;
        T_Reset <= 1'b0;
        
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
                current_state = S_FETCH_MSB;
            end
            
            S_FETCH_MSB: begin
                // Increment PC and fetch MSB
                ARF_RegSel = 3'b100; // Select PC
                ARF_FunSel = 2'b01; // Increment PC (FIXED)
                ARF_OutDSel = 2'b00; // Select PC for address
                Mem_CS = 1'b0; // Enable memory
                Mem_WR = 1'b0; // Read
                IR_Write = 1'b1; // Enable IR write
                IR_LH = 1'b1; // Write to MSB
                current_state = S_EXECUTE;
            end
            
            S_EXECUTE: begin
                // Increment PC for next instruction
                ARF_RegSel = 3'b100; // Select PC
                ARF_FunSel = 2'b01; // Increment PC
                
                // Execute based on opcode
                case (Opcode)
                    6'h00: begin // BRA PC <- VALUE
                        ARF_RegSel = 3'b100; // Select PC
                        ARF_FunSel = 2'b10; // Load PC
                        MuxBSel = 2'b11; // Select IR address field
                        current_state = S_FETCH_LSB;
                    end
                    
                    6'h01: begin // BNE IF Z=0 THEN PC <- VALUE
                        if (ALUSys.ALU.FlagsOut[0] == 1'b0) begin // Z=0 (FIXED)
                            ARF_RegSel = 3'b100; // Select PC
                            ARF_FunSel = 2'b10; // Load PC
                            MuxBSel = 2'b11; // Select IR address field
                        end
                        current_state = S_FETCH_LSB;
                    end
                    
                    6'h02: begin // BEQ IF Z=1 THEN PC <- VALUE
                        if (ALUSys.ALU.FlagsOut[0] == 1'b1) begin // Z=1 (FIXED)
                            ARF_RegSel = 3'b100; // Select PC
                            ARF_FunSel = 2'b10; // Load PC
                            MuxBSel = 2'b11; // Select IR address field
                        end
                        current_state = S_FETCH_LSB;
                    end
                    
                    6'h09: begin // INC DSTREG <- SREG1 + 1  
                        RF_OutASel = SrcReg1;
                        RF_OutBSel = 3'b111; // Constant 1 (via S4)
                        // Choose 32-bit or 16-bit ADD based on destination
                        if (DestReg >= 3'b100) 
                            ALU_FunSel = 5'b10100; // 32-bit ADD (FIXED)
                        else 
                            ALU_FunSel = 5'b00100; // 16-bit ADD
                        ALU_WF = 1'b1;
                        case (DestReg)
                            3'b000: begin // PC
                                ARF_RegSel = 3'b100;
                                ARF_FunSel = 2'b10; // Load
                                MuxBSel = 2'b00; // ALU output
                            end
                            3'b001: begin // SP
                                ARF_RegSel = 3'b010;
                                ARF_FunSel = 2'b10;
                                MuxBSel = 2'b00;
                            end
                            3'b010: begin // AR
                                ARF_RegSel = 3'b001;
                                ARF_FunSel = 2'b10;
                                MuxBSel = 2'b00;
                            end
                            3'b011: begin // AR
                                ARF_RegSel = 3'b001;
                                ARF_FunSel = 2'b10;
                                MuxBSel = 2'b00;
                            end
                            3'b100: begin // R1
                                RF_RegSel = 4'b1000; // FIXED one-hot
                                RF_FunSel = 3'b010; // Load
                                MuxASel = 2'b00; // ALU output
                            end
                            3'b101: begin // R2
                                RF_RegSel = 4'b0100; // FIXED
                                RF_FunSel = 3'b010;
                                MuxASel = 2'b00;
                            end
                            3'b110: begin // R3
                                RF_RegSel = 4'b0010; // FIXED
                                RF_FunSel = 3'b010;
                                MuxASel = 2'b00;
                            end
                            3'b111: begin // R4
                                RF_RegSel = 4'b0001; // FIXED
                                RF_FunSel = 3'b010;
                                MuxASel = 2'b00;
                            end
                        endcase
                        current_state = S_FETCH_LSB;
                    end
                    
                    6'h0A: begin // DEC DSTREG <- SREG1 - 1
                        RF_OutASel = SrcReg1;
                        RF_OutBSel = 3'b111; // Constant 1 (via S4)
                        // Choose 32-bit or 16-bit SUB based on destination
                        if (DestReg >= 3'b100) 
                            ALU_FunSel = 5'b10110; // 32-bit SUB
                        else 
                            ALU_FunSel = 5'b00110; // 16-bit SUB
                        ALU_WF = 1'b1;
                        case (DestReg)
                            3'b000: begin // PC
                                ARF_RegSel = 3'b100;
                                ARF_FunSel = 2'b10; // Load
                                MuxBSel = 2'b00; // ALU output
                            end
                            3'b001: begin // SP
                                ARF_RegSel = 3'b010;
                                ARF_FunSel = 2'b10;
                                MuxBSel = 2'b00;
                            end
                            3'b010: begin // AR
                                ARF_RegSel = 3'b001;
                                ARF_FunSel = 2'b10;
                                MuxBSel = 2'b00;
                            end
                            3'b011: begin // AR
                                ARF_RegSel = 3'b001;
                                ARF_FunSel = 2'b10;
                                MuxBSel = 2'b00;
                            end
                            3'b100: begin // R1
                                RF_RegSel = 4'b1000; // FIXED
                                RF_FunSel = 3'b010; // Load
                                MuxASel = 2'b00; // ALU output
                            end
                            3'b101: begin // R2
                                RF_RegSel = 4'b0100; // FIXED
                                RF_FunSel = 3'b010;
                                MuxASel = 2'b00;
                            end
                            3'b110: begin // R3
                                RF_RegSel = 4'b0010; // FIXED
                                RF_FunSel = 3'b010;
                                MuxASel = 2'b00;
                            end
                            3'b111: begin // R4
                                RF_RegSel = 4'b0001; // FIXED
                                RF_FunSel = 3'b010;
                                MuxASel = 2'b00;
                            end
                        endcase
                        current_state = S_FETCH_LSB;
                    end
                    
                    6'h15: begin // ADD DSTREG <- SREG1 + SREG2
                        RF_OutASel = SrcReg1;
                        RF_OutBSel = SrcReg2;
                        ALU_FunSel = 5'b10100; // 32-bit ADD
                        ALU_WF = 1'b1;
                        case (DestReg)
                            3'b000: begin // PC
                                ARF_RegSel = 3'b100;
                                ARF_FunSel = 2'b10; // Load
                                MuxBSel = 2'b00; // ALU output
                            end
                            3'b001: begin // SP
                                ARF_RegSel = 3'b010;
                                ARF_FunSel = 2'b10;
                                MuxBSel = 2'b00;
                            end
                            3'b010: begin // AR
                                ARF_RegSel = 3'b001;
                                ARF_FunSel = 2'b10;
                                MuxBSel = 2'b00;
                            end
                            3'b011: begin // AR
                                ARF_RegSel = 3'b001;
                                ARF_FunSel = 2'b10;
                                MuxBSel = 2'b00;
                            end
                            3'b100: begin // R1
                                RF_RegSel = 4'b1000; // FIXED
                                RF_FunSel = 3'b010; // Load
                                MuxASel = 2'b00; // ALU output
                            end
                            3'b101: begin // R2
                                RF_RegSel = 4'b0100; // FIXED
                                RF_FunSel = 3'b010;
                                MuxASel = 2'b00;
                            end
                            3'b110: begin // R3
                                RF_RegSel = 4'b0010; // FIXED
                                RF_FunSel = 3'b010;
                                MuxASel = 2'b00;
                            end
                            3'b111: begin // R4
                                RF_RegSel = 4'b0001; // FIXED
                                RF_FunSel = 3'b010;
                                MuxASel = 2'b00;
                            end
                        endcase
                        current_state = S_FETCH_LSB;
                    end
                    
                    6'h18: begin // MOV DSTREG <- SREG1
                        RF_OutASel = SrcReg1;
                        ALU_FunSel = 5'b10000; // Pass through A (32-bit)
                        case (DestReg)
                            3'b000: begin // PC
                                ARF_RegSel = 3'b100;
                                ARF_FunSel = 2'b10; // Load
                                MuxBSel = 2'b00; // ALU output
                            end
                            3'b001: begin // SP
                                ARF_RegSel = 3'b010;
                                ARF_FunSel = 2'b10;
                                MuxBSel = 2'b00;
                            end
                            3'b010: begin // AR
                                ARF_RegSel = 3'b001;
                                ARF_FunSel = 2'b10;
                                MuxBSel = 2'b00;
                            end
                            3'b011: begin // AR
                                ARF_RegSel = 3'b001;
                                ARF_FunSel = 2'b10;
                                MuxBSel = 2'b00;
                            end
                            3'b100: begin // R1
                                RF_RegSel = 4'b1000; // FIXED
                                RF_FunSel = 3'b010; // Load
                                MuxASel = 2'b00; // ALU output
                            end
                            3'b101: begin // R2
                                RF_RegSel = 4'b0100; // FIXED
                                RF_FunSel = 3'b010;
                                MuxASel = 2'b00;
                            end
                            3'b110: begin // R3
                                RF_RegSel = 4'b0010; // FIXED
                                RF_FunSel = 3'b010;
                                MuxASel = 2'b00;
                            end
                            3'b111: begin // R4
                                RF_RegSel = 4'b0001; // FIXED
                                RF_FunSel = 3'b010;
                                MuxASel = 2'b00;
                            end
                        endcase
                        current_state = S_FETCH_LSB;
                    end
                    
                    6'h19: begin // MOVL Rx[7:0] <- IMMEDIATE
                        case (RegSel)
                            2'b00: RF_RegSel = 4'b1000; // R1
                            2'b01: RF_RegSel = 4'b0100; // R2
                            2'b10: RF_RegSel = 4'b0010; // R3
                            2'b11: RF_RegSel = 4'b0001; // R4
                        endcase
                        RF_FunSel = 3'b100; // Load lower 8 bits (FIXED)
                        MuxASel = 2'b11;    // Select IR immediate
                        current_state = S_FETCH_LSB;
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
                        current_state = S_FETCH_LSB;
                    end
                    
                    6'h1B: begin // LDARL DSTREG <- M[AR] (16-bit)
                        ARF_OutDSel = 2'b10; // Select AR for address
                        Mem_CS = 1'b0;
                        Mem_WR = 1'b0; // Read
                        case (DestReg)
                            3'b100: RF_RegSel = 4'b1000; // R1 (FIXED)
                            3'b101: RF_RegSel = 4'b0100; // R2
                            3'b110: RF_RegSel = 4'b0010; // R3
                            3'b111: RF_RegSel = 4'b0001; // R4
                        endcase
                        RF_FunSel = 3'b010; // Load
                        MuxASel = 2'b10; // Memory output
                        current_state = S_FETCH_LSB;
                    end
                    
                    6'h1C: begin // LDARH DSTREG <- M[AR] (32-bit)
                        ARF_OutDSel = 2'b10; // Select AR for address
                        Mem_CS = 1'b0;
                        Mem_WR = 1'b0; // Read
                        DR_E = 1'b1; // Enable DR
                        DR_FunSel = 2'b10; // Load upper bits (FIXED)
                        case (DestReg)
                            3'b100: RF_RegSel = 4'b1000; // R1 (FIXED)
                            3'b101: RF_RegSel = 4'b0100; // R2
                            3'b110: RF_RegSel = 4'b0010; // R3
                            3'b111: RF_RegSel = 4'b0001; // R4
                        endcase
                        RF_FunSel = 3'b010; // Load 32-bit
                        MuxASel = 2'b01; // DR output
                        current_state = S_FETCH_LSB;
                    end
                    
                    6'h1D: begin // STAR M[AR] <- SREG1
                        ARF_OutDSel = 2'b10; // Select AR for address
                        Mem_CS = 1'b0;
                        Mem_WR = 1'b1; // Write
                        RF_OutASel = SrcReg1;
                        MuxCSel = 2'b00; // Lower 8 bits of RF output
                        current_state = S_FETCH_LSB;
                    end
                    
                    6'h1E: begin // LDAL Rx <- M[ADDRESS] (16-bit)
                        // Load address into AR first, then read
                        ARF_RegSel = 3'b001; // Select AR
                        ARF_FunSel = 2'b01; // Load AR
                        MuxBSel = 2'b11; // IR address field
                        // Then read from memory
                        ARF_OutDSel = 2'b10; // Select AR for address
                        Mem_CS = 1'b0;
                        Mem_WR = 1'b0; // Read
                        case (RegSel)
                            2'b00: RF_RegSel = 4'b1000; // R1 (FIXED)
                            2'b01: RF_RegSel = 4'b0100; // R2
                            2'b10: RF_RegSel = 4'b0010; // R3
                            2'b11: RF_RegSel = 4'b0001; // R4
                        endcase
                        RF_FunSel = 3'b010; // Load
                        MuxASel = 2'b10; // Memory output
                        current_state = S_FETCH_LSB;
                    end
                    
                    6'h1F: begin // LDAH Rx <- M[ADDRESS] (32-bit)
                        // Load address into AR first, then read
                        ARF_RegSel = 3'b001; // Select AR
                        ARF_FunSel = 2'b10; // Load AR
                        MuxBSel = 2'b11; // IR address field
                        // Then read from memory
                        ARF_OutDSel = 2'b10; // Select AR for address
                        Mem_CS = 1'b0;
                        Mem_WR = 1'b0; // Read
                        DR_E = 1'b1; // Enable DR (FIXED)
                        DR_FunSel = 2'b10; // Load upper bits (FIXED)
                        case (RegSel)
                            2'b00: RF_RegSel = 4'b1000; // R1 (FIXED)
                            2'b01: RF_RegSel = 4'b0100; // R2
                            2'b10: RF_RegSel = 4'b0010; // R3
                            2'b11: RF_RegSel = 4'b0001; // R4
                        endcase
                        RF_FunSel = 3'b010; // Load 32-bit
                        MuxASel = 2'b01; // DR output
                        current_state = S_FETCH_LSB;
                    end
                    
                    6'h20: begin // STA M[ADDRESS] <- Rx
                        // Load address into AR first, then write
                        ARF_RegSel = 3'b001; // Select AR
                        ARF_FunSel = 2'b10; // Load AR
                        MuxBSel = 2'b11; // IR address field
                        // Then write to memory
                        ARF_OutDSel = 2'b10; // Select AR for address
                        Mem_CS = 1'b0;
                        Mem_WR = 1'b1; // Write
                        case (RegSel)
                            2'b00: RF_OutASel = 3'b100; // R1
                            2'b01: RF_OutASel = 3'b101; // R2
                            2'b10: RF_OutASel = 3'b110; // R3
                            2'b11: RF_OutASel = 3'b111; // R4
                        endcase
                        MuxCSel = 2'b00; // Lower 8 bits of RF output
                        current_state = S_FETCH_LSB;
                    end
                    
                    default: begin
                        // Unknown instruction - just continue
                        current_state = S_FETCH_LSB;
                    end
                endcase
            end
        endcase
        end
        end
endmodule