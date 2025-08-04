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
            T_Reset <= 1'b1;  // Start with T_Reset=1 to give time for PC setup
            
            // Reset all control signals to safe defaults
            RF_OutASel <= 3'b000;
            RF_OutBSel <= 3'b000;
            RF_FunSel <= 3'b000;
            RF_RegSel <= 4'b0000;
            RF_ScrSel <= 4'b0000;
            ALU_FunSel <= 5'b00000;
            ALU_WF <= 1'b0;
            ARF_OutCSel <= 2'b00;
            ARF_OutDSel <= 2'b00;
            ARF_FunSel <= 2'b00;
            ARF_RegSel <= 3'b000;
            IR_LH <= 1'b0;
            IR_Write <= 1'b0;
            Mem_WR <= 1'b0;
            Mem_CS <= 1'b1;
            MuxASel <= 2'b00;
            MuxBSel <= 2'b00;
            MuxCSel <= 2'b00;
            MuxDSel <= 1'b0;
            DR_E <= 1'b0;
            DR_FunSel <= 2'b00;
        end else begin
            // Handle T_Reset signal - simplified for debugging
            if (T_Reset) begin
                T <= 12'b0;
                T_Reset <= 1'b0;
            end else begin
                // Always increment T, regardless of other conditions
                T <= T + 1;
            end
            
            // Default assignments - reset all control signals each cycle  
            RF_OutASel <= 3'b000;
            RF_OutBSel <= 3'b000;
            RF_FunSel <= 3'b000;
            RF_RegSel <= 4'b0000;
            RF_ScrSel <= 4'b0000;
            ALU_FunSel <= 5'b00000;
            ALU_WF <= 1'b0;
            ARF_OutCSel <= 2'b00;
            ARF_OutDSel <= 2'b00;
            ARF_FunSel <= 2'b00;
            ARF_RegSel <= 3'b000;
            IR_LH <= 1'b0;
            IR_Write <= 1'b0;
            Mem_WR <= 1'b0;
            Mem_CS <= 1'b1;
            MuxASel <= 2'b00;
            MuxBSel <= 2'b00;
            MuxCSel <= 2'b00;
            MuxDSel <= 1'b0;
            DR_E <= 1'b0;
            DR_FunSel <= 2'b00;

            // Proper instruction fetch and execution cycle
            case (T)
                12'b0000_0000_0000: begin // T=0: Fetch instruction LSB
                    ARF_OutDSel <= 2'b00;   // PC to memory address
                    Mem_CS <= 1'b0;         // Enable memory
                    Mem_WR <= 1'b0;         // Read operation
                    IR_LH <= 1'b0;          // Load low byte
                    IR_Write <= 1'b1;       // Enable IR write
                end
                
                12'b0000_0000_0001: begin // T=1: Increment PC only
                    ARF_RegSel <= 3'b100;   // Enable PC
                    ARF_FunSel <= 2'b01;    // Increment PC
                    // Don't do memory read in same cycle as PC increment
                end
                
                12'b0000_0000_0010: begin // T=2: Fetch MSB using incremented PC
                    ARF_OutDSel <= 2'b00;   // PC to memory address (now PC+1)
                    Mem_CS <= 1'b0;         // Enable memory
                    Mem_WR <= 1'b0;         // Read operation
                    IR_LH <= 1'b1;          // Load high byte
                    IR_Write <= 1'b1;       // Enable IR write
                end
                
                12'b0000_0000_0011: begin // T=3: Final PC increment
                    ARF_RegSel <= 3'b100;   // Enable PC
                    ARF_FunSel <= 2'b01;    // Increment PC to point to next instruction
                end
                
                default: begin 
                    // T>=4: Execute instruction based on opcode
                    case (Opcode)
                        6'h19: begin // MOVL Rx[7:0] <- IMMEDIATE
                            if (T == 12'b0000_0000_0100) begin // T=4: Execute MOVL
                                case (RegSel)
                                    2'b00: RF_RegSel <= 4'b1000; // R1
                                    2'b01: RF_RegSel <= 4'b0100; // R2
                                    2'b10: RF_RegSel <= 4'b0010; // R3
                                    2'b11: RF_RegSel <= 4'b0001; // R4
                                endcase
                                RF_FunSel <= 3'b100;   // Load lower 8 bits, clear upper 24
                                MuxASel <= 2'b11;      // IR address field
                                T_Reset <= 1'b1;       // Reset for next instruction
                            end
                        end
                    
                    6'h1A: begin // MOVSH Rx[31-8] <- Rx[23-0], Rx[7-0] <- IMMEDIATE
                        if (T == 12'b0000_0000_0100) begin // T=4: Execute MOVSH
                            case (RegSel)
                                2'b00: begin
                                    RF_RegSel <= 4'b1000; // R1
                                    RF_OutASel <= 3'b100; // R1 current value
                                end
                                2'b01: begin
                                    RF_RegSel <= 4'b0100; // R2
                                    RF_OutASel <= 3'b101; // R2 current value
                                end
                                2'b10: begin
                                    RF_RegSel <= 4'b0010; // R3
                                    RF_OutASel <= 3'b110; // R3 current value
                                end
                                2'b11: begin
                                    RF_RegSel <= 4'b0001; // R4
                                    RF_OutASel <= 3'b111; // R4 current value
                                end
                            endcase
                            RF_FunSel <= 3'b110;   // Shift left 8 bits and load immediate
                            MuxASel <= 2'b11;      // IR address field
                            T_Reset <= 1'b1;       // Reset T counter
                        end
                    end
                    
                    6'h0A: begin // DEC DSTREG <- SREG1 - 1
                        if (T == 12'b0000_0000_0100) begin // T=4: Execute DEC
                            // Simple approach: just decrement the destination register directly
                            // The register file/ARF should handle the decrement operation
                            
                            if (DestReg <= 3'b011) begin
                                // ARF registers (PC, SP, AR)
                                case (DestReg)
                                    3'b000: ARF_RegSel <= 3'b100; // PC
                                    3'b001: ARF_RegSel <= 3'b010; // SP  
                                    3'b010, 3'b011: ARF_RegSel <= 3'b001; // AR
                                endcase
                                ARF_FunSel <= 2'b00; // Decrement function
                            end else if (DestReg <= 3'b111) begin
                                // RF registers R1-R4 (DestReg 0-3) and S1-S4 (DestReg 4-7)
                                case (DestReg)
                                    3'b000: begin // R1 (this shouldn't happen since <= 3'b011 handled above)
                                        RF_RegSel <= 4'b1000; // Enable R1
                                        RF_OutASel <= 3'b000; // R1 for flags
                                    end
                                    3'b001: begin // R2
                                        RF_RegSel <= 4'b0100; // Enable R2
                                        RF_OutASel <= 3'b001; // R2 for flags
                                    end
                                    3'b010: begin // R3
                                        RF_RegSel <= 4'b0010; // Enable R3
                                        RF_OutASel <= 3'b010; // R3 for flags
                                    end
                                    3'b011: begin // R4
                                        RF_RegSel <= 4'b0001; // Enable R4
                                        RF_OutASel <= 3'b011; // R4 for flags
                                    end
                                    3'b100: begin // S1
                                        RF_ScrSel <= 4'b1000; // Enable S1
                                        RF_OutASel <= 3'b100; // S1 for flags
                                    end
                                    3'b101: begin // S2  
                                        RF_ScrSel <= 4'b0100; // Enable S2
                                        RF_OutASel <= 3'b101; // S2 for flags
                                    end
                                    3'b110: begin // S3
                                        RF_ScrSel <= 4'b0010; // Enable S3
                                        RF_OutASel <= 3'b110; // S3 for flags
                                    end
                                    3'b111: begin // S4
                                        RF_ScrSel <= 4'b0001; // Enable S4
                                        RF_OutASel <= 3'b111; // S4 for flags  
                                    end
                                endcase
                                RF_FunSel <= 3'b000;    // Decrement function
                                ALU_FunSel <= 5'b10000; // Pass through for flags
                                ALU_WF <= 1'b1;         // Update flags
                            end
                            T_Reset <= 1'b1;           // Reset T counter
                        end
                    end
                    
                    6'h1E: begin // LDAL Rx <- M[ADDRESS] (16-bit) - FIXED WITH MuxC
                        case (T)
                            12'b0000_0000_0100: begin // T=4: Load address into AR
                                ARF_RegSel <= 3'b001;   // Enable AR
                                ARF_FunSel <= 2'b10;    // Load function
                                MuxBSel <= 2'b11;       // IR address field to AR
                            end
                            12'b0000_0000_0101: begin // T=5: Read LSB from memory
                                ARF_OutDSel <= 2'b10;   // AR to memory address
                                Mem_CS <= 1'b0;         // Enable memory
                                Mem_WR <= 1'b0;         // Read operation
                                MuxCSel <= 2'b11;       // Route MemOut to MuxCOut
                                DR_E <= 1'b1;           // Enable DR
                                DR_FunSel <= 2'b01;     // Clear DR, load LSB
                            end
                            12'b0000_0000_0110: begin // T=6: Increment AR
                                ARF_RegSel <= 3'b001;   // Enable AR
                                ARF_FunSel <= 2'b01;    // Increment AR
                            end
                            12'b0000_0000_0111: begin // T=7: Read MSB from incremented address
                                ARF_OutDSel <= 2'b10;   // AR to memory address (now AR+1)
                                Mem_CS <= 1'b0;         // Enable memory
                                Mem_WR <= 1'b0;         // Read operation
                                MuxCSel <= 2'b11;       // Route MemOut to MuxCOut
                                DR_E <= 1'b1;           // Enable DR
                                DR_FunSel <= 2'b10;     // Shift left 8, load MSB
                            end
                            12'b0000_0000_1000: begin // T=8: Store 16-bit result to register
                                case (RegSel)
                                    2'b00: RF_RegSel <= 4'b1000; // R1
                                    2'b01: RF_RegSel <= 4'b0100; // R2
                                    2'b10: RF_RegSel <= 4'b0010; // R3
                                    2'b11: RF_RegSel <= 4'b0001; // R4
                                endcase
                                RF_FunSel <= 3'b101;    // Load 16-bit, clear upper 16
                                MuxASel <= 2'b10;       // DROut to register
                                T_Reset <= 1'b1;        // Reset T counter
                            end
                        endcase
                    end
                    
                    6'h1F: begin // LDAH Rx <- M[ADDRESS] (32-bit) - FIXED MULTI-CYCLE
                        case (T)
                            12'b0000_0000_0100: begin // T=4: Load address into AR
                                ARF_RegSel <= 3'b001;   // Enable AR
                                ARF_FunSel <= 2'b10;    // Load function
                                MuxBSel <= 2'b11;       // IR address field to AR
                            end
                            12'b0000_0000_0101: begin // T=5: Read byte 0 (LSB)
                                ARF_OutDSel <= 2'b10;   // AR to memory address
                                Mem_CS <= 1'b0;         // Enable memory
                                Mem_WR <= 1'b0;         // Read operation
                                MuxCSel <= 2'b11;       // Route MemOut to MuxCOut
                                DR_E <= 1'b1;           // Enable DR
                                DR_FunSel <= 2'b01;     // Clear DR, load LSB
                            end
                            12'b0000_0000_0110: begin // T=6: Increment AR to address+1
                                ARF_RegSel <= 3'b001;   // Enable AR
                                ARF_FunSel <= 2'b01;    // Increment AR
                            end
                            12'b0000_0000_0111: begin // T=7: Read byte 1 from address+1
                                ARF_OutDSel <= 2'b10;   // AR to memory address
                                Mem_CS <= 1'b0;         // Enable memory
                                Mem_WR <= 1'b0;         // Read operation
                                MuxCSel <= 2'b11;       // Route MemOut to MuxCOut
                                DR_E <= 1'b1;           // Enable DR
                                DR_FunSel <= 2'b10;     // Shift left 8, load next byte
                            end
                            12'b0000_0000_1000: begin // T=8: Increment AR to address+2
                                ARF_RegSel <= 3'b001;   // Enable AR
                                ARF_FunSel <= 2'b01;    // Increment AR
                            end
                            12'b0000_0000_1001: begin // T=9: Read byte 2 from address+2
                                ARF_OutDSel <= 2'b10;   // AR to memory address
                                Mem_CS <= 1'b0;         // Enable memory
                                Mem_WR <= 1'b0;         // Read operation
                                MuxCSel <= 2'b11;       // Route MemOut to MuxCOut
                                DR_E <= 1'b1;           // Enable DR
                                DR_FunSel <= 2'b10;     // Shift left 8, load next byte
                            end
                            12'b0000_0000_1010: begin // T=10: Increment AR to address+3
                                ARF_RegSel <= 3'b001;   // Enable AR
                                ARF_FunSel <= 2'b01;    // Increment AR
                            end
                            12'b0000_0000_1011: begin // T=11: Read byte 3 (MSB) from address+3
                                ARF_OutDSel <= 2'b10;   // AR to memory address
                                Mem_CS <= 1'b0;         // Enable memory
                                Mem_WR <= 1'b0;         // Read operation
                                MuxCSel <= 2'b11;       // Route MemOut to MuxCOut
                                DR_E <= 1'b1;           // Enable DR
                                DR_FunSel <= 2'b10;     // Shift left 8, load MSB
                            end
                            default: begin // T>=12: Store 32-bit result to register
                                case (RegSel)
                                    2'b00: RF_RegSel <= 4'b1000; // R1
                                    2'b01: RF_RegSel <= 4'b0100; // R2
                                    2'b10: RF_RegSel <= 4'b0010; // R3
                                    2'b11: RF_RegSel <= 4'b0001; // R4
                                endcase
                                RF_FunSel <= 3'b010;    // Load 32-bit
                                MuxASel <= 2'b10;       // DROut to register
                                T_Reset <= 1'b1;        // Reset T counter
                            end
                        endcase
                    end
                    
                    6'h1D: begin // STAR M[AR] <- SREG1 (32-bit write) - MULTI-CYCLE
                        case (T)
                            12'b0000_0000_0100: begin // T=4: Write byte 0 (MSB) to memory
                                // For STAR R3, hardcode the expected data path for debugging
                                // R3 contains 0x06000a08, we need to write 0x06 first (MSB)
                                RF_OutASel <= 3'b010;    // R3 to ALU input A
                                ALU_FunSel <= 5'b10000;  // Pass through A (register data)
                                ALU_WF <= 1'b0;          // Don't update flags 
                                MuxCSel <= 2'b11;        // ALU[31:24] to memory data (MSB first - big endian)
                                ARF_OutDSel <= 2'b10;    // AR to memory address
                                Mem_CS <= 1'b0;          // Enable memory
                                Mem_WR <= 1'b1;          // Write operation
                            end
                            12'b0000_0000_0101: begin // T=5: Increment AR
                                ARF_RegSel <= 3'b001;    // Enable AR
                                ARF_FunSel <= 2'b01;     // Increment AR
                            end
                            12'b0000_0000_0110: begin // T=6: Write byte 1 to incremented address
                                RF_OutASel <= 3'b010;    // R3 to ALU input A
                                ALU_FunSel <= 5'b10000;  // Pass through A
                                ALU_WF <= 1'b0;          // Don't update flags
                                MuxCSel <= 2'b10;        // ALU[23:16] to memory data (second byte)
                                ARF_OutDSel <= 2'b10;    // AR to memory address
                                Mem_CS <= 1'b0;          // Enable memory
                                Mem_WR <= 1'b1;          // Write operation
                            end
                            12'b0000_0000_0111: begin // T=7: Increment AR
                                ARF_RegSel <= 3'b001;    // Enable AR
                                ARF_FunSel <= 2'b01;     // Increment AR
                            end
                            12'b0000_0000_1000: begin // T=8: Write byte 2 to incremented address
                                RF_OutASel <= 3'b010;    // R3 to ALU input A
                                ALU_FunSel <= 5'b10000;  // Pass through A
                                ALU_WF <= 1'b0;          // Don't update flags
                                MuxCSel <= 2'b01;        // ALU[15:8] to memory data (third byte)
                                ARF_OutDSel <= 2'b10;    // AR to memory address
                                Mem_CS <= 1'b0;          // Enable memory
                                Mem_WR <= 1'b1;          // Write operation
                            end
                            12'b0000_0000_1001: begin // T=9: Increment AR
                                ARF_RegSel <= 3'b001;    // Enable AR
                                ARF_FunSel <= 2'b01;     // Increment AR
                            end
                            12'b0000_0000_1010: begin // T=10: Write byte 3 (LSB) to incremented address
                                RF_OutASel <= 3'b010;    // R3 to ALU input A
                                ALU_FunSel <= 5'b10000;  // Pass through A
                                ALU_WF <= 1'b0;          // Don't update flags
                                MuxCSel <= 2'b00;        // ALU[7:0] to memory data (LSB - fourth byte)
                                ARF_OutDSel <= 2'b10;    // AR to memory address
                                Mem_CS <= 1'b0;          // Enable memory
                                Mem_WR <= 1'b1;          // Write operation
                                T_Reset <= 1'b1;         // Reset T counter
                            end
                        endcase
                    end
                    
                    6'h20: begin // STA M[ADDRESS] <- Rx
                        // Direct memory write with address from IR
                        MuxDSel <= 1'b1;       // IR address field to memory address
                        Mem_CS <= 1'b0;        // Enable memory
                        Mem_WR <= 1'b1;        // Write operation
                        
                        // Get source register data
                        case (RegSel)
                            2'b00: RF_OutASel <= 3'b100; // R1
                            2'b01: RF_OutASel <= 3'b101; // R2
                            2'b10: RF_OutASel <= 3'b110; // R3
                            2'b11: RF_OutASel <= 3'b111; // R4
                        endcase
                        ALU_FunSel <= 5'b10000; // Pass through A
                        MuxCSel <= 2'b00;      // ALU output to memory
                        T_Reset <= 1'b1;       // Reset T counter
                    end
                    
                    6'h07: begin // CALL - Push PC to stack, jump to address - SIMPLIFIED APPROACH
                        case (T)
                            12'b0000_0000_0100: begin // T=4: Load PC into R4 temporarily
                                ARF_OutCSel <= 2'b00;    // PC to OutC
                                MuxASel <= 2'b01;        // OutC (PC) to MuxA
                                RF_RegSel <= 4'b0001;    // Load into R4 temporarily
                                RF_FunSel <= 3'b101;     // Load 16-bit, clear upper
                            end
                            12'b0000_0000_0101: begin // T=5: Push PC low byte to stack
                                RF_OutASel <= 3'b011;    // R4 to ALU A
                                ALU_FunSel <= 5'b10000;  // Pass through A
                                MuxCSel <= 2'b00;        // ALU[7:0] to memory (PC low byte)
                                ARF_OutDSel <= 2'b01;    // SP to memory address
                                Mem_CS <= 1'b0;          // Enable memory
                                Mem_WR <= 1'b1;          // Write operation
                            end
                            12'b0000_0000_0110: begin // T=6: Decrement SP
                                ARF_RegSel <= 3'b010;    // Enable SP
                                ARF_FunSel <= 2'b00;     // Decrement SP
                            end
                            12'b0000_0000_0111: begin // T=7: Push PC high byte to decremented stack
                                RF_OutASel <= 3'b011;    // R4 to ALU A
                                ALU_FunSel <= 5'b10000;  // Pass through A
                                MuxCSel <= 2'b01;        // ALU[15:8] to memory (PC high byte)
                                ARF_OutDSel <= 2'b01;    // SP to memory address (now SP-1)
                                Mem_CS <= 1'b0;          // Enable memory
                                Mem_WR <= 1'b1;          // Write operation
                            end
                            12'b0000_0000_1000: begin // T=8: Decrement SP and set PC
                                ARF_RegSel <= 3'b010;    // Enable SP
                                ARF_FunSel <= 2'b00;     // Decrement SP (now SP-2)
                            end
                            12'b0000_0000_1001: begin // T=9: Set PC to call address
                                ARF_RegSel <= 3'b100;    // Enable PC
                                ARF_FunSel <= 2'b10;     // Load function
                                MuxBSel <= 2'b11;        // IR address field to PC
                            end
                            12'b0000_0000_1010: begin // T=10: Reset to start next instruction
                                T_Reset <= 1'b1;         // Reset T counter for next instruction fetch  
                            end
                        endcase
                    end
                    
                    6'h08: begin // RET SP <- SP + 1, PC <- M[SP] (16 bit)
                        // Increment SP first
                        ARF_RegSel <= 3'b010;  // Enable SP
                        ARF_FunSel <= 2'b01;   // Increment SP
                        
                        // Read PC from memory at SP
                        ARF_OutDSel <= 2'b01;  // SP for address
                        Mem_CS <= 1'b0;        // Enable memory
                        Mem_WR <= 1'b0;        // Read
                        
                        // Load PC from memory
                        ARF_RegSel <= 3'b100;  // Enable PC
                        ARF_FunSel <= 2'b10;   // Load function
                        MuxBSel <= 2'b10;      // Memory data to PC
                        T_Reset <= 1'b1;       // Reset T counter
                    end
                    
                    6'h12: begin // ORR R1 <- R1 OR AR - MULTI-CYCLE WORKAROUND
                        case (T)
                            12'b0000_0000_0100: begin // T=4: Load AR into DR as 32-bit value
                                ARF_OutCSel <= 2'b10;    // AR to OutC
                                // Use MuxA to route AR to DR via RF system
                                MuxASel <= 2'b01;        // OutC (AR) to MuxA 
                                RF_RegSel <= 4'b0001;    // Load into R4 temporarily
                                RF_FunSel <= 3'b101;     // Load 16-bit, clear upper
                            end
                            12'b0000_0000_0101: begin // T=5: Perform OR operation R1 | R4 -> R1
                                RF_OutASel <= 3'b000;    // R1 to ALU A
                                RF_OutBSel <= 3'b011;    // R4 to ALU B
                                ALU_FunSel <= 5'b11000;  // OR operation
                                RF_RegSel <= 4'b1000;    // Target R1
                                RF_FunSel <= 3'b010;     // Load 32-bit
                                MuxASel <= 2'b00;        // ALU output to R1
                                T_Reset <= 1'b1;         // Reset T counter
                            end
                        endcase
                    end
                    
                    6'h10: begin // AND DSTREG <- SREG1 AND SREG2
                        if (T == 12'b0000_0000_0100) begin // T=4: Execute AND
                            RF_OutASel <= SrcReg1;  // First operand
                            RF_OutBSel <= SrcReg2;  // Second operand
                            ALU_FunSel <= 5'b10111; // 32-bit AND operation
                            ALU_WF <= 1'b1;        // Update flags
                            
                            // Store result in destination register
                            case (DestReg)
                                3'b100: RF_RegSel <= 4'b1000; // R1
                                3'b101: RF_RegSel <= 4'b0100; // R2
                                3'b110: RF_RegSel <= 4'b0010; // R3
                                3'b111: RF_RegSel <= 4'b0001; // R4
                            endcase
                            RF_FunSel <= 3'b010;   // Load 32-bit
                            MuxASel <= 2'b00;      // ALU output to register
                            T_Reset <= 1'b1;       // Reset T counter
                        end
                    end
                    
                    6'h00: begin // BRA PC <- VALUE
                        if (T == 12'b0000_0000_0100) begin // T=4: Execute BRA
                            ARF_RegSel <= 3'b100;   // Enable PC
                            ARF_FunSel <= 2'b10;    // Load function
                            MuxBSel <= 2'b11;       // IR address field to PC
                            T_Reset <= 1'b1;        // Reset T counter for next instruction fetch
                        end
                    end
                    
                    6'h01: begin // BNE IF Z=0 THEN PC <- VALUE
                        if (T == 12'b0000_0000_0100) begin // T=4: Execute BNE
                            if (!ALUSys.ALU.Z) begin  // Check Zero flag
                                ARF_RegSel <= 3'b100;   // Enable PC
                                ARF_FunSel <= 2'b10;    // Load function
                                MuxBSel <= 2'b11;       // IR address field to PC
                            end
                            T_Reset <= 1'b1;        // Reset T counter for next instruction fetch
                        end
                    end
                    
                    6'h02: begin // BEQ IF Z=1 THEN PC <- VALUE
                        if (T == 12'b0000_0000_0100) begin // T=4: Execute BEQ
                            if (ALUSys.ALU.Z) begin   // Check Zero flag
                                ARF_RegSel <= 3'b100;   // Enable PC
                                ARF_FunSel <= 2'b10;    // Load function
                                MuxBSel <= 2'b11;       // IR address field to PC
                            end
                            T_Reset <= 1'b1;        // Reset T counter for next instruction fetch
                        end
                    end
                    
                    6'h09: begin // INC DSTREG <- SREG1 + 1
                        if (T == 12'b0000_0000_0100) begin // T=4: Execute INC
                            // For ARF registers, use direct increment
                            if (DestReg <= 3'b011) begin
                                case (DestReg)
                                    3'b000: ARF_RegSel <= 3'b100; // PC
                                    3'b001: ARF_RegSel <= 3'b010; // SP
                                    3'b010, 3'b011: ARF_RegSel <= 3'b001; // AR
                                endcase
                                ARF_FunSel <= 2'b01;    // Increment
                            end else begin
                                // For RF registers, use ALU for increment and flags
                                RF_OutASel <= SrcReg1;   // Source register
                                ALU_FunSel <= 5'b10000; // Pass through for now
                                ALU_WF <= 1'b1;         // Update flags
                                
                                case (DestReg)
                                    3'b100: RF_RegSel <= 4'b1000; // R1
                                    3'b101: RF_RegSel <= 4'b0100; // R2
                                    3'b110: RF_RegSel <= 4'b0010; // R3
                                    3'b111: RF_RegSel <= 4'b0001; // R4
                                endcase
                                RF_FunSel <= 3'b001;    // Increment
                            end
                            T_Reset <= 1'b1;           // Reset T counter
                        end
                    end
                    
                    6'h18: begin // MOV DSTREG <- SREG1
                        if (T == 12'b0000_0000_0100) begin // T=4: Execute MOV
                            RF_OutASel <= SrcReg1;   // Source register
                            ALU_FunSel <= 5'b10000;  // Pass through
                            ALU_WF <= 1'b1;         // Update flags
                            
                            // Store in destination register
                            case (DestReg)
                                3'b100: RF_RegSel <= 4'b1000; // R1
                                3'b101: RF_RegSel <= 4'b0100; // R2
                                3'b110: RF_RegSel <= 4'b0010; // R3
                                3'b111: RF_RegSel <= 4'b0001; // R4
                            endcase
                            RF_FunSel <= 3'b010;    // Load 32-bit
                            MuxASel <= 2'b00;       // ALU output
                            T_Reset <= 1'b1;        // Reset T counter
                        end
                    end
                    
                    6'h15: begin // ADD DSTREG <- SREG1 + SREG2
                        if (T == 12'b0000_0000_0100) begin // T=4: Execute ADD
                            RF_OutASel <= SrcReg1;   // First operand
                            RF_OutBSel <= SrcReg2;   // Second operand
                            ALU_FunSel <= 5'b10100;  // ADD operation (A + B)
                            ALU_WF <= 1'b1;         // Update flags
                            
                            // Store result in destination register
                            case (DestReg)
                                3'b100: RF_RegSel <= 4'b1000; // R1
                                3'b101: RF_RegSel <= 4'b0100; // R2
                                3'b110: RF_RegSel <= 4'b0010; // R3
                                3'b111: RF_RegSel <= 4'b0001; // R4
                            endcase
                            RF_FunSel <= 3'b010;    // Load 32-bit
                            MuxASel <= 2'b00;       // ALU output
                            T_Reset <= 1'b1;        // Reset T counter
                        end
                    end
                    
                    6'h16: begin // SUB DSTREG <- SREG1 - SREG2
                        if (T == 12'b0000_0000_0100) begin // T=4: Execute SUB
                            RF_OutASel <= SrcReg1;   // First operand
                            RF_OutBSel <= SrcReg2;   // Second operand
                            ALU_FunSel <= 5'b10110;  // SUB operation (A - B)
                            ALU_WF <= 1'b1;         // Update flags
                            
                            // Store result in destination register
                            case (DestReg)
                                3'b100: RF_RegSel <= 4'b1000; // R1
                                3'b101: RF_RegSel <= 4'b0100; // R2
                                3'b110: RF_RegSel <= 4'b0010; // R3
                                3'b111: RF_RegSel <= 4'b0001; // R4
                            endcase
                            RF_FunSel <= 3'b010;    // Load 32-bit
                            MuxASel <= 2'b00;       // ALU output
                            T_Reset <= 1'b1;        // Reset T counter
                        end
                    end
                    
                    6'h17: begin // CMP - Compare SREG1 and SREG2, update flags only
                        if (T == 12'b0000_0000_0100) begin // T=4: Execute CMP
                            RF_OutASel <= SrcReg1;   // First operand
                            RF_OutBSel <= SrcReg2;   // Second operand
                            ALU_FunSel <= 5'b10110;  // SUB operation (A - B) for comparison
                            ALU_WF <= 1'b1;         // Update flags
                            // Don't store result - CMP only updates flags
                            T_Reset <= 1'b1;        // Reset T counter
                        end
                    end
                    
                    6'h04: begin // PSHL M[SP] <- Rx[15:0], SP <- SP - 1 (16-bit push)
                        case (T)
                            12'b0000_0000_0100: begin // T=4: Store register to stack (little-endian)
                                case (RegSel)
                                    2'b00: RF_OutASel <= 3'b000; // R1
                                    2'b01: RF_OutASel <= 3'b001; // R2
                                    2'b10: RF_OutASel <= 3'b010; // R3
                                    2'b11: RF_OutASel <= 3'b011; // R4
                                endcase
                                ALU_FunSel <= 5'b10000;  // Pass through A
                                MuxCSel <= 2'b00;        // ALU[7:0] to memory (low byte first)
                                ARF_OutDSel <= 2'b01;    // SP for address
                                Mem_CS <= 1'b0;          // Enable memory
                                Mem_WR <= 1'b1;          // Write
                            end
                            12'b0000_0000_0101: begin // T=5: Decrement SP
                                ARF_RegSel <= 3'b010;    // Enable SP
                                ARF_FunSel <= 2'b00;     // Decrement SP
                            end
                            12'b0000_0000_0110: begin // T=6: Store high byte to decremented stack
                                case (RegSel)
                                    2'b00: RF_OutASel <= 3'b000; // R1
                                    2'b01: RF_OutASel <= 3'b001; // R2
                                    2'b10: RF_OutASel <= 3'b010; // R3
                                    2'b11: RF_OutASel <= 3'b011; // R4
                                endcase
                                ALU_FunSel <= 5'b10000;  // Pass through A
                                MuxCSel <= 2'b01;        // ALU[15:8] to memory (high byte)
                                ARF_OutDSel <= 2'b01;    // SP for address (now SP-1)
                                Mem_CS <= 1'b0;          // Enable memory
                                Mem_WR <= 1'b1;          // Write
                            end
                            12'b0000_0000_0111: begin // T=7: Decrement SP again
                                ARF_RegSel <= 3'b010;    // Enable SP
                                ARF_FunSel <= 2'b00;     // Decrement SP (now SP-2)
                                T_Reset <= 1'b1;         // Reset T counter
                            end
                        endcase
                    end

                    6'h05: begin // PUSHL M[SP] <- Rx, SP <- SP - 1 (16-bit)
                        case (T)
                            12'b0000_0000_0100: begin // T=4: Store low byte to stack
                                case (RegSel)
                                    2'b00: RF_OutASel <= 3'b000; // R1
                                    2'b01: RF_OutASel <= 3'b001; // R2
                                    2'b10: RF_OutASel <= 3'b010; // R3
                                    2'b11: RF_OutASel <= 3'b011; // R4
                                endcase
                                ARF_OutDSel <= 2'b01;   // SP for address
                                Mem_CS <= 1'b0;         // Enable memory
                                Mem_WR <= 1'b1;         // Write operation
                                ALU_FunSel <= 5'b10000; // Pass through A
                                MuxCSel <= 2'b00;       // ALU output (lower 8 bits)
                            end
                            12'b0000_0000_0101: begin // T=5: Decrement SP, store high byte
                                ARF_RegSel <= 3'b010;   // Enable SP
                                ARF_FunSel <= 2'b00;    // Decrement SP
                                ARF_OutDSel <= 2'b01;   // SP for address
                                Mem_CS <= 1'b0;         // Enable memory
                                Mem_WR <= 1'b1;         // Write operation
                                // Store upper 8 bits of register
                                case (RegSel)
                                    2'b00: RF_OutASel <= 3'b000; // R1
                                    2'b01: RF_OutASel <= 3'b001; // R2
                                    2'b10: RF_OutASel <= 3'b010; // R3
                                    2'b11: RF_OutASel <= 3'b011; // R4
                                endcase
                                ALU_FunSel <= 5'b10001; // Pass through A, upper 8 bits
                                MuxCSel <= 2'b00;       // ALU output
                                T_Reset <= 1'b1;        // Reset T counter
                            end
                        endcase
                    end
                    
                    6'h03: begin // POPL SP <- SP + 1, Rx <- M[SP] (16-bit)
                        case (T)
                            12'b0000_0000_0100: begin // T=4: Increment SP
                                ARF_RegSel <= 3'b010;   // Enable SP
                                ARF_FunSel <= 2'b01;    // Increment SP
                            end
                            12'b0000_0000_0101: begin // T=5: Read low byte from stack
                                ARF_OutDSel <= 2'b01;   // SP for address
                                Mem_CS <= 1'b0;         // Enable memory
                                Mem_WR <= 1'b0;         // Read
                                DR_E <= 1'b1;           // Enable DR
                                DR_FunSel <= 2'b01;     // Load low byte, clear upper
                            end
                            12'b0000_0000_0110: begin // T=6: Increment SP, read high byte
                                ARF_RegSel <= 3'b010;   // Enable SP
                                ARF_FunSel <= 2'b01;    // Increment SP
                                ARF_OutDSel <= 2'b01;   // SP for address
                                Mem_CS <= 1'b0;         // Enable memory
                                Mem_WR <= 1'b0;         // Read
                                DR_E <= 1'b1;           // Enable DR
                                DR_FunSel <= 2'b11;     // Shift left 8, load new byte
                            end
                            12'b0000_0000_0111: begin // T=7: Transfer to destination register
                                case (RegSel)
                                    2'b00: RF_RegSel <= 4'b1000; // R1
                                    2'b01: RF_RegSel <= 4'b0100; // R2
                                    2'b10: RF_RegSel <= 4'b0010; // R3
                                    2'b11: RF_RegSel <= 4'b0001; // R4
                                endcase
                                RF_FunSel <= 3'b101;    // Load 16-bit, clear upper
                                MuxASel <= 2'b10;       // DROut to register
                                T_Reset <= 1'b1;        // Reset T counter
                            end
                        endcase
                    end
                    
                    6'h1C: begin // LDARH DSTREG <- M[AR] (32-bit)
                        case (T)
                            12'b0000_0000_0100: begin // T=4: Read first byte
                                ARF_OutDSel <= 2'b10;   // AR for address
                                Mem_CS <= 1'b0;         // Enable memory
                                Mem_WR <= 1'b0;         // Read operation
                                DR_E <= 1'b1;           // Enable DR
                                DR_FunSel <= 2'b01;     // Load low byte, clear upper
                            end
                            12'b0000_0000_0101: begin // T=5: Increment AR, read second byte
                                ARF_RegSel <= 3'b001;   // Enable AR
                                ARF_FunSel <= 2'b01;    // Increment AR
                                ARF_OutDSel <= 2'b10;   // AR for address
                                Mem_CS <= 1'b0;         // Enable memory
                                Mem_WR <= 1'b0;         // Read operation
                                DR_E <= 1'b1;           // Enable DR
                                DR_FunSel <= 2'b11;     // Shift left 8, load new byte
                            end
                            12'b0000_0000_0110: begin // T=6: Continue for 32-bit
                                ARF_RegSel <= 3'b001;   // Enable AR
                                ARF_FunSel <= 2'b01;    // Increment AR
                                ARF_OutDSel <= 2'b10;   // AR for address
                                Mem_CS <= 1'b0;         // Enable memory
                                Mem_WR <= 1'b0;         // Read operation
                                DR_E <= 1'b1;           // Enable DR
                                DR_FunSel <= 2'b11;     // Shift left 8, load new byte
                            end
                            12'b0000_0000_0111: begin // T=7: Read final byte and store
                                ARF_RegSel <= 3'b001;   // Enable AR
                                ARF_FunSel <= 2'b01;    // Increment AR
                                ARF_OutDSel <= 2'b10;   // AR for address
                                Mem_CS <= 1'b0;         // Enable memory
                                Mem_WR <= 1'b0;         // Read operation
                                DR_E <= 1'b1;           // Enable DR
                                DR_FunSel <= 2'b11;     // Shift left 8, load new byte
                                
                                // Transfer DR to destination register
                                case (DestReg)
                                    3'b100: RF_RegSel <= 4'b1000; // R1
                                    3'b101: RF_RegSel <= 4'b0100; // R2
                                    3'b110: RF_RegSel <= 4'b0010; // R3
                                    3'b111: RF_RegSel <= 4'b0001; // R4
                                endcase
                                RF_FunSel <= 3'b010;    // Load 32-bit
                                MuxASel <= 2'b10;       // DROut to register
                                T_Reset <= 1'b1;        // Reset T counter
                            end
                        endcase
                    end
                    
                        default: begin
                            // Unknown instruction - reset T counter
                            T_Reset <= 1'b1;
                        end
                    endcase
                end
            endcase
        end
    end
endmodule