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
            T_Reset <= 1'b1;
            
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
            // Handle T_Reset signal
            if (T_Reset) begin
                T <= 12'b0;
                T_Reset <= 1'b0;
            end else begin
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

            // Execute instruction when T == 4
            if (T == 12'b0000_0000_0100) begin
                case (Opcode)
                    6'h19: begin // MOVL Rx[7:0] <- IMMEDIATE (0x19 = 25 decimal)
                        // Enable the correct register
                        case (RegSel)
                            2'b00: RF_RegSel <= 4'b1000; // R1 - bit 3
                            2'b01: RF_RegSel <= 4'b0100; // R2 - bit 2
                            2'b10: RF_RegSel <= 4'b0010; // R3 - bit 1
                            2'b11: RF_RegSel <= 4'b0001; // R4 - bit 0
                        endcase
                        RF_FunSel <= 3'b100;   // Load lower 8 bits, clear upper 24
                        MuxASel <= 2'b11;      // Select IR address field (immediate value)
                    end
                    
                    6'h0A: begin // DEC DSTREG <- SREG1 - 1 (0x0A = 10 decimal)
                        case (DestReg)
                            3'b100: begin // R1
                                RF_RegSel <= 4'b1000;  // Enable R1
                                RF_FunSel <= 3'b000;   // Decrement function
                                // Set ALU to pass through current value for flags
                                RF_OutASel <= 3'b100;  // R1 to ALU input A
                                ALU_FunSel <= 5'b10000; // Pass through A
                                ALU_WF <= 1'b1;        // Update flags
                            end
                            3'b101: begin // R2
                                RF_RegSel <= 4'b0100;  // Enable R2
                                RF_FunSel <= 3'b000;   // Decrement function
                                RF_OutASel <= 3'b101;  // R2 to ALU input A
                                ALU_FunSel <= 5'b10000; // Pass through A
                                ALU_WF <= 1'b1;        // Update flags
                            end
                            // Add other cases as needed
                        endcase
                    end
                    
                    6'h1E: begin // LDAL Rx <- M[ADDRESS] (16-bit) (0x1E = 30 decimal)
                        // First load address into AR
                        ARF_RegSel <= 3'b001;  // Enable AR
                        ARF_FunSel <= 2'b10;   // Load function
                        MuxBSel <= 2'b11;      // IR address field to MuxB
                        
                        // Set up memory read
                        ARF_OutDSel <= 2'b10;  // AR output to memory address
                        Mem_CS <= 1'b0;        // Enable memory
                        Mem_WR <= 1'b0;        // Read operation
                        
                        // Load memory data into destination register
                        case (RegSel)
                            2'b00: RF_RegSel <= 4'b1000; // R1
                            2'b01: RF_RegSel <= 4'b0100; // R2  
                            2'b10: RF_RegSel <= 4'b0010; // R3
                            2'b11: RF_RegSel <= 4'b0001; // R4
                        endcase
                        RF_FunSel <= 3'b101;   // Load 16-bit, clear upper 16
                        MuxASel <= 2'b10;      // Memory data to MuxA (should be DROut per table)
                    end
                    
                    6'h1F: begin // LDAH Rx <- M[ADDRESS] (32-bit) (0x1F = 31 decimal)
                        // Load address into AR
                        ARF_RegSel <= 3'b001;  // Enable AR
                        ARF_FunSel <= 2'b10;   // Load function
                        MuxBSel <= 2'b11;      // IR address field
                        
                        // Start memory read
                        ARF_OutDSel <= 2'b10;  // AR for address
                        Mem_CS <= 1'b0;        // Enable memory
                        Mem_WR <= 1'b0;        // Read
                        
                        // Load into DR first
                        DR_E <= 1'b1;          // Enable DR
                        DR_FunSel <= 2'b01;    // Clear upper, load lower 8
                        
                        // Transfer DR to destination register
                        case (RegSel)
                            2'b00: RF_RegSel <= 4'b1000; // R1
                            2'b01: RF_RegSel <= 4'b0100; // R2
                            2'b10: RF_RegSel <= 4'b0010; // R3  
                            2'b11: RF_RegSel <= 4'b0001; // R4
                        endcase
                        RF_FunSel <= 3'b010;   // Load 32-bit
                        MuxASel <= 2'b10;      // DROut to MuxA
                    end
                    
                    6'h1D: begin // STAR M[AR] <- SREG1 (0x1D = 29 decimal)
                        // Memory write setup
                        ARF_OutDSel <= 2'b10;  // AR for address
                        Mem_CS <= 1'b0;        // Enable memory
                        Mem_WR <= 1'b1;        // Write operation
                        
                        // Get source register data
                        RF_OutASel <= SrcReg1;  // Source register to ALU A
                        ALU_FunSel <= 5'b10000; // Pass through A
                        MuxCSel <= 2'b00;      // ALU lower 8 bits to memory
                    end
                    
                    6'h07: begin // CALL M[SP] <- PC, SP <- SP – 1, PC <- VALUE (0x07 = 7 decimal)
                        // Store PC to memory at SP
                        ARF_OutDSel <= 2'b01;  // SP for address
                        ARF_OutCSel <= 2'b00;  // PC for data
                        Mem_CS <= 1'b0;        // Enable memory
                        Mem_WR <= 1'b1;        // Write
                        MuxCSel <= 2'b00;      // PC lower byte
                        
                        // Decrement SP and load new PC
                        ARF_RegSel <= 3'b110;  // Enable both SP and PC (SP=bit1, PC=bit2)
                        ARF_FunSel <= 2'b00;   // This will decrement SP
                        MuxBSel <= 2'b11;      // IR address for new PC
                    end
                    
                    6'h12: begin // ORR DSTREG <- SREG1 OR SREG2 (0x12 = 18 decimal)
                        // Set up ALU inputs
                        RF_OutASel <= SrcReg1;  // First operand
                        RF_OutBSel <= SrcReg2;  // Second operand
                        ALU_FunSel <= 5'b11000; // 32-bit OR operation
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
                    end
                    
                    default: begin
                        // Do nothing for unknown instructions
                    end
                endcase
            end
        end
    end
endmodule