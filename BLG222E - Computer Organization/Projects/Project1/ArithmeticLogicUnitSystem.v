`timescale 1ns / 1ps

module ArithmeticLogicUnitSystem(
    input [2:0] RF_OutASel,         // Selector for output A from Register File
    input [2:0] RF_OutBSel,         // Selector for output B from Register File
    input [2:0] RF_FunSel,          // Function select for Register File
    input [3:0] RF_RegSel,          // Register select for Register File
    input [3:0] RF_ScrSel,          // Source select for Register File

    input [4:0] ALU_FunSel,         // Function select for ALU operations
    input ALU_WF,                   // Write flag for ALU operations
    
    input [1:0] ARF_OutCSel,        // Selector for output C from Address Register File
    input [1:0] ARF_OutDSel,        // Selector for output D from Address Register File
    input [1:0] ARF_FunSel,         // Function select for Address Register File
    input [2:0] ARF_RegSel,         // Register select for Address Register File

    input IR_LH,                    // Load/Store flag for Instruction Register
    input IR_Write,                 // Write flag for Instruction Register

    input Mem_WR,                   // Write flag for Memory
    input Mem_CS,                   // Chip select for Memory

    input [1:0] MuxASel,            // Selector for Mux A
    input [1:0] MuxBSel,            // Selector for Mux B
    input [1:0] MuxCSel,            // Selector for Mux C
    input MuxDSel,                  // Selector for Mux D
    input DR_E,                     // Enable signal for Data Register
    input [1:0] DR_FunSel,          // Function select for Data Register

    input Clock                     // Clock signal for synchronous operations
);
    
    wire [31:0] MuxAOut;            // Output from Mux A
    wire [31:0] MuxBOut;            // Output from Mux B
    wire [7:0] MuxCOut;             // Output from Mux C
    wire [31:0] MuxDOut;            // Output from Mux D

    wire [31:0] ALUOut;             // Output from ALU
    wire [3:0] ALUOutFlag;          // Flags output from ALU

    wire [7:0] MemOut;              // Output from Memory

    wire [15:0] IROut;              // Output from Instruction Register
    
    wire [15:0] OutC;               // Output C from Address Register File
    wire [15:0] Address;            // Address output from Address Register File

    wire [31:0] OutA;               // Output A from Register File
    wire [31:0] OutB;               // Output B from Register File

    wire [31:0] DROut;              // Output from Data Register
    
                                    // Data Register instantiation with correct port names
    DataRegister DR(                
        .I(MuxCOut),                // Input data to be written to Data Register
        .FunSel(DR_FunSel),         // Function select for Data Register
        .E(DR_E),                   // Enable signal for Data Register
        .Clock(Clock),              // Clock signal for synchronous operations
        .DROut(DROut)               // Output from Data Register
    );
                                    // Instruction Register instantiation
    InstructionRegister IR(
        .I(MemOut),                 // Input data to be written to Instruction Register
        .Write(IR_Write),           // Write flag for Instruction Register
        .LH(IR_LH),                 // Load/Store flag for Instruction Register
        .Clock(Clock),              // Clock signal for synchronous operations
        .IROut(IROut)               // Output from Instruction Register
    );
                                    // Address Register File instantiation
    AddressRegisterFile ARF(
        .I(MuxBOut[15:0]),          // Input data to be written to Address Register File
        .OutCSel(ARF_OutCSel),      // Selector for output C from Address Register File
        .OutDSel(ARF_OutDSel),      // Selector for output D from Address Register File
        .FunSel(ARF_FunSel),        // Function select for Address Register File
        .RegSel(ARF_RegSel),        // Register select for Address Register File
        .Clock(Clock),              // Clock signal for synchronous operations
        .OutC(OutC),                // Output C from Address Register File
        .OutD(Address)              // Output D from Address Register File
    );
                                    // Register File instantiation
    RegisterFile RF(
        .I(MuxAOut),                // Input data to be written to registers
        .OutASel(RF_OutASel),       // Selector for output A from Register File
        .OutBSel(RF_OutBSel),       // Selector for output B from Register File
        .FunSel(RF_FunSel),         // Function select for Register File
        .RegSel(RF_RegSel),         // Register select for Register File
        .ScrSel(RF_ScrSel),         // Source select for Register File
        .Clock(Clock),              // Clock signal for synchronous operations
        .OutA(OutA),                // Output A from Register File
        .OutB(OutB)                 // Output B from Register File
    );
                                    // ALU instantiation
    ArithmeticLogicUnit ALU(
        .A(OutA),                   // Input A for ALU operations
        .B(OutB),                   // Input B for ALU operations
        .FunSel(ALU_FunSel),        // Function select for ALU
        .WF(ALU_WF),                // Write flag for ALU
        .Clock(Clock),              // Clock signal for synchronous operations
        .ALUOut(ALUOut),            // Output from ALU
        .FlagsOut(ALUOutFlag)       // Flags output from ALU
    );
    
                                    // Memory module instantiation
    Memory MEM(
        .Address(Address),          // Address input for Memory operations
        .Data(MuxCOut),             // Data input for Memory operations
        .WR(Mem_WR),                // Write flag for Memory operations
        .CS(Mem_CS),                // Chip select for Memory operations
        .Clock(Clock),              // Clock signal for synchronous operations
        .MemOut(MemOut)             // Output from Memory
    );
    
                                                                  // MuxD implementation
    assign MuxDOut = (MuxDSel) ? {{16{1'b0}}, OutC} : OutA;       // Select between OutC and OutA for Mux D
    
                                                                  // MuxA implementation
    assign MuxAOut = (MuxASel == 2'b00) ? ALUOut :                // Select ALUOut for Mux A
                    (MuxASel == 2'b01) ? {{16{1'b0}}, OutC} :     // Select OutC for Mux A
                    (MuxASel == 2'b10) ? {{24{1'b0}}, MemOut} :   // Select MemOut for Mux A
                    {{16{1'b0}}, IROut};                          // Select IROut for Mux A

                                                                  // MuxB implementation
    assign MuxBOut = (MuxBSel == 2'b00) ? ALUOut :                // Select ALUOut for Mux B
                    (MuxBSel == 2'b01) ? {{16{1'b0}}, OutC} :     // Select OutC for Mux B
                    (MuxBSel == 2'b10) ? {{24{1'b0}}, MemOut} :   // Select MemOut for Mux B
                    {{16{1'b0}}, IROut};                          // Select IROut for Mux B

                                                                  // MuxC implementation
    assign MuxCOut = (MuxCSel == 2'b00) ? ALUOut[7:0] :           // Select lower 8 bits of ALUOut for Mux C
                    (MuxCSel == 2'b01) ? ALUOut[15:8] :           // Select middle 8 bits of ALUOut for Mux C
                    (MuxCSel == 2'b10) ? ALUOut[23:16] :          // Select upper middle 8 bits of ALUOut for Mux C
                    ALUOut[31:24];                                // Select upper 8 bits of ALUOut for Mux C

endmodule