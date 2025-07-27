`timescale 1ns / 1ps

module AddressRegisterFile(
    input [15:0] I,             // Input data to be written to registers
    input [1:0] OutCSel,        // Selector for output C
    input [1:0] OutDSel,        // Selector for output D
    input [1:0] FunSel,         // Function selector
    input [2:0] RegSel,         // Register selector
    input Clock,                // Clock signal for synchronous operations
    output reg [15:0] OutC,     // Output C from selected register
    output reg [15:0] OutD      // Output D from selected register
    );
    
    wire [15:0] PCQ, ARQ, SPQ;  // Outputs from registers PC, AR, and SP
    
                                // Enable logic for registers based on RegSel
    wire PCE = RegSel[2];       // Enable for PC
    wire ARE = RegSel[0];       // Enable for AR
    wire SPE = RegSel[1];       // Enable for SP

    Register16bit PC(.I(I), .FunSel(FunSel), .E(PCE), .Clock(Clock), .Q(PCQ));      // Program Counter register
    Register16bit AR(.I(I), .FunSel(FunSel), .E(ARE), .Clock(Clock), .Q(ARQ));      // Address Register
    Register16bit SP(.I(I), .FunSel(FunSel), .E(SPE), .Clock(Clock), .Q(SPQ));      // Stack Pointer

    always @(*) begin
        case(OutCSel)
            2'b00: OutC = PCQ;      // Select output from PC
            2'b01: OutC = SPQ;      // Select output from SP
            2'b10: OutC = ARQ;      // Select output from AR
            2'b11: OutC = ARQ;      // Select output from AR
            default: OutC = 16'b0;  // Default case to avoid latches
        endcase
        
        case(OutDSel)
            2'b00: OutD = PCQ;      // Select output from PC
            2'b01: OutD = SPQ;      // Select output from SP
            2'b10: OutD = ARQ;      // Select output from AR
            2'b11: OutD = ARQ;      // Select output from AR
            default: OutD = 16'b0;  // Default case to avoid latches
        endcase
    end
endmodule