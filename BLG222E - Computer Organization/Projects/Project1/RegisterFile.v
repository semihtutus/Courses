`timescale 1ns / 1ps
module RegisterFile (
    input [31:0] I,              // Input data to be written to registers
    input [2:0] OutASel,         // Selector for output A
    input [2:0] OutBSel,         // Selector for output B
    input [2:0] FunSel,          // Function selector
    input [3:0] RegSel,          // Register selector
    input [3:0] ScrSel,          // Scratch register selector
    input Clock,                 // Clock signal for synchronous operations
    output reg [31:0] OutA,      // Output A from selected register       
    output reg [31:0] OutB       // Output B from selected register
    );
    wire [31:0] R1Q, R2Q, R3Q, R4Q, S1Q, S2Q, S3Q, S4Q;  // Outputs from registers R1, R2, R3, R4 and S1, S2, S3, S4
    
                                  // Enable logic for registers based on RegSel
    wire R1E = RegSel[3];         // Enable for R1
    wire R2E = RegSel[2];         // Enable for R2
    wire R3E = RegSel[1];         // Enable for R3
    wire R4E = RegSel[0];         // Enable for R4

                                  // Enable logic for scratch registers based on ScrSel
    wire S1E = ScrSel[3];         // Enable for S1
    wire S2E = ScrSel[2];         // Enable for S2
    wire S3E = ScrSel[1];         // Enable for S3
    wire S4E = ScrSel[0];         // Enable for S4
    
    Register32bit R1(.I(I), .FunSel(FunSel), .E(R1E), .Clock(Clock), .Q(R1Q));  // Register for R1
    Register32bit R2(.I(I), .FunSel(FunSel), .E(R2E), .Clock(Clock), .Q(R2Q));  // Register for R2
    Register32bit R3(.I(I), .FunSel(FunSel), .E(R3E), .Clock(Clock), .Q(R3Q));  // Register for R3
    Register32bit R4(.I(I), .FunSel(FunSel), .E(R4E), .Clock(Clock), .Q(R4Q));  // Register for R4

    Register32bit S1(.I(I), .FunSel(FunSel), .E(S1E), .Clock(Clock), .Q(S1Q));  // Register for S1
    Register32bit S2(.I(I), .FunSel(FunSel), .E(S2E), .Clock(Clock), .Q(S2Q));  // Register for S2
    Register32bit S3(.I(I), .FunSel(FunSel), .E(S3E), .Clock(Clock), .Q(S3Q));  // Register for S3
    Register32bit S4(.I(I), .FunSel(FunSel), .E(S4E), .Clock(Clock), .Q(S4Q));  // Register for S4

    always @(*) begin
        case(OutASel)
            3'b000: OutA = R1Q;     // Select output from R1
            3'b001: OutA = R2Q;     // Select output from R2
            3'b010: OutA = R3Q;     // Select output from R3
            3'b011: OutA = R4Q;     // Select output from R4
            3'b100: OutA = S1Q;     // Select output from S1
            3'b101: OutA = S2Q;     // Select output from S2
            3'b110: OutA = S3Q;     // Select output from S3
            3'b111: OutA = S4Q;     // Select output from S4
            default: OutA = 32'b0;  // Default case to avoid latches
        endcase
        
        case(OutBSel)
            3'b000: OutB = R1Q;     // Select output from R1
            3'b001: OutB = R2Q;     // Select output from R2
            3'b010: OutB = R3Q;     // Select output from R3
            3'b011: OutB = R4Q;     // Select output from R4
            3'b100: OutB = S1Q;     // Select output from S1
            3'b101: OutB = S2Q;     // Select output from S2
            3'b110: OutB = S3Q;     // Select output from S3
            3'b111: OutB = S4Q;     // Select output from S4
            default: OutB = 32'b0;  // Default case to avoid latches
        endcase
    end
endmodule