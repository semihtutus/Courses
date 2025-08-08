`timescale 1ns / 1ps

module ArithmeticLogicUnit(
    input [31:0] A,                  // Input A for ALU operations
    input [31:0] B,                  // Input B for ALU operations
    input [4:0] FunSel,              // Function select for ALU operations
    input WF,                        // Write flag for ALU operations
    input Clock,                     // Clock signal
    input Reset,                     // Reset signal
    output reg [31:0] ALUOut,        // ALU output
    output reg [3:0] FlagsOut        // Flags output: Zero, Carry, Negative, Overflow
);
    
    reg [32:0] temp_result;          // 33-bit for carry/borrow
    reg Z, C, N, O;                  // Flags: Zero, Carry, Negative, Overflow

    always @(*) begin
        // Default values
        Z = 0; C = 0; N = 0; O = 0;  // Reset flags
        temp_result = 0;             // Reset temp_result
        ALUOut = 0;                  // Reset ALU output
        
        case(FunSel)
            5'b00000: begin // 16-bit A
                ALUOut = {16'b0, A[15:0]};
                Z = (ALUOut[15:0] == 0);
                N = ALUOut[15];
            end

            5'b00001: begin // 16-bit B (restored)
                ALUOut = {16'b0, B[15:0]};
                Z = (ALUOut[15:0] == 0);
                N = ALUOut[15];
            end
            
            5'b00010: begin // 16-bit NOT A
                ALUOut = {16'b0, ~A[15:0]};
                Z = (ALUOut[15:0] == 0);
                N = ALUOut[15];
            end

            5'b00011: begin // 16-bit NOT B
                ALUOut = {16'b0, ~B[15:0]};
                Z = (ALUOut[15:0] == 0);
                N = ALUOut[15];
            end

            5'b00100: begin // 16-bit A + B
                temp_result = {17'b0, A[15:0]} + {17'b0, B[15:0]};
                ALUOut = {16'b0, temp_result[15:0]};
                Z = (temp_result[15:0] == 0);
                C = temp_result[16];
                N = temp_result[15];
                O = (A[15] == B[15]) && (A[15] != temp_result[15]);
            end

            5'b00101: begin // 16-bit A + B + Carry
                temp_result = {17'b0, A[15:0]} + {17'b0, B[15:0]} + {16'b0, FlagsOut[2]};
                ALUOut = {16'b0, temp_result[15:0]};
                Z = (temp_result[15:0] == 0);
                C = temp_result[16];
                N = temp_result[15];
                O = (A[15] == B[15]) && (A[15] != temp_result[15]);
            end

            5'b00110: begin // 16-bit A - B
                temp_result = {17'b0, A[15:0]} - {17'b0, B[15:0]};
                ALUOut = {16'b0, temp_result[15:0]};
                Z = (temp_result[15:0] == 0);
                C = (A[15:0] < B[15:0]);
                N = temp_result[15];
                O = (A[15] != B[15]) && (temp_result[15] != A[15]);
            end

            5'b00111: begin // 16-bit A AND B
                ALUOut = {16'b0, A[15:0] & B[15:0]};
                Z = (ALUOut[15:0] == 0);
                N = ALUOut[15];
            end

            5'b01000: begin // 16-bit A OR B
                ALUOut = {16'b0, A[15:0] | B[15:0]};
                Z = (ALUOut[15:0] == 0);
                N = ALUOut[15];
            end

            5'b01001: begin // 16-bit A XOR B
                ALUOut = {16'b0, A[15:0] ^ B[15:0]};
                Z = (ALUOut[15:0] == 0);
                N = ALUOut[15];
            end

            5'b01010: begin // 16-bit A NAND B
                ALUOut = {16'b0, ~(A[15:0] & B[15:0])};
                Z = (ALUOut[15:0] == 0);
                N = ALUOut[15];
            end

            5'b01011: begin // 16-bit LSL A
                ALUOut = {16'b0, A[14:0], 1'b0};
                Z = (ALUOut[15:0] == 0);
                C = A[15];
                N = ALUOut[15];
            end

            5'b01100: begin // 16-bit LSR A
                ALUOut = {16'b0, 1'b0, A[15:1]};
                Z = (ALUOut[15:0] == 0);
                C = A[0];
                N = ALUOut[15];
            end

            5'b01101: begin // 16-bit ASR A
                ALUOut = {16'b0, A[15], A[15:1]};
                Z = (ALUOut[15:0] == 0);
                N = ALUOut[15];
            end

            5'b01110: begin // 16-bit CSL A
                ALUOut = {16'b0, A[14:0], A[15]};
                Z = (ALUOut[15:0] == 0);
                C = A[15];
                N = ALUOut[15];
            end

            5'b01111: begin // 16-bit CSR A
                ALUOut = {16'b0, A[0], A[15:1]};
                Z = (ALUOut[15:0] == 0);
                C = A[0];
                N = ALUOut[15];
            end

            // 32-bit operations
            5'b10000: begin // 32-bit A
                ALUOut = A;
                Z = (ALUOut == 0);
                N = ALUOut[31];
            end

            5'b10001: begin // 32-bit B
                ALUOut = B;
                Z = (ALUOut == 0);
                N = ALUOut[31];
            end

            5'b10010: begin // 32-bit NOT A
                ALUOut = ~A;
                Z = (ALUOut == 0);
                N = ALUOut[31];
            end

            5'b10011: begin // 32-bit NOT B
                ALUOut = ~B;
                Z = (ALUOut == 0);
                N = ALUOut[31];
            end

            5'b10100: begin // 32-bit A + B
                temp_result = A + B;
                ALUOut = temp_result[31:0];
                Z = (ALUOut == 0);
                C = temp_result[32];
                N = ALUOut[31];
                O = (A[31] == B[31]) && (A[31] != ALUOut[31]);
            end

            5'b10101: begin // 32-bit A + B + Carry
                temp_result = A + B + FlagsOut[2];
                ALUOut = temp_result[31:0];
                Z = (ALUOut == 0);
                C = temp_result[32];
                N = ALUOut[31];
                O = (A[31] == B[31]) && (A[31] != ALUOut[31]);
            end

            5'b10110: begin // 32-bit A - B
                temp_result = A - B;
                ALUOut = temp_result[31:0];
                Z = (ALUOut == 0);
                C = (A < B);
                N = ALUOut[31];
                O = (A[31] != B[31]) && (ALUOut[31] != A[31]);
            end

            5'b10111: begin // 32-bit A AND B
                ALUOut = A & B;
                Z = (ALUOut == 0);
                N = ALUOut[31];
            end

            5'b11000: begin // 32-bit A OR B
                ALUOut = A | B;
                Z = (ALUOut == 0);
                N = ALUOut[31];
            end

            5'b11001: begin // 32-bit A XOR B
                ALUOut = A ^ B;
                Z = (ALUOut == 0);
                N = ALUOut[31];
            end

            5'b11010: begin // 32-bit A NAND B
                ALUOut = ~(A & B);
                Z = (ALUOut == 0);
                N = ALUOut[31];
            end

            5'b11011: begin // 32-bit LSL A
                ALUOut = {A[30:0], 1'b0};
                Z = (ALUOut == 0);
                C = A[31];
                N = ALUOut[31];
            end

            5'b11100: begin // 32-bit LSR A
                ALUOut = {1'b0, A[31:1]};
                Z = (ALUOut == 0);
                C = A[0];
                N = ALUOut[31];
            end

            5'b11101: begin // 32-bit ASR A
                ALUOut = {{1{A[31]}}, A[31:1]};
                Z = (ALUOut == 0);
                N = ALUOut[31];
            end

            5'b11110: begin // 32-bit CSL A
                ALUOut = {A[30:0], A[31]};
                Z = (ALUOut == 0);
                C = A[31];
                N = ALUOut[31];
            end

            5'b11111: begin // 32-bit CSR A
                ALUOut = {A[0], A[31:1]};
                Z = (ALUOut == 0);
                C = A[0];
                N = ALUOut[31];
            end

            default: begin
                ALUOut = 32'b0;
                Z = 1; C = 0; N = 0; O = 0;
            end
        endcase
    end

    // Register flags on clock edge
    always @(posedge Clock or negedge Reset) begin
        if (!Reset) begin
            FlagsOut <= 4'b0000; // Reset flags
        end else if (WF) begin
            FlagsOut <= {Z, C, N, O}; // Update flags output
        end
    end
endmodule