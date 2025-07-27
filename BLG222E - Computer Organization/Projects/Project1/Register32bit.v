`timescale 1ns / 1ps

module Register32bit(
    input [31:0] I,                           // Input data to be written to the register
    input [2:0] FunSel,                       // Function select for the register
    input E,                                  // Enable signal for the register
    input Clock,                              // Clock signal for synchronous operations
    output reg [31:0] Q                       // Output from the register
    );
    always@(posedge Clock) begin
        if(E) begin
            case(FunSel)                      // Function select cases
                3'b000: Q <= Q-1;             // Decrement
                3'b001: Q <= Q+1;             // Increment
                3'b010: Q <= I;               // Load
                3'b011: Q <= 32'b0;           // Clear
                3'b100: begin
                    Q[31:8] <= 24'b0;         // Clear upper 24 bits
                    Q[7:0] <= I[7:0];         // Load lower 8 bits
                end
                3'b101: begin
                    Q[31:16] <= 16'b0;        // Clear upper 16 bits
                    Q[15:0] <= I[15:0];       // Load lower 16 bits
                end
                3'b110: begin
                    Q[31:8] <= Q[23:0];       // 8-bit left shift
                    Q[7:0] <= I[7:0];         // Load lower 8 bits
                end
                3'b111: begin
                    Q[31:16] <= {16{I[15]}};  // Sign extend bit 15
                    Q[15:0] <= I[15:0];       // Load lower 16 bits
                end
                default: Q <= Q;              // No operation
            endcase
        end
    end
endmodule

    