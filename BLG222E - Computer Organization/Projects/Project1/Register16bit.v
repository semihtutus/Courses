`timescale 1ns / 1ps

module Register16bit(
    input [15:0] I,                 // Input data to be written to the register
    input [1:0] FunSel,             // Function select for the register
    input E,                        // Enable signal for the register
    input Clock,                    // Clock signal for synchronous operations
    output reg [15:0] Q             // Output from the register
    );
    always@(posedge Clock) begin
        if(E) begin
            case(FunSel)             // Function select cases
                2'b00: Q <= Q-1;     // Decrement
                2'b01: Q <= Q+1;     // Increment
                2'b10: Q <= I;       // Load
                2'b11: Q <= 16'b0;   // Clear
                default: Q <= Q;
            endcase
        end
    end
endmodule


