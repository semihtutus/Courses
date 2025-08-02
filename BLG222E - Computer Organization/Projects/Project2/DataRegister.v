`timescale 1ns / 1ps

module DataRegister(
    input [7:0] I,                                // Input data to be written to Data Register
    input [1:0] FunSel,                           // Function select for Data Register
    input E,                                      // Enable signal for Data Register
    input Clock,                                  // Clock signal for synchronous operations
    input Reset,                                  // Reset signal
    output reg [31:0] DROut                       // Output from Data Register
    );
    always@(posedge Clock or negedge Reset) begin
        if (!Reset) begin
            DROut <= 32'b0;                       // Reset DROut to 0
        end else if(E) begin
            case(FunSel)
                2'b00: begin                      // Load data into Data Register
                    DROut[31:8] <= {24{I[7]}};    // Sign extend bit 7 to upper 24 bits
                    DROut[7:0] <= I[7:0];         // Load lower 8 bits
                end
                2'b01: begin                      // Clear Data Register and load lower 8 bits
                    DROut[31:8] <= 24'b0;         // Clear upper 24 bits
                    DROut[7:0] <= I[7:0];         // Load lower 8 bits
                end
                2'b10: begin                      // Shift Data Register left by 8 bits
                    DROut[31:8] <= DROut[23:0];   // 8-bit left shift
                    DROut[7:0] <= I[7:0];         // Load lower 8 bits
                end
                2'b11: begin                      // Shift Data Register right by 8 bits
                    DROut[23:0] <= DROut[31:8];   // 8-bit right shift
                    DROut[31:24] <= I[7:0];       // Load upper 8 bits
                end
                default: DROut <= DROut;          // No operation
            endcase
        end
    end
endmodule

