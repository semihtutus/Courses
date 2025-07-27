`timescale 1ns / 1ps

module InstructionRegister(
    input [7:0] I,                // 8-bit input data
    input Write,                  // Write enable signal
    input LH,                     // Load High or Low signal
    input Clock,                  // Clock signal
    output reg [15:0] IROut       // 16-bit output register
    );
    
    initial begin
        IROut = 16'b0;            // Initialize IROut to 0
    end

    always @(posedge Clock) begin
        if (Write) begin
            if (LH)
                IROut[15:8] <= I;  // Write to MSB
            else
                IROut[7:0] <= I;   // Write to LSB
        end
    end
endmodule