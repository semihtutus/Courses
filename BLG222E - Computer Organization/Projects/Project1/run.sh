#!/bin/bash

# Create simulation directory
folder="Simulation_Files"
mkdir -p "$folder"
cp "RAM.mem" "$folder/RAM.mem"
cd "$folder" || exit

# Define common compilation function
compile_and_run() {
    local top_module=$1
    local source_files=$2
    echo "Compiling $top_module..."
    iverilog -o "${top_module}_sim" $source_files
    echo "Running simulation..."
    vvp "${top_module}_sim"
}

# Register 16 Simulation
compile_and_run "Register16bitSimulation" "../Register16bit.v ../Register16bitSimulation.v ../Helper.v"

# Register 32 Simulation
compile_and_run "Register32bitSimulation" "../Register32bit.v ../Register32bitSimulation.v ../Helper.v"

# Register File Simulation
compile_and_run "RegisterFileSimulation" "../Register32bit.v ../RegisterFile.v ../RegisterFileSimulation.v ../Helper.v"

# Address Register File Simulation
compile_and_run "AddressRegisterFileSimulation" "../Register16bit.v ../AddressRegisterFile.v ../AddressRegisterFileSimulation.v ../Helper.v"

# Instruction Register Simulation
compile_and_run "InstructionRegisterSimulation" "../InstructionRegister.v ../InstructionRegisterSimulation.v ../Helper.v"

# Data Register Simulation
compile_and_run "DataRegisterSimulation" "../DataRegister.v ../DataRegisterSimulation.v ../Helper.v"

# Arithmetic Logic Unit Simulation
compile_and_run "ArithmeticLogicUnitSimulation" "../ArithmeticLogicUnit.v ../ArithmeticLogicUnitSimulation.v ../Helper.v"

# Arithmetic Logic Unit System Simulation
compile_and_run "ArithmeticLogicUnitSystemSimulation" \
    "../Register16bit.v ../Register32bit.v ../RegisterFile.v \
    ../AddressRegisterFile.v ../InstructionRegister.v \
    ../DataRegister.v ../ArithmeticLogicUnit.v ../Memory.v \
    ../ArithmeticLogicUnitSystem.v ../ArithmeticLogicUnitSystemSimulation.v \
    ../Helper.v"

cd ..