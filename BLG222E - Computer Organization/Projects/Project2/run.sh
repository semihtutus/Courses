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

# CPU System Simulation
iverilog -o cpu_system_sim ../Register16bit.v \
    ../Register32bit.v \
    ../RegisterFile.v \
    ../AddressRegisterFile.v \
    ../InstructionRegister.v \
    ../DataRegister.v \
    ../ArithmeticLogicUnit.v \
    ../Memory.v \
    ../ArithmeticLogicUnitSystem.v \
    ../CPUSystem.v \
    ../CPUSystemSimulation.v \
    ../Helper.v

vvp cpu_system_sim

# CPU System Simulation for Factorial
iverilog -o cpu_system_fact_sim ../Register16bit.v \
    ../Register32bit.v \
    ../RegisterFile.v \
    ../AddressRegisterFile.v \
    ../InstructionRegister.v \
    ../DataRegister.v \
    ../ArithmeticLogicUnit.v \
    ../Memory.v \
    ../ArithmeticLogicUnitSystem.v \
    ../CPUSystem.v \
    ../CPUSystemSimulation_Factorial.v \
    ../Helper.v

vvp cpu_system_fact_sim

cd ..