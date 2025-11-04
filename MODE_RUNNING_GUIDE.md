# How to Run Different Pipeline Modes

This guide explains how to run the different pipeline modes implemented in the RISC-V simulator.

## Available Modes

1. **Mode 1: Single-Cycle (Baseline)** - Reference model for correctness
2. **Mode 2: Pipelined (No Hazard Handling)** - Basic 5-stage pipeline without stalls
3. **Mode 3: Pipelined (Hazard Detection)** - 5-stage pipeline with automatic stall insertion

## Prerequisites

1. **Build the project**:
   ```powershell
   # Navigate to project directory
   cd "C:\Users\Sarthak Ralhan\OneDrive\Desktop\assignment1_ic22btech11012\riscv-simulator-2"
   
   # Create build directory and compile
   mkdir build
   cd build
   cmake ..
   cmake --build . --config Release
   ```

2. **Prepare a test program** (optional, for testing):
   - Create a simple RISC-V assembly file in `examples/` directory

## Running Mode 1: Single-Cycle (Baseline)

### Method 1: Using Interactive VM

```powershell
# From build directory
.\vm.exe --start-vm
```

Then in the VM prompt:
```
modify_config Execution processor_type single_stage
load examples\test1.s
run
```

### Method 2: Using Config File

Edit `vm_state/config.ini`:
```ini
[Execution]
processor_type=single_stage
hazard_detection=false
forwarding=false
```

Then run:
```powershell
.\vm.exe --start-vm
load examples\test1.s
run
```

## Running Mode 2: Basic Pipeline (No Hazard Handling)

### Method 1: Using Interactive VM

```powershell
.\vm.exe --start-vm
```

Then:
```
modify_config Execution processor_type multi_stage
modify_config Execution hazard_detection false
load examples\test1.s
run
```

### Method 2: Using Config File

Edit `vm_state/config.ini`:
```ini
[Execution]
processor_type=multi_stage
hazard_detection=false
forwarding=false
```

Then run:
```powershell
.\vm.exe --start-vm
load examples\test1.s
run
```

**Note**: Mode 2 will produce incorrect results for programs with data dependencies (this is expected behavior).

## Running Mode 3: Pipeline with Hazard Detection

### Method 1: Using Interactive VM

```powershell
.\vm.exe --start-vm
```

Then:
```
modify_config Execution processor_type multi_stage
modify_config Execution hazard_detection true
load examples\test1.s
run
```

### Method 2: Using Config File

Edit `vm_state/config.ini`:
```ini
[Execution]
processor_type=multi_stage
hazard_detection=true
forwarding=false
```

Then run:
```powershell
.\vm.exe --start-vm
load examples\test1.s
run
```

**Features**:
- Automatically detects data hazards (RAW)
- Automatically detects load-use hazards
- Inserts pipeline stalls when needed
- Shows `[STALLED]` in output during stalls
- Displays statistics at end: Cycles, Instructions, CPI, Stalls, Data Hazards

## Quick Command Reference

### Start VM
```powershell
.\vm.exe --start-vm
```

### Switch Modes (Interactive)
```
modify_config Execution processor_type single_stage    # Mode 1
modify_config Execution processor_type multi_stage     # Mode 2 or 3

modify_config Execution hazard_detection false         # Mode 2
modify_config Execution hazard_detection true          # Mode 3
```

### Load and Run Program
```
load examples\test1.s
run
```

### Step-by-Step Execution
```
step
```

## Example Test Programs

### Test Program 1: Simple Arithmetic (No Hazards)
```assembly
addi x1, x0, 5
addi x2, x0, 10
add  x3, x1, x2
```

**Expected**:
- Mode 1: Works correctly
- Mode 2: Works correctly (no dependencies)
- Mode 3: Works correctly (no hazards detected)

### Test Program 2: Data Dependency (RAW Hazard)
```assembly
addi x1, x0, 5
addi x2, x0, 10
add  x3, x1, x2    # x3 = x1 + x2
add  x4, x3, x1    # x4 = x3 + x1 (depends on x3)
```

**Expected**:
- Mode 1: Works correctly
- Mode 2: **Incorrect** (x4 may use stale value of x3)
- Mode 3: Works correctly (stalls 1 cycle for x3)

### Test Program 3: Load-Use Hazard
```assembly
addi x2, x0, 0x100
lw   x1, 0(x2)      # Load from memory
add  x3, x1, x0     # Use x1 immediately
```

**Expected**:
- Mode 1: Works correctly
- Mode 2: **Incorrect** (x3 may use stale value of x1)
- Mode 3: Works correctly (stalls 1 cycle for load-use)

## Understanding the Output

### Mode 3 Output Example:
```
Cycle: 1 PC: 0
Cycle: 2 PC: 4
Cycle: 3 PC: 8 [STALLED]
Cycle: 4 PC: 8
Cycle: 5 PC: c
...
VM_PROGRAM_END
Statistics: Cycles=8 Instructions=5 CPI=1.6 Stalls=1 Data Hazards=1
```

**Interpretation**:
- `[STALLED]`: Pipeline is stalled due to hazard
- `CPI`: Cycles Per Instruction (should be > 1.0 due to stalls)
- `Stalls`: Total number of stall cycles
- `Data Hazards`: Number of hazards detected

## Troubleshooting

### Issue: "VM_MODIFY_CONFIG_ERROR"
- Check that you're using the correct section name: `Execution`
- Check that the value is correct: `single_stage`, `multi_stage`, `true`, `false`

### Issue: Program doesn't run
- Make sure you've loaded a program: `load examples\test1.s`
- Check that the assembly file exists and is valid

### Issue: Mode 3 shows no stalls
- This is normal if your program has no data dependencies
- Try a program with data dependencies (like Test Program 2 or 3 above)

### Issue: Cannot find config file
- The config file is created automatically on first run
- Location: `vm_state/config.ini`
- You can manually create it or let the system create it

## Configuration File Location

The configuration file is located at:
```
vm_state/config.ini
```

You can edit this file directly, or use the `modify_config` command in the interactive VM.

## Next Steps

- **Mode 4**: Will add forwarding to reduce stalls (not yet implemented)
- For now, Mode 3 provides correct execution with automatic stall insertion

