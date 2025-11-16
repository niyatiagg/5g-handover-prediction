# Building and Running the Handover Simulation

## Overview

The `handover-simulation.cc` file needs to be **compiled** before you can run it. NS-3.36 uses **CMake** (not waf) to build simulation programs.

## Step 1: Locate the Source File

The simulation source file is located at:
```
/home/ubuntu/cs297/ns-allinone-3.36/ns-3.36/scratch/handover-simulation.cc
```

## Step 2: Configure and Build the Simulation

Navigate to the NS3 directory and configure/build using CMake:

```bash
cd /home/ubuntu/cs297/ns-allinone-3.36/ns-3.36

# Configure the build (first time only)
cmake -S . -B build -DNS3_EXAMPLES=ON -DNS3_TESTS=ON

# Build the simulation (first build takes 10-30 minutes)
cmake --build build
```

**Note**: The first build may take 10-30 minutes depending on your system. Subsequent builds will be faster.

### Build Options

You can configure additional options during the cmake configuration step:

```bash
# Debug build (slower runtime, easier debugging)
cmake -S . -B build -DNS3_EXAMPLES=ON -DNS3_TESTS=ON -DCMAKE_BUILD_TYPE=Debug

# Release/optimized build (faster runtime, harder debugging)
cmake -S . -B build -DNS3_EXAMPLES=ON -DNS3_TESTS=ON -DCMAKE_BUILD_TYPE=Release

# Enable specific modules (if needed)
cmake -S . -B build -DNS3_EXAMPLES=ON -DNS3_TESTS=ON -DNS3_VERBOSE=ON
```

## Step 3: Run the Simulation

After building, the executable will be named `scratch_handover-simulation` (note the `scratch_` prefix). Run it using one of these methods:

**Method 1: Run from build directory**
```bash
cd /home/ubuntu/cs297/ns-allinone-3.36/ns-3.36

# Run with SUMO trace
./build/scratch/scratch_handover-simulation \
  --sumoTrace=/home/ubuntu/cs297/handover-simulation/ns3-mobility.tcl \
  --numUes=10 \
  --simTime=100

# Run without SUMO trace (uses RandomWaypoint)
./build/scratch/scratch_handover-simulation --numUes=10 --simTime=100

# Run with all parameters
./build/scratch/scratch_handover-simulation \
  --sumoTrace=/home/ubuntu/cs297/handover-simulation/ns3-mobility.tcl \
  --numUes=10 \
  --numGnbs=8 \
  --simTime=100 \
  --ueTxPower=26 \
  --gnbTxPower=46
```

**Method 2: Use CMake's test runner (alternative)**
```bash
cd /home/ubuntu/cs297/ns-allinone-3.36/ns-3.36
ctest -R handover-simulation -V
```

## Quick Start Example

Here's a complete workflow:

```bash
# 1. Navigate to NS3 directory
cd /home/ubuntu/cs297/ns-allinone-3.36/ns-3.36

# 2. Configure and build (first time only - takes 10-30 minutes)
cmake -S . -B build -DNS3_EXAMPLES=ON -DNS3_TESTS=ON
cmake --build build

# 3. Generate SUMO trace (if using SUMO)
cd /home/ubuntu/cs297/handover-simulation
/path/to/sumo/bin/sumo -c urban-scenario.sumocfg
python3 sumo_to_ns3_trace.py sumo-trace.xml ns3-mobility.tcl

# 4. Run simulation
cd /home/ubuntu/cs297/ns-allinone-3.36/ns-3.36
./build/scratch/scratch_handover-simulation \
  --sumoTrace=/home/ubuntu/cs297/handover-simulation/ns3-mobility.tcl \
  --numUes=10 \
  --simTime=100
```

## Build Troubleshooting

### Error: "cmake: command not found"

**Solution**: Install CMake:
```bash
# On Ubuntu/Debian
sudo apt-get install cmake

# On macOS
brew install cmake
```

### Error: "CMakeLists.txt not found"

**Solution**: Make sure you're in the NS3 root directory:
```bash
cd /home/ubuntu/cs297/ns-allinone-3.36/ns-3.36
```

### Error: "scratch_handover-simulation not found"

**Problem**: The simulation hasn't been built yet.

**Solution**: Run the build commands:
```bash
cmake -S . -B build -DNS3_EXAMPLES=ON -DNS3_TESTS=ON
cmake --build build
```

### Error: "No rule to make target"

**Solution**: Reconfigure CMake:
```bash
rm -rf build  # Remove old build directory (optional)
cmake -S . -B build -DNS3_EXAMPLES=ON -DNS3_TESTS=ON
cmake --build build
```

### Error: Compilation errors

**Solution**: 
- Check that all required NS3 modules are enabled
- Verify the source file is in `scratch/` directory
- Check for syntax errors in `handover-simulation.cc`
- Try a clean rebuild:
  ```bash
  rm -rf build
  cmake -S . -B build -DNS3_EXAMPLES=ON -DNS3_TESTS=ON
  cmake --build build
  ```

### Build takes too long

**Solution**: 
- First build always takes long (10-30 minutes) - be patient!
- Use optimized build for faster runtime (but slower compilation):
  ```bash
  cmake -S . -B build -DNS3_EXAMPLES=ON -DNS3_TESTS=ON -DCMAKE_BUILD_TYPE=Release
  cmake --build build
  ```

### Module not found errors

**Solution**: Enable the required modules during CMake configuration. Common modules are enabled by default, but you may need to enable specific ones:
```bash
# Enable verbose output to see what modules are available
cmake -S . -B build -DNS3_EXAMPLES=ON -DNS3_TESTS=ON -DNS3_VERBOSE=ON
```

## Verification

To verify the build was successful:

```bash
# Check if executable exists
ls -lh build/scratch/scratch_handover-simulation*

# Or try running with help
./build/scratch/scratch_handover-simulation --help

# List all scratch programs
ls -lh build/scratch/
```

## CMake Build Commands Reference

### Initial Configuration
```bash
cmake -S . -B build -DNS3_EXAMPLES=ON -DNS3_TESTS=ON
```
- `-S .` : Source directory (current directory)
- `-B build` : Build directory (creates `build/` folder)
- `-DNS3_EXAMPLES=ON` : Enable examples (required for scratch programs)
- `-DNS3_TESTS=ON` : Enable tests (optional but recommended)

### Building
```bash
cmake --build build
```
- Builds all configured targets
- Use `-j` flag for parallel builds: `cmake --build build -j$(nproc)`

### Rebuilding After Changes
```bash
# Just rebuild (CMake detects changes automatically)
cmake --build build

# Clean and rebuild
cmake --build build --clean-first
```

### Clean Build
```bash
# Remove build directory and reconfigure
rm -rf build
cmake -S . -B build -DNS3_EXAMPLES=ON -DNS3_TESTS=ON
cmake --build build
```

## Notes

1. **Executable Name**: NS-3.36 CMake adds a `scratch_` prefix to all scratch programs. So `handover-simulation.cc` becomes `scratch_handover-simulation`.

2. **First Build**: The first build compiles all of NS3 and takes a long time (10-30 minutes). Be patient!

3. **Subsequent Builds**: After the first build, only changed files are recompiled, so it's much faster.

4. **Build Directory**: CMake creates a `build/` directory in the NS3 root. The executable is in `build/scratch/`.

5. **Path to Trace File**: When using `--sumoTrace`, use absolute paths to avoid confusion:
   ```bash
   --sumoTrace=/home/ubuntu/cs297/handover-simulation/ns3-mobility.tcl
   ```

6. **CMake vs Waf**: NS-3.36 uses **CMake**, not waf. Older versions (3.35 and below) used waf.

## Differences from Waf (Old Versions)

If you're familiar with older NS3 versions using waf:

| Old (waf) | New (CMake) |
|-----------|-------------|
| `./waf configure` | `cmake -S . -B build -DNS3_EXAMPLES=ON` |
| `./waf build` | `cmake --build build` |
| `./waf --run handover-simulation` | `./build/scratch/scratch_handover-simulation` |
| Executable: `build/scratch/handover-simulation` | Executable: `build/scratch/scratch_handover-simulation` |

## Quick Command Cheat Sheet

```bash
# Configure (first time)
cd /home/ubuntu/cs297/ns-allinone-3.36/ns-3.36
cmake -S . -B build -DNS3_EXAMPLES=ON -DNS3_TESTS=ON

# Build
cmake --build build

# Run
./build/scratch/scratch_handover-simulation --sumoTrace=path/to/trace.tcl --numUes=10 --simTime=100

# Clean rebuild
rm -rf build && cmake -S . -B build -DNS3_EXAMPLES=ON -DNS3_TESTS=ON && cmake --build build
```
