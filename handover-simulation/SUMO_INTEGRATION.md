# SUMO Integration Guide for NS3 Handover Simulation

This guide explains how to use SUMO-generated mobility traces in the NS3 handover simulation.

## Overview

The simulation supports two mobility modes:
1. **SUMO TCL Trace**: Pre-generated mobility traces from SUMO (recommended)
2. **RandomWaypoint**: Built-in random mobility model (default fallback)

## Step 1: Generate SUMO FCD Output

First, run SUMO to generate a Floating Car Data (FCD) trace:

```bash
cd /home/ubuntu/cs297/handover-simulation

# Run SUMO (adjust path to your SUMO installation)
/path/to/sumo/bin/sumo -c urban-scenario.sumocfg

# This generates sumo-trace.xml (or the file specified in sumocfg)
```

## Step 2: Convert FCD to NS3 TCL Format

Convert the SUMO FCD output to ns2-style TCL format that NS3 can read:

### Option A: Using sumo_to_ns3_trace.py (Recommended)

```bash
python3 sumo_to_ns3_trace.py sumo-trace.xml ns3-mobility.tcl
```

### Option B: Using sumo_helper.py

```bash
python3 sumo_helper.py --convert-trace sumo-trace.xml --output ns3-mobility.tcl
```

This will generate a TCL file with the following format:
```tcl
$node_(0) set X_ 400.00
$node_(0) set Y_ 300.00
$node_(0) set Z_ 1.5
$ns_ at 0.10 "$node_(0) setdest 402.55 300.00 25.50"
$ns_ at 0.20 "$node_(0) setdest 405.10 300.00 25.50"
...
```

**Important**: The trace file must have nodes numbered from 0 to (numUes-1). For example:
- `$node_(0)`, `$node_(1)`, ..., `$node_(9)` for 10 UEs

## Step 3: Run NS3 Simulation with SUMO Trace

Use the `--sumoTrace` parameter to specify the TCL trace file:

```bash
# Basic usage
./handover-simulation --sumoTrace=ns3-mobility.tcl --numUes=10 --simTime=100

# With all parameters
./handover-simulation \
  --sumoTrace=ns3-mobility.tcl \
  --numUes=10 \
  --numGnbs=8 \
  --simTime=100 \
  --ueTxPower=26 \
  --gnbTxPower=46
```

### Without SUMO Trace (uses RandomWaypoint)

If you don't provide `--sumoTrace`, the simulation uses RandomWaypoint mobility:

```bash
./handover-simulation --numUes=10 --simTime=100
```

## Complete Workflow Example

```bash
# 1. Generate SUMO FCD output
/path/to/sumo/bin/sumo -c urban-scenario.sumocfg

# 2. Convert to NS3 TCL format
python3 sumo_to_ns3_trace.py sumo-trace.xml ns3-mobility.tcl

# 3. Verify the trace file
head -20 ns3-mobility.tcl

# 4. Run NS3 simulation (NS-3.36 uses CMake)
cd /path/to/ns3
./build/scratch/scratch_handover-simulation --sumoTrace=/path/to/ns3-mobility.tcl --numUes=10 --simTime=100
```

## Trace File Format Requirements

The TCL trace file must follow ns2-style format:

1. **Initial Position**: Set initial X, Y, Z coordinates
   ```tcl
   $node_(0) set X_ 400.00
   $node_(0) set Y_ 300.00
   $node_(0) set Z_ 1.5
   ```

2. **Mobility Events**: Use `setdest` to define movements
   ```tcl
   $ns_ at 0.10 "$node_(0) setdest 402.55 300.00 25.50"
   ```
   Format: `$ns_ at <time> "$node_(<id>) setdest <x> <y> <speed>"`

3. **Node Numbering**: Nodes must be numbered from 0 to (numUes-1)
   - For 10 UEs: `$node_(0)` through `$node_(9)`
   - For 20 UEs: `$node_(0)` through `$node_(19)`

4. **Time Format**: Use decimal seconds (e.g., `0.10`, `1.00`, `10.50`)

## Command-Line Parameters

### SUMO Trace Parameters

| Parameter | Description | Default | Example |
|-----------|-------------|---------|---------|
| `--sumoTrace` | Path to SUMO TCL trace file | (empty, uses RandomWaypoint) | `ns3-mobility.tcl` |
| `--numUes` | Number of UEs (must match trace file) | 10 | `10` |

### Other Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `--numGnbs` | Number of gNBs | 8 |
| `--simTime` | Simulation time in seconds | 100 |
| `--ueTxPower` | UE transmit power (dBm) | 26 |
| `--gnbTxPower` | gNB transmit power (dBm) | 46 |

## Troubleshooting

### Error: "SUMO trace file not found"

**Problem**: The trace file path is incorrect or the file doesn't exist.

**Solution**:
- Use absolute path: `--sumoTrace=/full/path/to/ns3-mobility.tcl`
- Verify file exists: `ls -l ns3-mobility.tcl`
- Check file permissions

### Error: "Node index mismatch"

**Problem**: The trace file has fewer nodes than `--numUes`.

**Solution**:
- Ensure trace file has nodes `$node_(0)` through `$node_(N-1)` where N = numUes
- Regenerate trace file with correct number of vehicles
- Use `--numUes` matching the trace file

### Error: "No mobility data for node X"

**Problem**: Some nodes in the trace file have no mobility data.

**Solution**:
- Ensure all nodes have initial position and at least one `setdest` command
- Check SUMO FCD output contains data for all vehicles
- Verify conversion script processed all vehicles

### Trace file loads but UEs don't move

**Problem**: Trace file format might be incorrect.

**Solution**:
- Verify TCL syntax: `$node_(0)`, not `$node(0)` or `$node_0`
- Check time values are in seconds and increasing
- Ensure `setdest` commands have correct format: `"$node_(X) setdest X Y speed"`

### Using relative vs absolute paths

**Recommended**: Use absolute paths for trace files to avoid confusion:
```bash
./build/scratch/scratch_handover-simulation --sumoTrace=/home/ubuntu/cs297/handover-simulation/ns3-mobility.tcl
```

For relative paths, the path is relative to where you run the command:
```bash
cd /home/ubuntu/cs297/ns-allinone-3.36/ns-3.36
./build/scratch/scratch_handover-simulation --sumoTrace=../handover-simulation/ns3-mobility.tcl
```

**Note**: NS-3.36 uses CMake, so the executable name is `scratch_handover-simulation` (with `scratch_` prefix) and is located in `build/scratch/`.

## Code Implementation Details

The integration uses `Ns2MobilityHelper` from NS3's mobility module:

```cpp
// Load trace file
Ns2MobilityHelper ns2MobilityHelper (sumoTraceFile);

// Install on UE nodes (nodes 0 to numUes-1)
ns2MobilityHelper.Install (ueNodes.Begin (), ueNodes.End ());
```

The helper automatically:
- Parses the TCL trace file
- Installs `WaypointMobilityModel` on nodes
- Schedules mobility events at the specified times

## Verification

To verify everything works:

1. **Check trace file format**:
   ```bash
   head -30 ns3-mobility.tcl
   ```

2. **Count nodes in trace file**:
   ```bash
   grep -o '\$node_([0-9]*)' ns3-mobility.tcl | sort -u | wc -l
   ```
   This should equal `numUes`

3. **Run simulation with verbose logging**:
   ```bash
   NS_LOG=HandoverSimulation=info ./handover-simulation --sumoTrace=ns3-mobility.tcl --numUes=10
   ```
   Look for: "SUMO TCL trace loaded successfully for X UEs"

## Performance Considerations

- **Trace file size**: Large trace files (many timesteps) may slow down simulation startup
- **Simulation time**: Ensure trace file covers the entire simulation duration (`simTime`)
- **Update frequency**: More frequent updates (smaller `step-length` in SUMO) create larger trace files

## References

- NS3 Mobility Models: https://www.nsnam.org/docs/models/html/mobility.html
- Ns2MobilityHelper: https://www.nsnam.org/doxygen/classns3_1_1_ns2_mobility_helper.html
- SUMO FCD Output: https://sumo.dlr.de/docs/Simulation/Output/FCDOutput.html

