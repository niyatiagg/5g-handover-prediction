# SUMO Integration Guide for NS3 Simulation

This guide explains how to run SUMO to generate mobility traces that can be used in the NS3 handover simulation.

## Overview

The workflow consists of three main steps:
1. **Run SUMO** to generate FCD (Floating Car Data) output
2. **Convert the FCD output** to NS3-compatible format
3. **Use the converted trace** in NS3 simulation (optional, if using trace-based mobility)

## Step 1: Run SUMO to Generate FCD Output

### Prerequisites

Ensure SUMO is installed:
```bash
# Check if SUMO is installed
sumo --version

# If not installed, on Ubuntu/Debian:
sudo apt-get install sumo sumo-tools

# On macOS:
brew install sumo
```

### Running SUMO

Navigate to the handover-simulation directory and run SUMO with the configuration file:

```bash
cd /home/ubuntu/cs297/handover-simulation

# Run SUMO (non-GUI mode)
sumo -c urban-scenario.sumocfg

# Or run with GUI to visualize (optional)
sumo-gui -c urban-scenario.sumocfg
```

This will generate the FCD output file `sumo-trace.xml` as specified in the configuration file.

### Understanding the Output

The `sumo-trace.xml` file contains vehicle positions at each timestep:
```xml
<fcd-export>
  <timestep time="0.00">
    <vehicle id="ue0" x="400.00" y="400.00" angle="0.00" speed="10.00"/>
    <vehicle id="ue1" x="401.00" y="400.00" angle="0.00" speed="12.00"/>
    ...
  </timestep>
  <timestep time="0.10">
    ...
  </timestep>
</fcd-export>
```

### SUMO Configuration Parameters

The current configuration (`urban-scenario.sumocfg`) is set to:
- **Simulation time**: 0 to 1000 seconds
- **Step length**: 0.1 seconds (10 Hz update rate)
- **Output**: FCD trace to `sumo-trace.xml`
- **TraCI server**: Port 8813 (for real-time integration, if needed)

You can modify these parameters in `urban-scenario.sumocfg` if needed.

## Step 2: Convert SUMO FCD Output to NS3 Format

Use the `sumo_helper.py` script to convert the FCD output:

```bash
# Convert SUMO trace to NS3 mobility format
python3 sumo_helper.py --convert-trace sumo-trace.xml --output ns3_mobility.tcl
```

This will create `ns3_mobility.tcl` (or your specified output file) with the following format:
```
# NS3 Mobility Trace (converted from SUMO FCD)
# Format: time,node_id,x,y,speed,angle
# Vehicle IDs are mapped to node indices (ue0 -> 0, ue1 -> 1, etc.)

0.00,0,400.00,400.00,10.00,0.00
0.10,0,401.00,400.50,10.00,0.00
...
```

### Conversion Script Options

```bash
# Basic conversion
python3 sumo_helper.py --convert-trace sumo-trace.xml

# Specify custom output file
python3 sumo_helper.py --convert-trace sumo-trace.xml --output my_mobility.tcl

# Get help
python3 sumo_helper.py --help
```

## Step 3: Using SUMO Traces in NS3

### Option A: Direct SUMO Integration (Recommended if available)

If your NS3 installation has SUMO integration support, you can use SUMO directly:

```bash
# Run NS3 simulation with SUMO
./handover-simulation --useSumo=true --sumoConfig=urban-scenario.sumocfg
```

**Note**: This requires NS3's SUMO integration module. If not available, the simulation will fall back to RandomWaypoint mobility.

### Option B: Trace-Based Mobility (Alternative)

If direct SUMO integration is not available, you can use the converted trace file with NS3's trace-based mobility model. This requires modifying the NS3 simulation code to read the trace file.

### Option C: RandomWaypoint Fallback (Current Default)

The current implementation uses RandomWaypoint mobility as a fallback when SUMO integration is not available. This is the default behavior.

## Complete Workflow Example

Here's a complete example of the workflow:

```bash
# 1. Navigate to simulation directory
cd /home/ubuntu/cs297/handover-simulation

# 2. Run SUMO to generate FCD output
sumo -c urban-scenario.sumocfg

# 3. Verify the output was created
ls -lh sumo-trace.xml

# 4. Convert FCD output to NS3 format
python3 sumo_helper.py --convert-trace sumo-trace.xml --output ns3_mobility.tcl

# 5. Verify the converted file
head -20 ns3_mobility.tcl

# 6. (Optional) Run NS3 with SUMO integration
# ./handover-simulation --useSumo=true --sumoConfig=urban-scenario.sumocfg
```

## Troubleshooting

### Issue: SUMO fails to start
**Solution**: 
- Check that all required files exist: `urban-scenario.net.xml`, `urban-scenario.rou.xml`
- Verify SUMO is installed: `sumo --version`
- Check for errors in the configuration file

### Issue: No FCD output generated
**Solution**:
- Check the `urban-scenario.sumocfg` file for the output path
- Ensure SUMO has write permissions in the directory
- Check SUMO output for errors

### Issue: Conversion script fails
**Solution**:
- Verify the FCD file exists and is not empty
- Check that the file is valid XML: `xmllint sumo-trace.xml`
- Ensure Python 3 is installed: `python3 --version`

### Issue: NS3 doesn't recognize SUMO
**Solution**:
- The current NS3 code uses RandomWaypoint as fallback
- For full SUMO integration, you may need to:
  1. Install NS3's SUMO integration module
  2. Use TraCI for real-time integration
  3. Modify the NS3 code to use trace-based mobility

## Advanced: Customizing SUMO Simulation

### Modifying Simulation Duration

Edit `urban-scenario.sumocfg`:
```xml
<time>
    <begin value="0"/>
    <end value="100"/>  <!-- Change to desired duration -->
    <step-length value="0.1"/>
</time>
```

### Changing Output Frequency

To reduce output file size, you can use SUMO's `--fcd-output.period` option:
```bash
sumo -c urban-scenario.sumocfg --fcd-output.period 1.0
```
This outputs positions every 1.0 seconds instead of every 0.1 seconds.

### Adding More Vehicles

Edit `urban-scenario.rou.xml` or use the helper script:
```bash
python3 sumo_helper.py --create-routes --num-ues 20
```

### Changing Network Area

Edit the network file or regenerate:
```bash
python3 sumo_helper.py --create-network \
    --x-min 0 --x-max 2000 \
    --y-min 0 --y-max 2000
```

## File Structure

After running SUMO, you should have:
```
handover-simulation/
├── urban-scenario.sumocfg    # SUMO configuration
├── urban-scenario.net.xml    # Road network
├── urban-scenario.rou.xml     # Vehicle routes
├── sumo-trace.xml            # FCD output (generated by SUMO)
└── ns3_mobility.tcl          # NS3 mobility trace (generated by converter)
```

## References

- SUMO Documentation: https://sumo.dlr.de/docs/
- SUMO FCD Output: https://sumo.dlr.de/docs/Simulation/Output/FCDOutput.html
- NS3 Mobility Models: https://www.nsnam.org/docs/models/html/mobility.html
- NS3 SUMO Integration: https://www.nsnam.org/docs/models/html/sumo.html


