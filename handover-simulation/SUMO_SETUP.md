# SUMO Setup and Trace Generation Guide

This guide explains how to run SUMO and generate trace files for use with the ns3 handover simulation.

## Prerequisites

1. **Install SUMO**:
   ```bash
   # macOS
   brew install sumo
   
   # Ubuntu/Debian
   sudo apt-get install sumo sumo-tools sumo-gui
   
   # Verify installation
   sumo --version
   ```

2. **Install Python dependencies** (for trace conversion):
   ```bash
   pip install traci sumolib lxml
   ```

## Step 1: Create SUMO Network File

Create a network file that defines the road topology. For our simulation area (X[175m,1250m], Y[230m,830m]):

```bash
# Create a simple road network
netgenerate --grid --grid.x-number=4 --grid.y-number=3 \
  --grid.length=300 --grid.length.total=false \
  --default.speed=60 --default.lanenum=1 \
  --output-file=urban-scenario.net.xml \
  --begin=175 --end=1250 \
  --begin=230 --end=830
```

Or use a predefined network file (see `urban-scenario.net.xml`).

## Step 2: Create Routes File

Create a routes file defining vehicle movements:

```xml
<!-- urban-scenario.rou.xml -->
<?xml version="1.0" encoding="UTF-8"?>
<routes>
    <vType id="ue" vClass="passenger" maxSpeed="60.00" minGap="2.50" 
           accel="2.60" decel="4.50" sigma="0.5" tau="1.0"/>
    
    <route id="r1" edges="edge0 edge1 edge2"/>
    <route id="r2" edges="edge1 edge2 edge3"/>
    <!-- Add more routes as needed -->
    
    <vehicle id="ue0" type="ue" route="r1" depart="0.00" 
             departLane="best" departPos="random" departSpeed="random"/>
    <vehicle id="ue1" type="ue" route="r2" depart="1.00" 
             departLane="best" departPos="random" departSpeed="random"/>
    <!-- Add more vehicles (10 total for the simulation) -->
</routes>
```

## Step 3: Create SUMO Configuration File

```xml
<!-- urban-scenario.sumocfg -->
<?xml version="1.0" encoding="UTF-8"?>
<configuration>
    <input>
        <net-file value="urban-scenario.net.xml"/>
        <route-files value="urban-scenario.rou.xml"/>
    </input>
    
    <output>
        <fcd-output value="sumo-trace.xml"/>
        <fcd-output.geo value="true"/>
    </output>
    
    <time>
        <begin value="0"/>
        <end value="1000"/>
        <step-length value="0.1"/>
    </time>
    
    <report>
        <verbose value="true"/>
        <no-warnings value="false"/>
    </report>
    
    <traci_server>
        <remote-port value="8813"/>
    </traci_server>
</configuration>
```

## Step 4: Run SUMO and Generate Trace File

### Method 1: Generate FCD (Floating Car Data) Trace

```bash
# Run SUMO and generate FCD trace
sumo -c urban-scenario.sumocfg --fcd-output sumo-fcd-trace.xml

# This generates an XML file with position and speed data for all vehicles
# Output format: sumo-fcd-trace.xml
```

### Method 2: Generate Full Data (FCD) with Detailed Information

```bash
# Generate FCD with more details (position, speed, acceleration, angle)
sumo -c urban-scenario.sumocfg \
     --fcd-output sumo-fcd-trace.xml \
     --fcd-output.attributes time,id,x,y,angle,type,speed,pos,lon,lat \
     --fcd-output.geo true
```

### Method 3: Generate Trace with Specific Vehicle IDs

```bash
# Generate trace only for specific vehicles (UEs)
sumo -c urban-scenario.sumocfg \
     --fcd-output sumo-fcd-trace.xml \
     --fcd-output.filter-edges.input-file vehicle-ids.txt
```

## Step 5: Convert SUMO Trace to ns3 Format

The ns3 simulation needs the trace in a specific format. Here's a Python script to convert:

```python
# sumo_to_ns3_trace.py
import xml.etree.ElementTree as ET
import sys

def convert_sumo_to_ns3(sumo_fcd_file, output_file):
    """Convert SUMO FCD trace to ns3 mobility trace format"""
    tree = ET.parse(sumo_fcd_file)
    root = tree.getroot()
    
    # Group timesteps by vehicle
    vehicle_data = {}
    
    for timestep in root.findall('timestep'):
        time = float(timestep.get('time'))
        for vehicle in timestep.findall('vehicle'):
            veh_id = vehicle.get('id')
            x = float(vehicle.get('x'))
            y = float(vehicle.get('y'))
            speed = float(vehicle.get('speed'))
            angle = float(vehicle.get('angle'))
            
            if veh_id not in vehicle_data:
                vehicle_data[veh_id] = []
            
            vehicle_data[veh_id].append({
                'time': time,
                'x': x,
                'y': y,
                'speed': speed,
                'angle': angle
            })
    
    # Write ns3 trace format (TCL format for ns3)
    with open(output_file, 'w') as f:
        # Write header
        f.write("# SUMO trace converted to ns3 format\n")
        f.write("# Vehicle ID mappings:\n")
        
        # Map vehicle IDs to node IDs (assuming sequential mapping)
        veh_ids = sorted(vehicle_data.keys())
        for idx, veh_id in enumerate(veh_ids):
            f.write(f"# {veh_id} -> Node {idx}\n")
        
        # Write mobility commands
        f.write("# Format: $node($id) set X_ Y_ Z_ at time\n")
        f.write("#         $node($id) setdest X_ Y_ speed at time\n\n")
        
        for idx, veh_id in enumerate(veh_ids):
            data = sorted(vehicle_data[veh_id], key=lambda x: x['time'])
            
            for i, point in enumerate(data):
                time = point['time']
                x = point['x']
                y = point['y']
                speed = point['speed']
                angle = point['angle']
                
                # Convert angle to direction vector
                import math
                dx = speed * math.cos(math.radians(angle))
                dy = speed * math.sin(math.radians(angle))
                
                if i == 0:
                    # Initial position
                    f.write(f"$node_({idx}) set X_ {x}\n")
                    f.write(f"$node_({idx}) set Y_ {y}\n")
                    f.write(f"$node_({idx}) set Z_ 1.5\n")
                else:
                    # Movement command (setdest: x, y, speed)
                    # Calculate target position
                    dt = point['time'] - data[i-1]['time'] if i > 0 else 0.1
                    target_x = x + dx * dt
                    target_y = y + dy * dt
                    
                    f.write(f"$ns_ at {time} \"$node_({idx}) setdest {target_x} {target_y} {speed}\"\n")
    
    print(f"Converted {len(vehicle_data)} vehicles to {output_file}")

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python sumo_to_ns3_trace.py <sumo_fcd_file> <output_file>")
        sys.exit(1)
    
    convert_sumo_to_ns3(sys.argv[1], sys.argv[2])
```

## Step 6: Alternative - Use SUMO with TraCI (Real-time Integration)

For real-time integration with ns3, you can use TraCI (Traffic Control Interface):

### Method A: Python Script for SUMO TraCI

```python
# sumo_traci_trace.py
import traci
import sumolib
import sys

def run_sumo_with_traci(config_file, output_file):
    """Run SUMO with TraCI and generate trace"""
    sumoCmd = [
        sumolib.checkBinary('sumo'),
        '-c', config_file,
        '--step-length', '0.1',
        '--start',
        '--quit-on-end',
        '--no-warnings'
    ]
    
    traci.start(sumoCmd)
    
    # Open output file
    with open(output_file, 'w') as f:
        f.write("# SUMO TraCI trace\n")
        f.write("# Time,VehicleID,X,Y,Speed,Angle\n")
        
        step = 0
        while step < 10000:  # Run for 1000 seconds (10000 * 0.1s)
            traci.simulationStep()
            time = traci.simulation.getTime()
            
            # Get all vehicle IDs
            vehicle_ids = traci.vehicle.getIDList()
            
            for veh_id in vehicle_ids:
                try:
                    pos = traci.vehicle.getPosition(veh_id)
                    speed = traci.vehicle.getSpeed(veh_id)
                    angle = traci.vehicle.getAngle(veh_id)
                    
                    f.write(f"{time},{veh_id},{pos[0]},{pos[1]},{speed},{angle}\n")
                except traci.exceptions.TraCIException:
                    continue  # Vehicle may have left the simulation
            
            step += 1
    
    traci.close()
    print(f"Trace generated: {output_file}")

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python sumo_traci_trace.py <config_file> <output_file>")
        sys.exit(1)
    
    run_sumo_with_traci(sys.argv[1], sys.argv[2])
```

### Run with TraCI:
```bash
python sumo_traci_trace.py urban-scenario.sumocfg sumo-trace.csv
```

## Step 7: Integrate with ns3 Simulation

### Option 1: Use SUMO Trace Directly in ns3

If you have ns3's SUMO integration module installed:

1. Convert SUMO trace to ns3 mobility trace format
2. Use `ns3::MobilityHelper` to load the trace:

```cpp
// In your ns3 simulation code
MobilityHelper mobility;
mobility.SetMobilityModel("ns3::MobilityTraceHelper");
mobility.Install(ueNodes);
```

### Option 2: Use TraCI for Real-time Integration

Run SUMO and ns3 in parallel, using TraCI to exchange data in real-time:

1. Start SUMO with TraCI server:
   ```bash
   sumo -c urban-scenario.sumocfg --remote-port=8813
   ```

2. In ns3 code, connect to SUMO via TraCI (requires ns3 SUMO integration module)

## Quick Start: Complete Example

Here's a complete example to generate a SUMO trace:

```bash
# 1. Create network (if not already created)
netgenerate --grid --grid.x-number=4 --grid.y-number=3 \
  --grid.length=300 --output-file=urban-scenario.net.xml

# 2. Generate routes for 10 vehicles
# (You may need to create routes.xml manually based on your network)

# 3. Create SUMO config file (see Step 3 above)
# Save as urban-scenario.sumocfg

# 4. Run SUMO and generate FCD trace
sumo -c urban-scenario.sumocfg --fcd-output sumo-fcd-trace.xml

# 5. Convert to ns3 format
python sumo_to_ns3_trace.py sumo-fcd-trace.xml ns3-mobility.tcl

# 6. Use in ns3 simulation
# Load the TCL file or integrate directly
```

## Output File Formats

### SUMO FCD Output (XML):
```xml
<fcd-export>
    <timestep time="0.00">
        <vehicle id="ue0" x="400.00" y="300.00" angle="90.00" 
                 type="ue" speed="25.50" pos="100.00" lane="edge0_0" 
                 slope="0.00" lon="..." lat="..."/>
    </timestep>
    ...
</fcd-export>
```

### CSV Format (TraCI):
```csv
Time,VehicleID,X,Y,Speed,Angle
0.00,ue0,400.00,300.00,25.50,90.00
0.10,ue0,402.55,300.00,25.50,90.00
...
```

### ns3 TCL Format:
```tcl
$node_(0) set X_ 400.00
$node_(0) set Y_ 300.00
$node_(0) set Z_ 1.5
$ns_ at 0.10 "$node_(0) setdest 402.55 300.00 25.50"
...
```

## Troubleshooting

1. **SUMO not found**: Ensure SUMO is in your PATH
   ```bash
   export SUMO_HOME=/path/to/sumo
   export PATH=$PATH:$SUMO_HOME/bin
   ```

2. **Network file errors**: Validate your network file
   ```bash
   netconvert --xml-validation never -n urban-scenario.net.xml
   ```

3. **No vehicles in trace**: Check that routes match network edges
   ```bash
   sumo -c urban-scenario.sumocfg --summary-output summary.xml
   ```

4. **TraCI connection errors**: Ensure SUMO is running with TraCI server enabled

## Additional Resources

- SUMO Documentation: https://sumo.dlr.de/docs/
- TraCI Documentation: https://sumo.dlr.de/docs/TraCI.html
- ns3 SUMO Integration: https://www.nsnam.org/docs/models/html/sumo-mobility.html

