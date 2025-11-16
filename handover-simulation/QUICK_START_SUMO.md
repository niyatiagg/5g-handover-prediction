# Quick Start: Generate SUMO Trace Files

This is a quick reference for generating SUMO trace files for the handover simulation.

## Prerequisites

```bash
# Install SUMO
brew install sumo  # macOS
# or
sudo apt-get install sumo sumo-tools  # Ubuntu

# Install Python dependencies
pip install traci sumolib lxml
```

## Quick Steps

### Step 1: Generate SUMO Network

```bash
# Generate the road network
./generate_sumo_network.sh

# This creates: urban-scenario.net.xml
```

### Step 2: Create SUMO Configuration File

Create `urban-scenario.sumocfg`:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<configuration>
    <input>
        <net-file value="urban-scenario.net.xml"/>
        <route-files value="urban-scenario.rou.xml"/>
    </input>
    
    <output>
        <fcd-output value="sumo-fcd-trace.xml"/>
        <fcd-output.geo value="true"/>
    </output>
    
    <time>
        <begin value="0"/>
        <end value="1000"/>
        <step-length value="0.1"/>
    </time>
    
    <traci_server>
        <remote-port value="8813"/>
    </traci_server>
</configuration>
```

### Step 3: Create Routes File

Create `urban-scenario.rou.xml` with 10 vehicles (UEs):

```xml
<?xml version="1.0" encoding="UTF-8"?>
<routes>
    <vType id="ue" vClass="passenger" maxSpeed="60.00" 
           minGap="2.50" accel="2.60" decel="4.50" 
           sigma="0.5" tau="1.0"/>
    
    <!-- Define routes based on your network edges -->
    <route id="r1" edges="edge0 edge1 edge2"/>
    <route id="r2" edges="edge1 edge2 edge3"/>
    <!-- Add more routes... -->
    
    <!-- Add 10 vehicles -->
    <vehicle id="ue0" type="ue" route="r1" depart="0.00" 
             departLane="best" departPos="random" departSpeed="random"/>
    <vehicle id="ue1" type="ue" route="r2" depart="1.00" 
             departLane="best" departPos="random" departSpeed="random"/>
    <!-- Add vehicles ue2 through ue9... -->
</routes>
```

### Step 4: Generate Trace Files

```bash
# Option A: Use the automated script
./run_sumo_trace.sh 1000  # Duration in seconds

# Option B: Generate FCD trace manually
sumo -c urban-scenario.sumocfg --fcd-output sumo-fcd-trace.xml

# Option C: Use TraCI for CSV output
python3 sumo_traci_trace.py urban-scenario.sumocfg sumo-trace.csv 1000
```

### Step 5: Convert to ns3 Format

```bash
# Convert FCD XML to ns3 TCL format
python3 sumo_to_ns3_trace.py sumo-fcd-trace.xml ns3-mobility.tcl

# Convert to CSV format
python3 sumo_to_ns3_trace.py sumo-fcd-trace.xml sumo-trace.csv --csv
```

## Output Files

After running the scripts, you'll get:

1. **sumo-fcd-trace.xml** - SUMO FCD (Floating Car Data) trace in XML format
2. **sumo-trace.csv** - CSV trace with columns: Time,VehicleID,X,Y,Speed,Angle
3. **ns3-mobility.tcl** - ns3 mobility trace in TCL format

## Example Output

### CSV Format:
```csv
Time,VehicleID,X,Y,Speed,Angle
0.00,ue0,400.00,300.00,25.50,90.00
0.10,ue0,402.55,300.00,25.50,90.00
0.20,ue0,405.10,300.00,25.50,90.00
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

## Using Traces with ns3

### Method 1: Load TCL Trace File
In your ns3 simulation code, you can load the TCL trace file using `MobilityHelper`:

```cpp
MobilityHelper mobility;
mobility.SetMobilityModel("ns3::MobilityTraceHelper");
mobility.Install(ueNodes);
```

### Method 2: Parse CSV in ns3
You can parse the CSV file and set UE positions programmatically in your ns3 code.

### Method 3: Real-time TraCI Integration
For real-time integration, run SUMO and ns3 in parallel:

```bash
# Terminal 1: Start SUMO with TraCI server
sumo -c urban-scenario.sumocfg --remote-port=8813

# Terminal 2: Run ns3 simulation (with TraCI client)
./your-ns3-simulation
```

## Troubleshooting

1. **SUMO not found**: 
   ```bash
   export SUMO_HOME=/usr/local  # or where SUMO is installed
   export PATH=$PATH:$SUMO_HOME/bin
   ```

2. **Network file errors**: 
   ```bash
   netconvert --xml-validation never -n urban-scenario.net.xml
   ```

3. **No vehicles in output**: Check that routes match network edges:
   ```bash
   sumo -c urban-scenario.sumocfg --summary-output summary.xml
   ```

4. **Python import errors**: 
   ```bash
   pip install traci sumolib lxml
   ```

## Full Example Command Sequence

```bash
# 1. Generate network
./generate_sumo_network.sh

# 2. Create routes file (manual or use randomTrips.py)
# (Create urban-scenario.rou.xml with 10 vehicles)

# 3. Create config file
# (Create urban-scenario.sumocfg)

# 4. Generate traces
./run_sumo_trace.sh 1000

# 5. Convert to ns3 format
python3 sumo_to_ns3_trace.py sumo-fcd-trace.xml ns3-mobility.tcl

# 6. Use in ns3 simulation
# Load ns3-mobility.tcl in your simulation code
```

## More Information

See `SUMO_SETUP.md` for detailed documentation and advanced options.

