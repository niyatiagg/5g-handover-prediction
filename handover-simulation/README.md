# 5G Handover Simulation using ns3 and SUMO

This simulation generates a dataset for analyzing handovers in 5G networks, based on the research paper:
**"Reducing Unnecessary Handovers Using Logistic Regression in 5G Networks"** by ALISON M. FERNANDES, HERMES I. DEL MONEGO, BRUNO S. CHANG, AND ANELISE MUNARETTO.

## Overview

This simulation uses ns3 and SUMO to model:
- 8 gNBs (base stations) in a predefined urban area
- 10 UEs (User Equipment) with mobile movement
- Handover events with detailed logging
- CBR (Constant Bit Rate) traffic pattern
- Urban mobility using SUMO or RandomWaypoint model

## Simulation Parameters

Based on the research paper's Table 2:

| Parameter | Value |
|-----------|-------|
| Number of UEs | 10 |
| Number of gNBs | 8 |
| UE Tx Power | 26 dBm |
| gNB Tx Power | 46 dBm |
| Area | X[175m, 1250m], Y[230m, 830m] |
| Mobility Model | RandomWaypoint (or SUMO) |
| Speed | Uniform(10 m/s, 60 m/s) |
| Handover | Enabled (A3 RSRP algorithm) |
| Traffic | CBR (Constant Bit Rate) |

## Prerequisites

### 1. Install ns3

```bash
# Download ns3 (version 3.36 or later)
wget https://www.nsnam.org/releases/ns-allinone-3.36.tar.bz2
tar -xjf ns-allinone-3.36.tar.bz2
cd ns-allinone-3.36
./build.py
```

### 2. Install SUMO (Optional, for urban mobility)

```bash
# On macOS
brew install sumo

# On Ubuntu/Debian
sudo apt-get install sumo sumo-tools

# On Fedora
sudo dnf install sumo
```

### 3. Verify Installation

```bash
# Check ns3
ns3 --version

# Check SUMO
sumo --version
```

## Building the Simulation

**Note**: NS-3.36 uses **CMake** (not waf). For older versions using waf, see the documentation.

### Method 1: Using ns3's CMake build system (Recommended for NS-3.36+)

1. Ensure the simulation file is in your ns3 scratch directory:
```bash
# The file should already be at:
# /path/to/ns3/scratch/handover-simulation.cc
```

2. Configure and build using CMake:
```bash
cd /path/to/ns3
cmake -S . -B build -DNS3_EXAMPLES=ON -DNS3_TESTS=ON
cmake --build build
```

3. Run the simulation:
```bash
# Executable is named scratch_handover-simulation (with scratch_ prefix)
./build/scratch/scratch_handover-simulation --numUes=10 --simTime=100
```

For detailed CMake build instructions, see [BUILD_AND_RUN.md](BUILD_AND_RUN.md).

### Method 2: Using waf (NS-3.35 and earlier)

For NS-3.35 and earlier versions that use waf:

1. Copy the simulation files to your ns3 scratch directory:
```bash
cp handover-simulation.cc /path/to/ns3/scratch/
```

2. Build using waf:
```bash
cd /path/to/ns3
./waf --build-profile=debug --enable-examples --enable-tests
./waf --run handover-simulation
```

### Method 3: Using Makefile (Alternative)

1. Edit the `Makefile` to set your ns3 installation path:
```bash
NS3_DIR=/path/to/ns3
```

2. Build:
```bash
make
./handover-simulation
```

### Method 3: Manual compilation

```bash
g++ -std=c++11 -I/path/to/ns3/build/include \
    handover-simulation.cc \
    -L/path/to/ns3/build/lib \
    -lns3.36-core-debug -lns3.36-network-debug \
    -lns3.36-mobility-debug -lns3.36-lte-debug \
    -lns3.36-internet-debug -lns3.36-applications-debug \
    -lns3.36-point-to-point-debug -lns3.36-flow-monitor-debug \
    -o handover-simulation
```

## Running the Simulation

### Basic usage (NS-3.36 with CMake):
```bash
cd /path/to/ns3
./build/scratch/scratch_handover-simulation
```

### With custom parameters:
```bash
./build/scratch/scratch_handover-simulation --numUes=10 --numGnbs=8 --simTime=100 \
                      --ueTxPower=26 --gnbTxPower=46
```

### Available command-line arguments:
- `--numUes`: Number of UEs (default: 10)
- `--numGnbs`: Number of gNBs (default: 8)
- `--simTime`: Simulation time in seconds (default: 100)
- `--useSumo`: Use SUMO for mobility (default: false)
- `--sumoConfig`: SUMO configuration file (default: urban-scenario.sumocfg)
- `--ueTxPower`: UE transmit power in dBm (default: 26)
- `--gnbTxPower`: gNB transmit power in dBm (default: 46)

## Output Files

The simulation generates two CSV files:

### 1. `handover_dataset.csv`
Contains handover events and periodic measurements with the following columns:
- `Time`: Simulation time in seconds
- `UE_ID`: User Equipment identifier (IMSI)
- `Old_gNB_ID`: Previous gNB cell ID
- `New_gNB_ID`: New gNB cell ID
- `RSRP_Old`: Reference Signal Received Power (old cell) in dBm
- `RSRP_New`: Reference Signal Received Power (new cell) in dBm
- `RSRQ_Old`: Reference Signal Received Quality (old cell) in dB
- `RSRQ_New`: Reference Signal Received Quality (new cell) in dB
- `SINR_Old`: Signal-to-Interference-plus-Noise Ratio (old cell) in dB
- `SINR_New`: Signal-to-Interference-plus-Noise Ratio (new cell) in dB
- `Throughput_DL`: Downlink throughput in Mbps
- `Throughput_UL`: Uplink throughput in Mbps
- `X_Position`: UE X coordinate in meters
- `Y_Position`: UE Y coordinate in meters
- `Speed`: UE speed in m/s
- `Handover_Type`: Event type (HANDOVER or MEASUREMENT)

### 2. `flow_statistics.csv`
Contains traffic flow statistics:
- `Flow_ID`: Flow identifier
- `Source`: Source IP address
- `Destination`: Destination IP address
- `Throughput_DL_Mbps`: Downlink throughput
- `Throughput_UL_Mbps`: Uplink throughput
- `Packets_Sent`: Number of packets sent
- `Packets_Received`: Number of packets received
- `Packets_Lost`: Number of lost packets
- `Delay_Mean_ms`: Mean delay in milliseconds
- `Jitter_ms`: Jitter in milliseconds

## SUMO Integration

To use SUMO for urban mobility:

1. Ensure SUMO is installed and in your PATH
2. The SUMO configuration files are provided:
   - `urban-scenario.sumocfg`: Main SUMO configuration
   - `urban-scenario.net.xml`: Road network
   - `urban-scenario.rou.xml`: Vehicle routes

3. Run with SUMO:
```bash
# With SUMO trace (recommended for NS-3.36)
./build/scratch/scratch_handover-simulation --sumoTrace=ns3-mobility.tcl --numUes=10 --simTime=100

# Real-time SUMO integration (if available)
./build/scratch/scratch_handover-simulation --useSumo=true --sumoConfig=urban-scenario.sumocfg
```

**Note**: Full SUMO integration may require additional ns3 modules or custom integration code. The current implementation uses RandomWaypoint mobility as a fallback.

## Data Analysis

The generated CSV files can be analyzed using:
- Python (pandas, numpy, matplotlib)
- R
- MATLAB
- Excel

Example Python analysis:
```python
import pandas as pd
import matplotlib.pyplot as plt

# Load handover data
df = pd.read_csv('handover_dataset.csv')

# Filter handover events
handovers = df[df['Handover_Type'] == 'HANDOVER']

# Analyze handover frequency
print(f"Total handovers: {len(handovers)}")
print(f"Average handovers per UE: {len(handovers) / df['UE_ID'].nunique()}")

# Plot handover events over time
plt.figure(figsize=(10, 6))
plt.scatter(handovers['Time'], handovers['UE_ID'])
plt.xlabel('Time (s)')
plt.ylabel('UE ID')
plt.title('Handover Events Over Time')
plt.grid(True)
plt.savefig('handover_timeline.png')
```

## Troubleshooting

### Issue: Cannot find ns3 headers
**Solution**: Ensure ns3 is properly installed and set the correct include paths in your build configuration.

### Issue: SUMO integration not working
**Solution**: The current implementation uses RandomWaypoint as default. For full SUMO integration, you may need to:
1. Install ns3's SUMO integration module
2. Use TraCI interface for real-time SUMO control
3. Modify the mobility model initialization

### Issue: No handover events in output
**Solution**: 
- Increase simulation time
- Reduce handover hysteresis (currently 3 dB)
- Increase UE mobility speed
- Adjust gNB positions to create more overlapping coverage

### Issue: Compilation errors
**Solution**: 
- Check ns3 version compatibility
- Ensure all required ns3 modules are enabled
- Verify compiler supports C++11 standard

## References

1. Fernandes, A. M., et al. "Reducing Unnecessary Handovers Using Logistic Regression in 5G Networks"
2. ns3 Documentation: https://www.nsnam.org/documentation/
3. SUMO Documentation: https://sumo.dlr.de/docs/
4. 3GPP Technical Reports:
   - TR 38.300: NR and NG-RAN Overall Description
   - TR 38.211: Physical channels and modulation
   - TR 38.801: Study on new radio access technology

## License

This simulation code is provided for research and educational purposes.

## Contact

For questions or issues, please refer to the original research paper or ns3 documentation.

