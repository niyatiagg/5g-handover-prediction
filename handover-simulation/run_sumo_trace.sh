#!/bin/bash
# Quick script to run SUMO and generate trace files
# Usage: ./run_sumo_trace.sh [duration_seconds]

DURATION=${1:-1000}  # Default 1000 seconds
CONFIG_FILE="urban-scenario.sumocfg"
OUTPUT_FCD="sumo-fcd-trace.xml"
OUTPUT_CSV="sumo-trace.csv"
OUTPUT_NS3="ns3-mobility.tcl"

echo "=========================================="
echo "SUMO Trace Generation"
echo "=========================================="
echo ""
echo "Duration: $DURATION seconds"
echo "Config: $CONFIG_FILE"
echo ""

# Check if config file exists
if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: Config file not found: $CONFIG_FILE"
    echo "Please create the SUMO configuration file first."
    exit 1
fi

# Check if SUMO is installed
if ! command -v sumo &> /dev/null; then
    echo "Error: SUMO not found. Please install SUMO first."
    echo "  macOS: brew install sumo"
    echo "  Ubuntu: sudo apt-get install sumo"
    exit 1
fi

echo "Step 1: Generating FCD trace (XML format)..."
sumo -c $CONFIG_FILE \
     --fcd-output $OUTPUT_FCD \
     --fcd-output.geo true \
     --end $DURATION \
     --step-length 0.5 \
     --no-warnings \
     --log "sumo_simulation.log"

if [ $? -eq 0 ]; then
    echo "✓ FCD trace generated: $OUTPUT_FCD"
else
    echo "✗ Error generating FCD trace"
    exit 1
fi

echo ""
echo "Step 2: Converting to CSV format..."

# Use Python script if available
if command -v python3 &> /dev/null && [ -f "../sumo_to_ns3_trace.py" ]; then
    python3 ../sumo_to_ns3_trace.py $OUTPUT_FCD $OUTPUT_CSV --csv
    if [ $? -eq 0 ]; then
        echo "✓ CSV trace generated: $OUTPUT_CSV"
    fi
else
    echo "  (Skipping CSV conversion - Python script not found)"
fi

echo ""
echo "Step 3: Converting to ns3 TCL format..."

if command -v python3 &> /dev/null && [ -f "../sumo_to_ns3_trace.py" ]; then
    python3 ../sumo_to_ns3_trace.py $OUTPUT_FCD $OUTPUT_NS3
    if [ $? -eq 0 ]; then
        echo "✓ ns3 TCL trace generated: $OUTPUT_NS3"
    fi
else
    echo "  (Skipping ns3 conversion - Python script not found)"
fi

echo ""
echo "=========================================="
echo "Trace Generation Complete!"
echo "=========================================="
echo ""
echo "Generated files:"
if [ -f "$OUTPUT_FCD" ]; then
    echo "  ✓ $OUTPUT_FCD ($(wc -l < $OUTPUT_FCD | tr -d ' ') lines)"
fi
if [ -f "$OUTPUT_CSV" ]; then
    echo "  ✓ $OUTPUT_CSV ($(wc -l < $OUTPUT_CSV | tr -d ' ') lines)"
fi
if [ -f "$OUTPUT_NS3" ]; then
    echo "  ✓ $OUTPUT_NS3 ($(wc -l < $OUTPUT_NS3 | tr -d ' ') lines)"
fi
echo ""
echo "To use with TraCI for real-time integration:"
echo "  python3 sumo_traci_trace.py $CONFIG_FILE traci-trace.csv $DURATION"

