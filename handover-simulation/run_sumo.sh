#!/bin/bash
# Quick script to run SUMO and convert output for NS3 simulation

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=========================================="
echo "SUMO to NS3 Workflow"
echo "=========================================="
echo ""

# Check if SUMO is installed
if ! command -v sumo &> /dev/null; then
    echo "Error: SUMO is not installed or not in PATH"
    echo "Please install SUMO: sudo apt-get install sumo sumo-tools"
    exit 1
fi

# Check if Python 3 is installed
if ! command -v python3 &> /dev/null; then
    echo "Error: Python 3 is not installed"
    exit 1
fi

# Step 1: Run SUMO
echo "Step 1: Running SUMO to generate FCD output..."
if [ ! -f "urban-scenario.sumocfg" ]; then
    echo "Error: urban-scenario.sumocfg not found!"
    exit 1
fi

sumo -c urban-scenario.sumocfg

if [ ! -f "sumo-trace.xml" ]; then
    echo "Error: SUMO did not generate sumo-trace.xml"
    exit 1
fi

echo "✓ SUMO completed. Generated sumo-trace.xml"
echo ""

# Step 2: Convert to NS3 format
echo "Step 2: Converting SUMO FCD output to NS3 format..."
python3 sumo_helper.py --convert-trace sumo-trace.xml --output ns3_mobility.tcl

if [ ! -f "ns3_mobility.tcl" ]; then
    echo "Error: Conversion failed - ns3_mobility.tcl not created"
    exit 1
fi

echo "✓ Conversion completed. Generated ns3_mobility.tcl"
echo ""

# Summary
echo "=========================================="
echo "Summary"
echo "=========================================="
echo "Generated files:"
ls -lh sumo-trace.xml ns3_mobility.tcl 2>/dev/null | awk '{print "  " $9 " (" $5 ")"}'
echo ""
echo "Next steps:"
echo "  1. Review the converted trace: head -20 ns3_mobility.tcl"
echo "  2. Use in NS3 simulation (if SUMO integration is available):"
echo "     ./handover-simulation --useSumo=true --sumoConfig=urban-scenario.sumocfg"
echo ""
echo "For more details, see SUMO_GUIDE.md"


