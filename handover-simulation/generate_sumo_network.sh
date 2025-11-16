#!/bin/bash
# Generate SUMO network for handover simulation
# Area: X[175m,1250m], Y[230m,830m]

NET_FILE="urban-scenario.net.xml"

echo "Generating SUMO network..."
echo "Area: X[175m,1250m], Y[230m,830m]"

# Create a grid network
netgenerate --grid \
  --grid.x-number=4 \
  --grid.y-number=3 \
  --grid.length=300 \
  --grid.x-length=1075 \
  --grid.y-length=600 \
  --default.speed=60 \
  --default.lanenumber=1 \
  --offset.x=175 \
  --offset.y=230 \
  --no-turnarounds=false \
  --no-left-connections=false \
  --output-file=$NET_FILE

if [ $? -eq 0 ]; then
    echo "✓ Network generated: $NET_FILE"
    echo ""
    echo "To validate the network, run:"
    echo "  netconvert --xml-validation never -n $NET_FILE"
else
    echo "✗ Error generating network"
    exit 1
fi

