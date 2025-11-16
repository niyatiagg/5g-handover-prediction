# Makefile for building handover simulation
# This assumes ns3 is installed and available in the system

NS3_DIR ?= /usr/local/ns3
NS3_BUILD_DIR ?= $(NS3_DIR)/build

CXX = g++
CXXFLAGS = -Wall -O2 -std=c++11
INCLUDES = -I$(NS3_DIR)/build/include
LIBS = -L$(NS3_BUILD_DIR)/lib \
       -lns3.36-core-debug \
       -lns3.36-network-debug \
       -lns3.36-mobility-debug \
       -lns3.36-lte-debug \
       -lns3.36-internet-debug \
       -lns3.36-applications-debug \
       -lns3.36-point-to-point-debug \
       -lns3.36-flow-monitor-debug \
       -lns3.36-config-store-debug

TARGET = handover-simulation
SOURCE = handover-simulation.cc

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(SOURCE)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(TARGET) $(SOURCE) $(LIBS)

clean:
	rm -f $(TARGET) *.csv *.pcap

run: $(TARGET)
	./$(TARGET)

help:
	@echo "Available targets:"
	@echo "  all   - Build the simulation"
	@echo "  clean - Remove built files"
	@echo "  run   - Build and run the simulation"
	@echo ""
	@echo "Note: This Makefile is a template. You may need to adjust"
	@echo "      NS3_DIR and library names based on your ns3 installation."

