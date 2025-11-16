#!/usr/bin/env python3
"""
Convert SUMO FCD trace to ns3 mobility trace format
Usage: python sumo_to_ns3_trace.py <sumo_fcd_file> <output_file>
"""

import xml.etree.ElementTree as ET
import sys
import math

def convert_sumo_to_ns3(sumo_fcd_file, output_file):
    """Convert SUMO FCD trace to ns3 mobility trace format"""
    print(f"Reading SUMO FCD file: {sumo_fcd_file}")
    
    try:
        tree = ET.parse(sumo_fcd_file)
        root = tree.getroot()
    except ET.ParseError as e:
        print(f"Error parsing XML file: {e}")
        sys.exit(1)
    except FileNotFoundError:
        print(f"Error: File not found: {sumo_fcd_file}")
        sys.exit(1)
    
    # Group timesteps by vehicle
    vehicle_data = {}
    
    print("Processing vehicle data...")
    timestep_count = 0
    
    # Handle both <fcd-export> and <timestep> root elements
    if root.tag == 'fcd-export':
        timesteps = root.findall('timestep')
    else:
        timesteps = [root] if root.tag == 'timestep' else root.findall('timestep')
    
    for timestep in timesteps:
        time = float(timestep.get('time'))
        timestep_count += 1
        
        for vehicle in timestep.findall('vehicle'):
            veh_id = vehicle.get('id')
            x = float(vehicle.get('x'))
            y = float(vehicle.get('y'))
            speed = float(vehicle.get('speed', 0.0))
            angle = float(vehicle.get('angle', 0.0))
            
            if veh_id not in vehicle_data:
                vehicle_data[veh_id] = []
            
            vehicle_data[veh_id].append({
                'time': time,
                'x': x,
                'y': y,
                'speed': speed,
                'angle': angle
            })
    
    print(f"Processed {timestep_count} timesteps")
    print(f"Found {len(vehicle_data)} vehicles")
    
    # Write ns3 trace format (TCL format for ns3)
    print(f"Writing ns3 trace file: {output_file}")
    
    with open(output_file, 'w') as f:
        # Write header
        f.write("# SUMO trace converted to ns3 format\n")
        f.write("# Generated from: " + sumo_fcd_file + "\n")
        f.write("# Vehicle ID mappings:\n")
        
        # Map vehicle IDs to node IDs (assuming sequential mapping)
        veh_ids = sorted(vehicle_data.keys())
        for idx, veh_id in enumerate(veh_ids):
            f.write(f"# {veh_id} -> Node {idx}\n")
        
        f.write("\n")
        f.write("# Format: $node($id) set X_ Y_ Z_ at time\n")
        f.write("#         $node($id) setdest X_ Y_ speed at time\n\n")
        
        # Write initial positions and movements
        for idx, veh_id in enumerate(veh_ids):
            data = sorted(vehicle_data[veh_id], key=lambda x: x['time'])
            
            if len(data) == 0:
                continue
            
            # Initial position
            first_point = data[0]
            f.write(f"$node_({idx}) set X_ {first_point['x']:.2f}\n")
            f.write(f"$node_({idx}) set Y_ {first_point['y']:.2f}\n")
            f.write(f"$node_({idx}) set Z_ 1.5\n")
            f.write(f"$ns_ at {first_point['time']:.2f} \"$node_({idx}) setdest {first_point['x']:.2f} {first_point['y']:.2f} {first_point['speed']:.2f}\"\n")
            
            # Subsequent movements
            for i in range(1, len(data)):
                point = data[i]
                prev_point = data[i-1]
                
                # Calculate target position based on current position and speed
                dt = point['time'] - prev_point['time']
                if dt > 0 and point['speed'] > 0:
                    # Convert angle from degrees to radians
                    angle_rad = math.radians(point['angle'])
                    dx = point['speed'] * dt * math.cos(angle_rad)
                    dy = point['speed'] * dt * math.sin(angle_rad)
                    
                    target_x = point['x']
                    target_y = point['y']
                    
                    f.write(f"$ns_ at {point['time']:.2f} \"$node_({idx}) setdest {target_x:.2f} {target_y:.2f} {point['speed']:.2f}\"\n")
    
    print(f"✓ Successfully converted {len(vehicle_data)} vehicles")
    print(f"✓ Output written to: {output_file}")

def convert_to_csv(sumo_fcd_file, output_csv):
    """Convert SUMO FCD trace to CSV format"""
    print(f"Reading SUMO FCD file: {sumo_fcd_file}")
    
    try:
        tree = ET.parse(sumo_fcd_file)
        root = tree.getroot()
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
    
    # Handle both <fcd-export> and <timestep> root elements
    if root.tag == 'fcd-export':
        timesteps = root.findall('timestep')
    else:
        timesteps = [root] if root.tag == 'timestep' else root.findall('timestep')
    
    print(f"Writing CSV file: {output_csv}")
    
    with open(output_csv, 'w') as f:
        # Write CSV header
        f.write("Time,VehicleID,X,Y,Speed,Angle\n")
        
        for timestep in timesteps:
            time = float(timestep.get('time'))
            
            for vehicle in timestep.findall('vehicle'):
                veh_id = vehicle.get('id')
                x = float(vehicle.get('x'))
                y = float(vehicle.get('y'))
                speed = float(vehicle.get('speed', 0.0))
                angle = float(vehicle.get('angle', 0.0))
                
                f.write(f"{time:.2f},{veh_id},{x:.2f},{y:.2f},{speed:.2f},{angle:.2f}\n")
    
    print(f"✓ CSV file written to: {output_csv}")

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage:")
        print("  python sumo_to_ns3_trace.py <sumo_fcd_file> <output_tcl_file>")
        print("  python sumo_to_ns3_trace.py <sumo_fcd_file> <output_csv_file> --csv")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    
    if len(sys.argv) > 3 and sys.argv[3] == '--csv':
        convert_to_csv(input_file, output_file)
    else:
        convert_sumo_to_ns3(input_file, output_file)

