#!/usr/bin/env python3
"""
SUMO Helper Script for ns3 Integration
This script helps generate SUMO configuration files and convert SUMO traces
to a format compatible with ns3 mobility models
"""

import xml.etree.ElementTree as ET
import numpy as np
import argparse
import sys

def create_sumo_network(x_min=175, x_max=1250, y_min=230, y_max=830, output_file='urban-scenario.net.xml'):
    """Create a SUMO network file for the simulation area"""
    print(f"Creating SUMO network: {output_file}")
    
    # Create a simple grid network
    width = x_max - x_min
    height = y_max - y_min
    
    # Generate junctions in a grid
    num_cols = 4
    num_rows = 3
    junctions = []
    junction_id = 1
    
    for row in range(num_rows):
        for col in range(num_cols):
            x = x_min + (col + 1) * (width / (num_cols + 1))
            y = y_min + (row + 1) * (height / (num_rows + 1))
            junctions.append((junction_id, x, y))
            junction_id += 1
    
    # Create XML structure
    root = ET.Element('net', {
        'version': '1.0',
        'junctionCornerDetail': '5',
        'limitTurnSpeed': '5.50'
    })
    
    # Location element
    location = ET.SubElement(root, 'location', {
        'netOffset': '0.00,0.00',
        'convBoundary': f'0.00,0.00,{x_max:.2f},{y_max:.2f}',
        'origBoundary': f'{x_min:.2f},{y_min:.2f},{x_max:.2f},{y_max:.2f}',
        'projParameter': '!'
    })
    
    # Create junctions
    for j_id, x, y in junctions:
        junction = ET.SubElement(root, 'junction', {
            'id': f'j{j_id}',
            'type': 'priority',
            'x': f'{x:.2f}',
            'y': f'{y:.2f}',
            'incLanes': '',
            'intLanes': '',
            'shape': f'{x-5:.2f},{y-5:.2f} {x+5:.2f},{y-5:.2f} {x+5:.2f},{y+5:.2f} {x-5:.2f},{y+5:.2f}'
        })
    
    # Create edges and lanes (simplified - you may want to expand this)
    # This is a basic implementation - for a real scenario, you'd want more sophisticated road network
    
    # Write to file
    tree = ET.ElementTree(root)
    ET.indent(tree, space='  ')
    tree.write(output_file, encoding='UTF-8', xml_declaration=True)
    print(f"✓ Created {output_file}")

def create_sumo_routes(num_ues=10, output_file='urban-scenario.rou.xml'):
    """Create SUMO routes file for UEs"""
    print(f"Creating SUMO routes: {output_file}")
    
    root = ET.Element('routes')
    
    # Vehicle type
    vtype = ET.SubElement(root, 'vType', {
        'id': 'ue',
        'vClass': 'passenger',
        'maxSpeed': '60.00',
        'minGap': '2.50',
        'accel': '2.60',
        'decel': '4.50',
        'sigma': '0.5',
        'tau': '1.0'
    })
    
    # Create routes and vehicles
    for i in range(num_ues):
        # Simple route (you may want to create more complex routes)
        route_id = f'r{i+1}'
        route = ET.SubElement(root, 'route', {
            'id': route_id,
            'edges': 'e1 e2 e3 e4'
        })
        
        # Vehicle
        vehicle = ET.SubElement(root, 'vehicle', {
            'id': f'ue{i}',
            'type': 'ue',
            'route': route_id,
            'depart': f'{i * 1.0:.2f}',
            'departLane': 'best',
            'departPos': 'random',
            'departSpeed': 'random'
        })
    
    # Write to file
    tree = ET.ElementTree(root)
    ET.indent(tree, space='  ')
    tree.write(output_file, encoding='UTF-8', xml_declaration=True)
    print(f"✓ Created {output_file}")

def convert_sumo_trace(sumo_trace_file, output_file='ns3_mobility.tcl'):
    """Convert SUMO FCD trace to ns3 mobility format (ns2-style TCL)
    
    Uses the existing sumo_to_ns3_trace.py conversion logic.
    For direct conversion, use: python3 sumo_to_ns3_trace.py <fcd_file> <output_tcl>
    """
    import subprocess
    import os
    
    print(f"Converting SUMO trace: {sumo_trace_file} -> {output_file}")
    
    # Check if sumo_to_ns3_trace.py exists in the same directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    converter_script = os.path.join(script_dir, 'sumo_to_ns3_trace.py')
    
    if os.path.exists(converter_script):
        print(f"Using {converter_script} for conversion...")
        try:
            result = subprocess.run(
                ['python3', converter_script, sumo_trace_file, output_file],
                capture_output=True,
                text=True,
                check=True
            )
            print(result.stdout)
            print(f"✓ Conversion completed: {output_file}")
        except subprocess.CalledProcessError as e:
            print(f"Error during conversion: {e.stderr}")
            sys.exit(1)
    else:
        print(f"Warning: {converter_script} not found.")
        print("Please use sumo_to_ns3_trace.py directly:")
        print(f"  python3 sumo_to_ns3_trace.py {sumo_trace_file} {output_file}")
        sys.exit(1)

def main():
    parser = argparse.ArgumentParser(description='SUMO Helper for ns3 Integration')
    parser.add_argument('--create-network', action='store_true',
                       help='Create SUMO network file')
    parser.add_argument('--create-routes', action='store_true',
                       help='Create SUMO routes file')
    parser.add_argument('--num-ues', type=int, default=10,
                       help='Number of UEs (default: 10)')
    parser.add_argument('--x-min', type=float, default=175.0,
                       help='Minimum X coordinate (default: 175.0)')
    parser.add_argument('--x-max', type=float, default=1250.0,
                       help='Maximum X coordinate (default: 1250.0)')
    parser.add_argument('--y-min', type=float, default=230.0,
                       help='Minimum Y coordinate (default: 230.0)')
    parser.add_argument('--y-max', type=float, default=830.0,
                       help='Maximum Y coordinate (default: 830.0)')
    parser.add_argument('--convert-trace', type=str, metavar='FCD_FILE',
                       help='Convert SUMO FCD trace to NS3 TCL format')
    parser.add_argument('--output', type=str, metavar='OUTPUT_FILE',
                       help='Output file for conversion (default: ns3_mobility.tcl)')
    
    args = parser.parse_args()
    
    if args.create_network:
        create_sumo_network(args.x_min, args.x_max, args.y_min, args.y_max)
    
    if args.create_routes:
        create_sumo_routes(args.num_ues)
    
    if args.convert_trace:
        output_file = args.output if args.output else 'ns3_mobility.tcl'
        convert_sumo_trace(args.convert_trace, output_file)
    
    if not args.create_network and not args.create_routes and not args.convert_trace:
        parser.print_help()

if __name__ == '__main__':
    main()

