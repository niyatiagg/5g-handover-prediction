#!/usr/bin/env python3
"""
Run SUMO with TraCI and generate trace file
Usage: python sumo_traci_trace.py <config_file> <output_file>
"""

import os
import sys

try:
    import traci
    import sumolib
except ImportError:
    print("Error: traci and sumolib not found. Install with:")
    print("  pip install traci sumolib")
    sys.exit(1)

def run_sumo_with_traci(config_file, output_file, duration=1000):
    """Run SUMO with TraCI and generate trace"""
    print(f"Starting SUMO with config: {config_file}")
    
    # Check if SUMO binary exists
    sumoBinary = sumolib.checkBinary('sumo')
    print(f"Using SUMO binary: {sumoBinary}")
    
    sumoCmd = [
        sumoBinary,
        '-c', config_file,
        '--step-length', '0.1',
        '--start',
        '--quit-on-end',
        '--no-warnings',
        '--no-step-log',
        '--duration-log.disable'
    ]
    
    try:
        traci.start(sumoCmd)
        print("SUMO TraCI connection established")
    except Exception as e:
        print(f"Error starting SUMO: {e}")
        sys.exit(1)
    
    # Open output file
    print(f"Writing trace to: {output_file}")
    with open(output_file, 'w') as f:
        # Write CSV header
        f.write("Time,VehicleID,X,Y,Speed,Angle,Lane,Edge\n")
        
        step = 0
        max_steps = int(duration / 0.1)  # Convert seconds to steps
        
        print(f"Running simulation for {duration} seconds ({max_steps} steps)...")
        
        while step < max_steps:
            try:
                traci.simulationStep()
                time = traci.simulation.getTime()
                
                # Get all vehicle IDs
                vehicle_ids = traci.vehicle.getIDList()
                
                for veh_id in vehicle_ids:
                    try:
                        # Get vehicle position
                        pos = traci.vehicle.getPosition(veh_id)
                        x, y = pos[0], pos[1]
                        
                        # Get vehicle speed (m/s)
                        speed = traci.vehicle.getSpeed(veh_id)
                        
                        # Get vehicle angle (degrees)
                        angle = traci.vehicle.getAngle(veh_id)
                        
                        # Get lane information
                        try:
                            lane_id = traci.vehicle.getLaneID(veh_id)
                            edge_id = lane_id.split('_')[0]  # Extract edge ID from lane ID
                        except:
                            lane_id = "unknown"
                            edge_id = "unknown"
                        
                        # Write to file
                        f.write(f"{time:.2f},{veh_id},{x:.2f},{y:.2f},{speed:.2f},{angle:.2f},{lane_id},{edge_id}\n")
                        
                    except traci.exceptions.TraCIException as e:
                        # Vehicle may have left the simulation
                        continue
                
                # Progress update every 100 steps
                if step % 100 == 0:
                    print(f"  Step {step}/{max_steps} ({time:.1f}s), Vehicles: {len(vehicle_ids)}")
                
                step += 1
                
                # Check if simulation has ended
                if traci.simulation.getMinExpectedNumber() == 0:
                    print("Simulation ended (no more vehicles)")
                    break
                    
            except traci.exceptions.FatalTraCIError as e:
                print(f"Fatal TraCI error: {e}")
                break
    
    try:
        traci.close()
        print("TraCI connection closed")
    except:
        pass
    
    print(f"✓ Trace generation complete: {output_file}")
    print(f"  Total steps: {step}")

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python sumo_traci_trace.py <config_file> <output_file> [duration_seconds]")
        print("\nExample:")
        print("  python sumo_traci_trace.py urban-scenario.sumocfg sumo-trace.csv 100")
        sys.exit(1)
    
    config_file = sys.argv[1]
    output_file = sys.argv[2]
    duration = float(sys.argv[3]) if len(sys.argv) > 3 else 1000.0
    
    # Check if config file exists
    if not os.path.exists(config_file):
        print(f"Error: Config file not found: {config_file}")
        sys.exit(1)
    
    run_sumo_with_traci(config_file, output_file, duration)

