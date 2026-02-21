import os
import random

# SUMO imports
if "SUMO_HOME" not in os.environ:
    raise RuntimeError("Please set SUMO_HOME (e.g., export SUMO_HOME=/path/to/sumo).")

import traci
import sumolib

SIM_END = 500000          # seconds
STEP_LEN = 1.0            # seconds (keep it 1.0 for long runs)
FCD_PERIOD = 1            # write FCD every 1s (increase to 5/10 to reduce file size)
SEED = 42

random.seed(SEED)

NET_FILE = "net_offset.net.xml"
ROUTE_FILE = "routes.rou.xml"
ADD_FILE = "vtypes.add.xml"
FCD_FILE = "fcd.xml"

# Read edges for choosing random destinations (skip internal edges that start with ':')
net = sumolib.net.readNet(NET_FILE)
edges = [e.getID() for e in net.getEdges() if not e.getID().startswith(":")]
if not edges:
    raise RuntimeError("No usable edges found in the network.")

sumo_cmd = [
    "sumo",
    "-n", NET_FILE,
    "-r", ROUTE_FILE,
    "-a", ADD_FILE,
    "--begin", "0",
    "--end", str(SIM_END),
    "--step-length", str(STEP_LEN),
    "--fcd-output", FCD_FILE,
    "--device.fcd.period", str(FCD_PERIOD),
    "--fcd-output.attributes", "id,x,y,speed,angle",
]

traci.start(sumo_cmd)

try:
    while traci.simulation.getTime() < SIM_END:
        traci.simulationStep()

        # Vehicles currently in the network
        vids = traci.vehicle.getIDList()
        for vid in vids:
            try:
                route = traci.vehicle.getRoute(vid)
                idx = traci.vehicle.getRouteIndex(vid)  # current edge index in route

                # If vehicle is at (or near) the end of its route, pick a new random destination
                if route and idx >= len(route) - 2:
                    current_edge = traci.vehicle.getRoadID(vid)
                    dest = random.choice(edges)
                    # avoid trivial "reroute to same edge" if possible
                    if dest == current_edge and len(edges) > 1:
                        dest = random.choice([e for e in edges if e != current_edge])

                    # changeTarget rebuilds the route toward the new destination
                    traci.vehicle.changeTarget(vid, dest)

            except traci.TraCIException:
                # vehicle may have teleported/been removed this step; ignore and continue
                pass

finally:
    traci.close()
