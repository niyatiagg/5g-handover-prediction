#!/usr/bin/env python3
import os
import sys
import random

# SUMO tools path (uses your $SUMO_HOME if set)
SUMO_HOME = os.environ.get("SUMO_HOME", "/usr/share/sumo")
TOOLS = os.path.join(SUMO_HOME, "tools")
sys.path.append(TOOLS)

import traci  # noqa: E402
import sumolib  # noqa: E402

def pick_edge(edges):
    # pick a random drivable edge (not internal)
    while True:
        e = random.choice(edges)
        if not e.getID().startswith(":") and e.allows("passenger"):
            return e.getID()

def make_route(net, edges, max_tries=50):
    # Find a drivable route from random fromEdge to toEdge
    for _ in range(max_tries):
        fr = pick_edge(edges)
        to = pick_edge(edges)
        if fr == to:
            continue
        r = net.getShortestPath(net.getEdge(fr), net.getEdge(to))[0]
        if r and len(r) >= 2:
            return fr, to, [e.getID() for e in r]
    # fallback: just pick two edges; SUMO will reject if impossible
    fr = pick_edge(edges)
    to = pick_edge(edges)
    return fr, to, [fr, to]

def add_or_respawn_vehicle(vid, net, edges, route_idx):
    rid = f"r_{vid}_{route_idx}"
    fr, to, edge_list = make_route(net, edges)

    # Register route
    try:
        traci.route.add(rid, edge_list)
    except traci.TraCIException:
        # route id may already exist; ignore
        pass

    # Add vehicle "now"
    try:
        traci.vehicle.add(vid, rid, typeID="DEFAULT_VEHTYPE", depart="now")
    except traci.TraCIException as e:
        # If vehicle exists, try setRoute instead
        try:
            traci.vehicle.setRoute(vid, edge_list)
        except traci.TraCIException:
            raise e

def main():
    if len(sys.argv) < 6:
        print("Usage: keep50_traci.py <sumocfg> <N> <endTime> <seed> <port>")
        sys.exit(1)

    sumocfg = sys.argv[1]
    N = int(sys.argv[2])
    end_time = float(sys.argv[3])
    seed = int(sys.argv[4])
    port = int(sys.argv[5])

    random.seed(seed)

    # Connect to SUMO
    traci.init(port)

    # Load network for route generation
    net = sumolib.net.readNet("manhattan.net.xml")
    edges = net.getEdges()

    # Spawn N vehicles with fixed IDs: veh0..veh(N-1)
    route_idx = 0
    for i in range(N):
        add_or_respawn_vehicle(f"veh{i}", net, edges, route_idx)
        route_idx += 1

    # Main loop
    while traci.simulation.getTime() < end_time:
        traci.simulationStep()

        # Vehicles that arrived in this step are removed by SUMO
        arrived = traci.simulation.getArrivedIDList()
        for vid in arrived:
            # respawn immediately with a new route (same ID)
            add_or_respawn_vehicle(vid, net, edges, route_idx)
            route_idx += 1

    traci.close()

if __name__ == "__main__":
    main()
