#!/usr/bin/env python3
import os, sys, random

SUMO_HOME = os.environ.get("SUMO_HOME", "/usr/share/sumo")
TOOLS = os.path.join(SUMO_HOME, "tools")
sys.path.append(TOOLS)

import traci
import sumolib

def pick_edge(edges):
    while True:
        e = random.choice(edges)
        eid = e.getID()
        if eid.startswith(":"):
            continue
        if e.allows("passenger"):
            return eid

def shortest_path_edges(net, fr, to):
    try:
        path, _ = net.getShortestPath(net.getEdge(fr), net.getEdge(to))
    except Exception:
        return None
    if not path or len(path) < 2:
        return None
    return [e.getID() for e in path]

def safe_change_target(net, edges, vid, max_tries=120):
    cur = traci.vehicle.getRoadID(vid)
    if not cur or cur.startswith(":"):
        return False
    for _ in range(max_tries):
        dest = pick_edge(edges)
        if dest == cur:
            continue
        if shortest_path_edges(net, cur, dest) is None:
            continue
        try:
            traci.vehicle.changeTarget(vid, dest)
            return True
        except traci.TraCIException:
            continue
    return False

def main():
    if len(sys.argv) < 6:
        print("Usage: keep50_traci_nodie.py <port> <N> <endTime> <seed> <rerouteMarginEdges>")
        sys.exit(1)

    port = int(sys.argv[1])
    N = int(sys.argv[2])
    end_time = float(sys.argv[3])
    seed = int(sys.argv[4])
    margin = int(sys.argv[5])

    random.seed(seed)
    traci.init(port)

    net = sumolib.net.readNet("manhattan.net.xml")
    edges = net.getEdges()

    # Main loop: reroute existing vehicles before they reach end of route
    while traci.simulation.getTime() < end_time:
        traci.simulationStep()
        current_ids = set(traci.vehicle.getIDList())

        for i in range(N):
            vid = f"veh{i}"
            if vid not in current_ids:
                continue

            try:
                route = traci.vehicle.getRoute(vid)
                idx = traci.vehicle.getRouteIndex(vid)

                if len(route) < (margin + 2) or idx >= (len(route) - margin):
                    safe_change_target(net, edges, vid)

            except traci.TraCIException:
                # If it vanished, do nothing
                pass

    traci.close()

if __name__ == "__main__":
    main()
