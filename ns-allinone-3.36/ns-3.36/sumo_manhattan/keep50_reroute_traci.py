#!/usr/bin/env python3
import os, sys, random

SUMO_HOME = os.environ.get("SUMO_HOME", "/usr/share/sumo")
TOOLS = os.path.join(SUMO_HOME, "tools")
sys.path.append(TOOLS)

import traci  # noqa
import sumolib  # noqa

def pick_edge(edges):
    while True:
        e = random.choice(edges)
        eid = e.getID()
        if eid.startswith(":"):
            continue
        if e.allows("passenger"):
            return eid

def shortest_route(net, edges, max_tries=50):
    for _ in range(max_tries):
        fr = pick_edge(edges)
        to = pick_edge(edges)
        if fr == to:
            continue
        path, _ = net.getShortestPath(net.getEdge(fr), net.getEdge(to))
        if path and len(path) >= 2:
            return [e.getID() for e in path]
    # fallback (rare)
    fr = pick_edge(edges)
    to = pick_edge(edges)
    return [fr, to]

def ensure_vehicle(net, edges, vid, route_idx):
    rid = f"r_{vid}_{route_idx}"
    route_edges = shortest_route(net, edges)
    try:
        traci.route.add(rid, route_edges)
    except traci.TraCIException:
        pass
    try:
        traci.vehicle.add(vid, rid, typeID="DEFAULT_VEHTYPE", depart="now")
    except traci.TraCIException:
        # already exists, just set a route
        traci.vehicle.setRoute(vid, route_edges)
    return route_edges

def reroute_vehicle(net, edges, vid, route_idx):
    # Build a route that starts from the vehicle's current edge
    cur = traci.vehicle.getRoadID(vid)

    # If vehicle is in an internal edge like ":...", keep stepping until it leaves,
    # or just skip reroute this round.
    if cur.startswith(":") or cur == "":
        return

    # Pick a destination edge that is not internal and is reachable
    for _ in range(50):
        to = pick_edge(edges)
        if to == cur:
            continue
        path, _ = net.getShortestPath(net.getEdge(cur), net.getEdge(to))
        if path and len(path) >= 2:
            traci.vehicle.setRoute(vid, [e.getID() for e in path])
            return

    # If no route found, do nothing this round
    return

def main():
    if len(sys.argv) < 7:
        print("Usage: keep50_reroute_traci.py <sumocfg> <N> <endTime> <seed> <port> <reroutePeriodSec>")
        sys.exit(1)

    sumocfg = sys.argv[1]
    N = int(sys.argv[2])
    end_time = float(sys.argv[3])
    seed = int(sys.argv[4])
    port = int(sys.argv[5])
    period = float(sys.argv[6])

    random.seed(seed)

    # connect
    traci.init(port)

    net = sumolib.net.readNet("manhattan.net.xml")
    edges = net.getEdges()

    route_idx = 0
    # create N vehicles once
    for i in range(N):
        ensure_vehicle(net, edges, f"veh{i}", route_idx)
        route_idx += 1

    next_reroute_t = period

    while traci.simulation.getTime() < end_time:
        traci.simulationStep()

        t = traci.simulation.getTime()

        # Safety: if any vehicle somehow arrived and got removed, re-add it (should be rare)
        arrived = traci.simulation.getArrivedIDList()
        for vid in arrived:
            ensure_vehicle(net, edges, vid, route_idx)
            route_idx += 1

        # periodic reroute of all vehicles (keeps them from finishing and disappearing)
        if t >= next_reroute_t:
            for i in range(N):
                vid = f"veh{i}"
                try:
                    reroute_vehicle(net, edges, vid, route_idx)
                    route_idx += 1
                except traci.TraCIException:
                    # If reroute fails, skip this vehicle this cycle
                    pass
            next_reroute_t += period

    traci.close()

if __name__ == "__main__":
    main()
