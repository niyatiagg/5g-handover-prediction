#!/usr/bin/env python3
"""
SUMO TraCI runner (robust):

- ue0..ue(N-1) created ONCE (no duplicate vehicle.add -> no "already exists")
- Random initial depart times
- Each vehicle completes a trip, then waits random time, then starts another trip
- New trip is created via traci.vehicle.changeTarget() (NO setRouteID replacement)
  -> fixes: "Route replacement failed ... current edge not found in new route"
- Keeps at least MIN_MOVING vehicles moving at all times by waking some early
"""

import os
import random
from typing import List, Optional

if "SUMO_HOME" not in os.environ:
    raise RuntimeError("Please set SUMO_HOME (export SUMO_HOME=/path/to/sumo).")

import traci
import sumolib

# -----------------------------
# PARAMETERS
# -----------------------------
SIM_END = 500000
STEP_LEN = 1.0
FCD_PERIOD = 1
SEED = 42

NUM_VEHICLES = 10
MIN_MOVING = 2

INIT_DEPART_MIN = 0
INIT_DEPART_MAX = 500000

WAIT_MIN = 5
WAIT_MAX = 120

# Used only for initial route generation
ROUTE_TRIES = 800

NET_FILE = "net_offset.net.xml"
ADD_FILE = "vtypes.add.xml"
FCD_FILE = "fcd.xml"

TYPE_ID = "ue"
VCLASS = "passenger"

random.seed(SEED)

# -----------------------------
# Network helpers
# -----------------------------
net = sumolib.net.readNet(NET_FILE)

def is_internal_edge_id(eid: str) -> bool:
    return eid.startswith(":")

def edge_allows(edge, vclass: str) -> bool:
    try:
        return edge.allows(vclass)
    except Exception:
        return True

# Candidate destination edges: non-internal, allow passenger
dest_edges = [
    e.getID() for e in net.getEdges()
    if (not e.getID().startswith(":")) and edge_allows(e, VCLASS)
]
if len(dest_edges) < 2:
    raise RuntimeError("Not enough usable destination edges (need >= 2).")

# For initial route generation we also keep edge objects:
edge_objs = [
    e for e in net.getEdges()
    if (not e.getID().startswith(":")) and edge_allows(e, VCLASS)
]

def pick_two_distinct_edges():
    a = random.choice(edge_objs)
    b = random.choice(edge_objs)
    while b.getID() == a.getID():
        b = random.choice(edge_objs)
    return a, b

def shortest_path_edge_ids(from_edge, to_edge) -> Optional[List[str]]:
    try:
        path, _ = net.getShortestPath(from_edge, to_edge, vClass=VCLASS)
    except TypeError:
        path, _ = net.getShortestPath(from_edge, to_edge)
    if not path:
        return None
    return [e.getID() for e in path]

def make_initial_route() -> Optional[List[str]]:
    for _ in range(ROUTE_TRIES):
        frm, to = pick_two_distinct_edges()
        r = shortest_path_edge_ids(frm, to)
        if r and len(r) >= 1:
            return r
    return None

def random_dest_edge(exclude: Optional[str] = None) -> str:
    """Pick a random destination edge, optionally excluding one edge id."""
    if exclude is None or len(dest_edges) == 1:
        return random.choice(dest_edges)
    cand = random.choice(dest_edges)
    tries = 0
    while cand == exclude and tries < 10:
        cand = random.choice(dest_edges)
        tries += 1
    return cand

# -----------------------------
# Start SUMO
# -----------------------------
sumo_cmd = [
    "sumo",
    "-n", NET_FILE,
    "-a", ADD_FILE,
    "--begin", "0",
    "--end", str(SIM_END),
    "--step-length", str(STEP_LEN),
    "--fcd-output", FCD_FILE,
    "--device.fcd.period", str(FCD_PERIOD),
    "--fcd-output.attributes", "id,x,y,speed,angle",
]
traci.start(sumo_cmd)

veh_ids = [f"ue{i}" for i in range(NUM_VEHICLES)]

# waiting_until[vid] = None if moving; else time when it should resume
waiting_until = {vid: None for vid in veh_ids}

try:
    # 1) Add vehicles ONCE with random initial depart times
    initial_departs = {vid: float(random.randint(INIT_DEPART_MIN, INIT_DEPART_MAX)) for vid in veh_ids}
    # ensure at least MIN_MOVING depart early
    for vid in veh_ids[:MIN_MOVING]:
        initial_departs[vid] = min(initial_departs[vid], 5.0)

    for vid in veh_ids:
        r = make_initial_route()
        if not r:
            raise RuntimeError("Could not build an initial route; network may be disconnected.")
        rid = f"r_init_{vid}"
        traci.route.add(rid, r)
        traci.vehicle.add(vid, rid, typeID=TYPE_ID, depart=initial_departs[vid])

    # 2) Main loop
    while traci.simulation.getTime() < SIM_END:
        traci.simulationStep()
        now = float(traci.simulation.getTime())

        active = traci.vehicle.getIDList()

        # A) If a moving vehicle is at the end of its route, stop it (wait)
        # We only stop when the route is essentially finished; we do NOT remove/re-add vehicles.
        for vid in active:
            if waiting_until.get(vid) is not None:
                continue  # already waiting

            try:
                route = traci.vehicle.getRoute(vid)
                idx = traci.vehicle.getRouteIndex(vid)
                # routeIndex points to current edge index in the route
                if route and idx >= len(route) - 1:
                    # finished route -> start waiting
                    wait = float(random.randint(WAIT_MIN, WAIT_MAX))
                    waiting_until[vid] = now + wait

                    # force stop
                    traci.vehicle.setSpeedMode(vid, 0)
                    traci.vehicle.setSpeed(vid, 0.0)
            except traci.TraCIException:
                pass

        # B) Resume vehicles whose waiting time elapsed:
        # Instead of setRouteID (which can fail), use changeTarget so SUMO reroutes from current position.
        for vid in active:
            t = waiting_until.get(vid)
            if t is None:
                continue
            if now >= t:
                try:
                    # release speed control
                    traci.vehicle.setSpeedMode(vid, 31)
                    traci.vehicle.setSpeed(vid, -1)

                    # choose a new destination and reroute from current position
                    cur_edge = traci.vehicle.getRoadID(vid)
                    # if we're on an internal edge/junction, still pick a normal destination
                    dest = random_dest_edge(exclude=None if is_internal_edge_id(cur_edge) else cur_edge)
                    traci.vehicle.changeTarget(vid, dest)

                    waiting_until[vid] = None
                except traci.TraCIException:
                    # retry next step
                    waiting_until[vid] = now + STEP_LEN

        # C) Enforce MIN_MOVING vehicles moving at all times by waking some early
        active = traci.vehicle.getIDList()
        moving = [vid for vid in active if waiting_until.get(vid) is None]
        if len(moving) < MIN_MOVING:
            need = MIN_MOVING - len(moving)
            sleepers = [vid for vid in active if waiting_until.get(vid) is not None]
            for vid in sleepers[:need]:
                waiting_until[vid] = now  # wake immediately next loop; try now too
                try:
                    traci.vehicle.setSpeedMode(vid, 31)
                    traci.vehicle.setSpeed(vid, -1)
                    cur_edge = traci.vehicle.getRoadID(vid)
                    dest = random_dest_edge(exclude=None if is_internal_edge_id(cur_edge) else cur_edge)
                    traci.vehicle.changeTarget(vid, dest)
                    waiting_until[vid] = None
                except traci.TraCIException:
                    waiting_until[vid] = now + STEP_LEN

finally:
    traci.close()

