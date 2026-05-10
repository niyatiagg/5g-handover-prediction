#!/usr/bin/env python3
import argparse, os, re, xml.etree.ElementTree as ET

def emit(title, items):
    print(f"=== {title} ===")
    for k, v in items.items():
        print(f"{k}: {v}")
    print()

def parse_sumocfg(path):
    out = {}
    if not path or not os.path.exists(path):
        out['sumocfg'] = 'not provided'
        return out
    root = ET.parse(path).getroot()
    time = root.find('time')
    if time is not None:
        for tag in ['begin', 'end', 'step-length']:
            node = time.find(tag)
            out[tag.replace('-', '_')] = node.get('value') if node is not None else 'not found'
    inp = root.find('input')
    if inp is not None:
        for tag in ['net-file', 'route-files', 'additional-files']:
            node = inp.find(tag)
            if node is not None:
                out[tag.replace('-', '_')] = node.get('value')
    return out

def parse_netxml(path):
    out = {}
    if not path or not os.path.exists(path):
        out['netxml'] = 'not provided'
        return out
    root = ET.parse(path).getroot()
    loc = root.find('location')
    if loc is not None:
        out['convBoundary'] = loc.get('convBoundary', 'not found')
        out['origBoundary'] = loc.get('origBoundary', 'not found')
    max_lanes = 0
    lane_speeds = []
    edge_count = 0
    for edge in root.findall('edge'):
        if edge.get('function'):
            continue
        edge_count += 1
        lanes = edge.findall('lane')
        max_lanes = max(max_lanes, len(lanes))
        for lane in lanes:
            sp = lane.get('speed')
            if sp:
                try:
                    lane_speeds.append(float(sp))
                except Exception:
                    pass
    out['road_edges'] = edge_count
    out['max_lanes_per_edge'] = max_lanes if max_lanes else 'not found'
    out['lane_speed_min_mps'] = min(lane_speeds) if lane_speeds else 'not found'
    out['lane_speed_max_mps'] = max(lane_speeds) if lane_speeds else 'not found'
    return out

def grep_ns3(path):
    out = {}
    if not path or not os.path.exists(path):
        out['ns3cc'] = 'not provided'
        return out
    txt = open(path, 'r', encoding='utf-8', errors='ignore').read()
    pats = {
        'ue_tx_power': r'UeTxPower[^0-9\-]*([0-9]+(?:\.[0-9]+)?)',
        'enb_tx_power': r'EnbTxPower[^0-9\-]*([0-9]+(?:\.[0-9]+)?)',
        'hysteresis': r'hysteresis[^0-9\-]*([0-9]+(?:\.[0-9]+)?)',
        'ttt_ms': r'TimeToTrigger[^0-9]*([0-9]+)',
        'scheduler_hint': r'SchedulerType.*?"([^"]+)"',
        'ho_algorithm_hint': r'HandoverAlgorithm.*?"([^"]+)"',
        'bandwidth_hint': r'(?:DlBandwidth|UlBandwidth)[^0-9]*([0-9]+)',
        'propagation_model_hint': r'(?:PathlossModel|PropagationLossModel).*?"([^"]+)"',
        'fading_model_hint': r'Fading.*?"([^"]+)"',
        'epc_hint': r'(PointToPointEpcHelper|NoBackhaulEpcHelper|EpcHelper)',
    }
    for k, pat in pats.items():
        m = re.search(pat, txt, flags=re.I | re.S)
        out[k] = m.group(1) if m else 'not found'
    return out

if __name__ == '__main__':
    ap = argparse.ArgumentParser(description='Extract useful SUMO and ns-3 parameters for reporting')
    ap.add_argument('--sumocfg', help='Path to SUMO .sumocfg file')
    ap.add_argument('--netxml', help='Path to SUMO .net.xml file')
    ap.add_argument('--ns3cc', help='Path to ns-3 .cc source file')
    args = ap.parse_args()
    emit('SUMO / MOBILITY PARAMETERS', parse_sumocfg(args.sumocfg))
    emit('SUMO NETWORK PARAMETERS', parse_netxml(args.netxml))
    emit('NS-3 / LTE PARAMETERS', grep_ns3(args.ns3cc))

