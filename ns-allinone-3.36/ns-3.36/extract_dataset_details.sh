#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   bash extract_dataset_details.sh \
#     /path/to/per_second.csv \
#     /path/to/ho_events.csv \
#     /path/to/ho_train.csv

if [ "$#" -lt 3 ]; then
  echo "Usage: bash extract_dataset_details.sh <per_second_csv> <ho_events_csv> <ho_train_csv>"
  exit 1
fi

PERSEC="$1"
EVENTS="$2"
TRAIN="$3"

python3 - "$PERSEC" "$EVENTS" "$TRAIN" <<'PY'
import csv
import math
import sys
from collections import Counter

persec, events, train = sys.argv[1:4]

def maybe_float(x):
    if x is None:
        return None
    s = str(x).strip()
    if s == '' or s.lower() == 'nan':
        return None
    try:
        return float(s)
    except Exception:
        return None

def maybe_int(x):
    v = maybe_float(x)
    if v is None:
        return None
    return int(v)

# ----- per-second CSV -----
with open(persec, newline='') as f:
    reader = csv.DictReader(f)
    rows = 0
    times = []
    ue_set = set()
    current_ids = set()
    rssi_ids = set()
    a3_count = 0
    kpi_count = 0
    hys_counter = Counter()
    has_speed = False
    for row in reader:
        rows += 1
        t = maybe_float(row.get('time_s'))
        if t is not None:
            times.append(t)
        ue = maybe_int(row.get('ue_imsi'))
        if ue is not None:
            ue_set.add(ue)
        cid = maybe_int(row.get('current_id'))
        if cid is not None and cid != 0:
            current_ids.add(cid)
        rid = maybe_int(row.get('rssi_id'))
        if rid is not None and rid != 0:
            rssi_ids.add(rid)
        a3 = maybe_int(row.get('trigger_a3'))
        if a3 == 1:
            a3_count += 1
        kpi = maybe_int(row.get('trigger_kpi'))
        if kpi == 1:
            kpi_count += 1
        hys = maybe_float(row.get('hysteresis'))
        if hys is not None:
            hys_counter[round(hys, 4)] += 1
        if 'speed_ue' in row:
            has_speed = True

print('=== PER-SECOND DATASET SUMMARY ===')
print(f'file: {persec}')
print(f'rows: {rows}')
print(f'columns: {sum(1 for _ in csv.DictReader(open(persec, newline="")).fieldnames)}')
if times:
    print(f'time_start_s: {min(times)}')
    print(f'time_end_s: {max(times)}')
    print(f'simulation_duration_s: {max(times) - min(times)}')
print(f'unique_ues: {len(ue_set)}')
print(f'unique_serving_cells_nonzero: {len(current_ids)}')
print(f'unique_measured_cells_nonzero: {len(rssi_ids)}')
print(f'trigger_a3_count: {a3_count}')
print(f'trigger_kpi_count: {kpi_count}')
print(f'mobility_columns_present: {has_speed}')
if hys_counter:
    common_hys = ', '.join(f'{k}:{v}' for k, v in hys_counter.most_common(5))
    print(f'most_common_hysteresis_values: {common_hys}')
print()

# ----- ho_events CSV -----
with open(events, newline='') as f:
    reader = csv.DictReader(f)
    rows = 0
    reversal_15 = 0
    short_dwell_15 = 0
    stable_true = 0
    dwells = []
    ue_set_ev = set()
    src_cells = set()
    dst_cells = set()
    for row in reader:
        rows += 1
        ue = maybe_int(row.get('ue_imsi'))
        if ue is not None:
            ue_set_ev.add(ue)
        src = maybe_int(row.get('src_cell'))
        dst = maybe_int(row.get('dst_cell'))
        if src is not None:
            src_cells.add(src)
        if dst is not None:
            dst_cells.add(dst)
        dwell = maybe_float(row.get('dwell_s'))
        if dwell is not None:
            dwells.append(dwell)
            if dwell <= 15:
                short_dwell_15 += 1
        pp = str(row.get('pingpong', '')).strip().lower()
        if pp == 'true':
            reversal_15 += 1
        st = str(row.get('stable', '')).strip().lower()
        if st == 'true':
            stable_true += 1

print('=== HO_EVENTS SUMMARY ===')
print(f'file: {events}')
print(f'rows: {rows}')
print(f'unique_ues: {len(ue_set_ev)}')
print(f'unique_src_cells: {len(src_cells)}')
print(f'unique_dst_cells: {len(dst_cells)}')
if dwells:
    print(f'dwell_min_s: {min(dwells)}')
    print(f'dwell_median_s: {sorted(dwells)[len(dwells)//2]}')
    print(f'dwell_max_s: {max(dwells)}')
print(f'reversal_events_from_pingpong_column: {reversal_15}')
print(f'short_dwell_15_from_dwell_s: {short_dwell_15}')
print(f'stable_true_count: {stable_true}')
print()

# ----- ho_train CSV -----
with open(train, newline='') as f:
    reader = csv.DictReader(f)
    rows = 0
    unstable = 0
    reversal = 0
    short_dwell = 0
    stable_final = 0
    has_temporal = False
    has_temporal_summary = False
    train_ues = set()
    for row in reader:
        rows += 1
        ue = maybe_int(row.get('ue_imsi'))
        if ue is not None:
            train_ues.add(ue)
        if maybe_int(row.get('unstable_15')) == 1:
            unstable += 1
        if maybe_int(row.get('reversal_15')) == 1:
            reversal += 1
        if maybe_int(row.get('short_dwell_15')) == 1:
            short_dwell += 1
        if maybe_int(row.get('stable_final')) == 1:
            stable_final += 1
        if 'delta_gap_1s' in row or 'delta_sinr_1s' in row or 'delta_distance_1s' in row:
            has_temporal = True
        if 'avg_gap_last3' in row or 'avg_sinr_last3' in row or 'gap_slope_last3' in row:
            has_temporal_summary = True

print('=== HO_TRAIN SUMMARY ===')
print(f'file: {train}')
print(f'rows: {rows}')
print(f'unique_ues: {len(train_ues)}')
print(f'reversal_15_count: {reversal}')
print(f'short_dwell_15_count: {short_dwell}')
print(f'unstable_15_count: {unstable}')
print(f'stable_final_count: {stable_final}')
if rows > 0:
    print(f'unstable_15_rate: {unstable / rows:.6f}')
print(f'temporal_delta_features_present: {has_temporal}')
print(f'temporal_summary_features_present: {has_temporal_summary}')
print()

print('=== PAPER FILL-IN CHECKLIST ===')
print('- Report the per-second row count, ho_events row count, and ho_train row count.')
print('- Report the number of UEs and the number of unique non-zero serving cells.')
print('- Report simulation start, end, and total duration from the per-second CSV.')
print('- Report trigger_a3 and trigger_kpi counts if you discuss trigger sparsity.')
print('- Report reversal_15, short_dwell_15, unstable_15, and positive-rate counts from ho_train.')
print('- Add the exact TTT and hysteresis values separately from your simulator configuration if they are not fully inferable from the CSV.')
PY

