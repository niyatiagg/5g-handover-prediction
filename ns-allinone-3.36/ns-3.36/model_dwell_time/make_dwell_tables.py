import argparse
import numpy as np
import pandas as pd

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True, help="Input per-second dataset CSV (15-col)")
    ap.add_argument("--out_events", default="ho_events.csv")
    ap.add_argument("--out_train", default="ho_train.csv")
    ap.add_argument("--sim_end", type=float, default=None, help="Optional. If not set, uses max(time_s)+1")
    ap.add_argument("--pingpong_w", type=float, default=10.0, help="Ping-pong window seconds")
    ap.add_argument("--dwell_thresh", type=float, default=10.0, help="Stable dwell threshold seconds")
    args = ap.parse_args()

    df = pd.read_csv(args.csv)

    # Coerce numerics
    for c in df.columns:
        if c not in ["ue_imsi"]:
            df[c] = pd.to_numeric(df[c], errors="coerce")

    # Keep only valid serving rows
    df = df[(df["current_id"] != 0) & (df["distance_ue"].notna())].copy()

    # Sort correctly
    df.sort_values(["ue_imsi", "time_s"], inplace=True)

    # Estimate sim end if needed
    sim_end = args.sim_end
    if sim_end is None:
        sim_end = float(df["time_s"].max() + 1.0)

    g = df.groupby("ue_imsi", sort=False)

    # Previous state for HO execution detection
    df["prev_cid"] = g["current_id"].shift(1)
    df["prev_time"] = g["time_s"].shift(1)

    # HO execution = serving cell changed
    df["ho_exec"] = (df["prev_cid"].notna()) & (df["current_id"] != df["prev_cid"])

    # Add pre-HO lagged features FIRST
    feat_cols = [
        "rssi", "rsrp", "sinr_db_calc", "distance_ue",
        "gap_db", "trigger_a3", "trigger_kpi", "hysteresis", "rssi_id"
    ]
    for c in feat_cols:
        df["pre_" + c] = g[c].shift(1)

    # Build HO event table from rows where HO happens
    ev = df[df["ho_exec"]].copy()
    ev.rename(columns={"time_s": "t_ho"}, inplace=True)
    ev["src_cell"] = ev["prev_cid"].astype(int)
    ev["dst_cell"] = ev["current_id"].astype(int)

    # Next HO info per UE
    ev["next_t_ho"] = ev.groupby("ue_imsi")["t_ho"].shift(-1)
    ev["next_dst"] = ev.groupby("ue_imsi")["dst_cell"].shift(-1)

    # Dwell time after HO
    ev["dwell_s"] = np.where(
        ev["next_t_ho"].notna(),
        ev["next_t_ho"] - ev["t_ho"],
        sim_end - ev["t_ho"]
    )

    # Ping-pong: A->B then B->A within W seconds
    W = args.pingpong_w
    ev["pingpong"] = (
        ev["next_dst"].notna() &
        (ev["next_dst"] == ev["src_cell"]) &
        (ev["dwell_s"] <= W)
    )

    # Stable label
    T = args.dwell_thresh
    ev["stable"] = (ev["dwell_s"] >= T) & (~ev["pingpong"])

    # Save event table
    keep_cols = ["ue_imsi", "t_ho", "src_cell", "dst_cell", "dwell_s", "pingpong", "stable"]
    ev[keep_cols].to_csv(args.out_events, index=False)

    # Training table = event table + pre-HO features
    pre_cols = ["pre_" + c for c in feat_cols]
    train_cols = ["ue_imsi", "t_ho", "src_cell", "dst_cell", "dwell_s", "pingpong", "stable"] + pre_cols
    train = ev[train_cols].copy()

    # Drop rows where pre-HO features are unavailable
    train.dropna(subset=pre_cols, inplace=True)

    train.to_csv(args.out_train, index=False)

    total_ho = len(ev)
    pp = int(ev["pingpong"].sum())
    stable = int(ev["stable"].sum())

    print("HO_events=", total_ho)
    print("PingPong_events=", pp, "PingPong_rate=", (pp / total_ho if total_ho > 0 else 0.0))
    print("Stable_events=", stable)
    print("Saved:", args.out_events, "and", args.out_train)

if __name__ == "__main__":
    main()
