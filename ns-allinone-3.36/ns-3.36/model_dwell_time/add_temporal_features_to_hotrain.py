#!/usr/bin/env python3

import argparse
import pandas as pd


def main():
    parser = argparse.ArgumentParser(
        description="Add 1-second temporal delta features to ho_train.csv using per-second CSV"
    )
    parser.add_argument("--persec_csv", required=True, help="Per-second 18-column CSV")
    parser.add_argument("--ho_train_csv", required=True, help="Existing ho_train CSV")
    parser.add_argument("--output_csv", required=True, help="Output ho_train CSV with temporal features")
    args = parser.parse_args()

    # Load per-second dataset
    ps = pd.read_csv(args.persec_csv)
    ht = pd.read_csv(args.ho_train_csv)

    required_ps = {"time_s", "ue_imsi", "gap_db", "sinr_db_calc", "distance_ue"}
    required_ht = {"ue_imsi", "t_ho"}

    missing_ps = required_ps - set(ps.columns)
    missing_ht = required_ht - set(ht.columns)

    if missing_ps:
        raise ValueError(f"Missing required per-second columns: {sorted(missing_ps)}")
    if missing_ht:
        raise ValueError(f"Missing required ho_train columns: {sorted(missing_ht)}")

    # Normalize types
    ps["ue_imsi"] = pd.to_numeric(ps["ue_imsi"], errors="raise").astype(int)
    ps["time_s"] = pd.to_numeric(ps["time_s"], errors="raise").astype(float)

    ht["ue_imsi"] = pd.to_numeric(ht["ue_imsi"], errors="raise").astype(int)
    ht["t_ho"] = pd.to_numeric(ht["t_ho"], errors="raise").astype(float)

    # Sort so lag features are correct
    ps = ps.sort_values(["ue_imsi", "time_s"]).reset_index(drop=True)

    # Previous values by UE
    ps["prev_gap_db"] = ps.groupby("ue_imsi")["gap_db"].shift(1)
    ps["prev_sinr_db_calc"] = ps.groupby("ue_imsi")["sinr_db_calc"].shift(1)
    ps["prev_distance_ue"] = ps.groupby("ue_imsi")["distance_ue"].shift(1)

    # 1-second deltas at each per-second row
    ps["delta_gap_1s"] = ps["gap_db"] - ps["prev_gap_db"]
    ps["delta_sinr_1s"] = ps["sinr_db_calc"] - ps["prev_sinr_db_calc"]
    ps["delta_distance_1s"] = ps["distance_ue"] - ps["prev_distance_ue"]

    # ho_train pre-HO features correspond to the per-second row at t_ho - 1
    ht["pre_time_s"] = ht["t_ho"] - 1.0

    # Round merge keys a bit to avoid float merge issues
    ps["time_key"] = ps["time_s"].round(6)
    ht["pre_time_key"] = ht["pre_time_s"].round(6)

    merge_cols = [
        "ue_imsi",
        "time_key",
        "delta_gap_1s",
        "delta_sinr_1s",
        "delta_distance_1s",
    ]
    delta_df = ps[merge_cols].copy()

    merged = ht.merge(
        delta_df,
        left_on=["ue_imsi", "pre_time_key"],
        right_on=["ue_imsi", "time_key"],
        how="left",
    )

    # Cleanup merge helper columns
    merged = merged.drop(columns=["time_key", "pre_time_key", "pre_time_s"])

    # Report missing merges
    missing_gap = int(merged["delta_gap_1s"].isna().sum())
    missing_sinr = int(merged["delta_sinr_1s"].isna().sum())
    missing_dist = int(merged["delta_distance_1s"].isna().sum())

    merged.to_csv(args.output_csv, index=False)

    print(f"Wrote: {args.output_csv}")
    print(f"Rows: {len(merged)}")
    print(f"missing_delta_gap_1s: {missing_gap}")
    print(f"missing_delta_sinr_1s: {missing_sinr}")
    print(f"missing_delta_distance_1s: {missing_dist}")


if __name__ == "__main__":
    main()
