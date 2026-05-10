#!/usr/bin/env python3

import argparse
import numpy as np
import pandas as pd


def slope_last3(values):
    if len(values) != 3 or np.any(pd.isna(values)):
        return np.nan
    x = np.array([0.0, 1.0, 2.0])
    y = np.array(values, dtype=float)
    x_mean = x.mean()
    y_mean = y.mean()
    denom = ((x - x_mean) ** 2).sum()
    if denom == 0:
        return np.nan
    return (((x - x_mean) * (y - y_mean)).sum()) / denom


def main():
    parser = argparse.ArgumentParser(
        description="Add 3-second temporal summary features to ho_train.csv"
    )
    parser.add_argument("--persec_csv", required=True, help="Per-second 18-column CSV")
    parser.add_argument("--ho_train_csv", required=True, help="Existing ho_train CSV")
    parser.add_argument("--output_csv", required=True, help="Output ho_train CSV")
    args = parser.parse_args()

    ps = pd.read_csv(args.persec_csv)
    ht = pd.read_csv(args.ho_train_csv)

    required_ps = {"time_s", "ue_imsi", "gap_db", "sinr_db_calc"}
    required_ht = {"ue_imsi", "t_ho"}

    missing_ps = required_ps - set(ps.columns)
    missing_ht = required_ht - set(ht.columns)

    if missing_ps:
        raise ValueError(f"Missing required per-second columns: {sorted(missing_ps)}")
    if missing_ht:
        raise ValueError(f"Missing required ho_train columns: {sorted(missing_ht)}")

    ps["ue_imsi"] = pd.to_numeric(ps["ue_imsi"], errors="raise").astype(int)
    ps["time_s"] = pd.to_numeric(ps["time_s"], errors="raise").astype(float)

    ht["ue_imsi"] = pd.to_numeric(ht["ue_imsi"], errors="raise").astype(int)
    ht["t_ho"] = pd.to_numeric(ht["t_ho"], errors="raise").astype(float)

    ps = ps.sort_values(["ue_imsi", "time_s"]).reset_index(drop=True)

    ps["avg_gap_last3"] = (
        ps.groupby("ue_imsi")["gap_db"]
        .rolling(window=3, min_periods=3)
        .mean()
        .reset_index(level=0, drop=True)
    )

    ps["avg_sinr_last3"] = (
        ps.groupby("ue_imsi")["sinr_db_calc"]
        .rolling(window=3, min_periods=3)
        .mean()
        .reset_index(level=0, drop=True)
    )

    ps["gap_slope_last3"] = (
        ps.groupby("ue_imsi")["gap_db"]
        .rolling(window=3, min_periods=3)
        .apply(lambda x: slope_last3(list(x)), raw=False)
        .reset_index(level=0, drop=True)
    )

    ps["sinr_slope_last3"] = (
        ps.groupby("ue_imsi")["sinr_db_calc"]
        .rolling(window=3, min_periods=3)
        .apply(lambda x: slope_last3(list(x)), raw=False)
        .reset_index(level=0, drop=True)
    )

    ht["pre_time_s"] = ht["t_ho"] - 1.0

    ps["time_key"] = ps["time_s"].round(6)
    ht["pre_time_key"] = ht["pre_time_s"].round(6)

    merge_cols = [
        "ue_imsi",
        "time_key",
        "avg_gap_last3",
        "avg_sinr_last3",
        "gap_slope_last3",
        "sinr_slope_last3",
    ]

    merged = ht.merge(
        ps[merge_cols],
        left_on=["ue_imsi", "pre_time_key"],
        right_on=["ue_imsi", "time_key"],
        how="left",
    )

    merged = merged.drop(columns=["time_key", "pre_time_key", "pre_time_s"])

    print(f"Rows: {len(merged)}")
    print(f"missing_avg_gap_last3: {int(merged['avg_gap_last3'].isna().sum())}")
    print(f"missing_avg_sinr_last3: {int(merged['avg_sinr_last3'].isna().sum())}")
    print(f"missing_gap_slope_last3: {int(merged['gap_slope_last3'].isna().sum())}")
    print(f"missing_sinr_slope_last3: {int(merged['sinr_slope_last3'].isna().sum())}")

    merged.to_csv(args.output_csv, index=False)
    print(f"Wrote: {args.output_csv}")


if __name__ == "__main__":
    main()
