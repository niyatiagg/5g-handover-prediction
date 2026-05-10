#!/usr/bin/env python3

import argparse
import pandas as pd


def main():
    parser = argparse.ArgumentParser(description="Add final instability labels to ho_train.csv")
    parser.add_argument("--input", required=True, help="Input ho_train.csv")
    parser.add_argument("--output", required=True, help="Output labeled CSV")
    parser.add_argument("--window", type=float, default=15.0, help="Reversal/short-dwell window in seconds")
    args = parser.parse_args()

    df = pd.read_csv(args.input)

    required = {"ue_imsi", "t_ho", "src_cell", "dst_cell", "dwell_s"}
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"Missing required columns: {sorted(missing)}")

    # enforce numeric
    for col in ["ue_imsi", "t_ho", "src_cell", "dst_cell", "dwell_s"]:
        df[col] = pd.to_numeric(df[col], errors="raise")

    # deterministic order
    df = df.sort_values(["ue_imsi", "t_ho"]).reset_index(drop=True)

    # next HO info per UE
    df["next_t_ho"] = df.groupby("ue_imsi")["t_ho"].shift(-1)
    df["next_src_cell"] = df.groupby("ue_imsi")["src_cell"].shift(-1)
    df["next_dst_cell"] = df.groupby("ue_imsi")["dst_cell"].shift(-1)

    # time to next HO
    df["dt_next_ho"] = df["next_t_ho"] - df["t_ho"]

    # reversal_15:
    # current HO A->B, next HO must be B->A within window
    reversal_mask = (
        df["next_t_ho"].notna()
        & (df["next_src_cell"] == df["dst_cell"])
        & (df["next_dst_cell"] == df["src_cell"])
        & (df["dt_next_ho"] <= args.window)
    )
    df["reversal_15"] = reversal_mask.astype(int)

    # short_dwell_15
    df["short_dwell_15"] = (df["dwell_s"] <= args.window).astype(int)

    # unstable_15 = reversal OR short dwell
    df["unstable_15"] = ((df["reversal_15"] == 1) | (df["short_dwell_15"] == 1)).astype(int)

    # stable_final = not unstable_15
    df["stable_final"] = (df["unstable_15"] == 0).astype(int)

    # optional event id for debugging/modeling
    df.insert(0, "event_id", range(len(df)))

    # drop helper columns
    df = df.drop(columns=["next_t_ho", "next_src_cell", "next_dst_cell", "dt_next_ho"])

    df.to_csv(args.output, index=False)

    print(f"Wrote: {args.output}")
    print(f"Rows: {len(df)}")
    print(f"reversal_15: {int(df['reversal_15'].sum())}")
    print(f"short_dwell_15: {int(df['short_dwell_15'].sum())}")
    print(f"unstable_15: {int(df['unstable_15'].sum())}")
    print(f"stable_final: {int(df['stable_final'].sum())}")


if __name__ == "__main__":
    main()
