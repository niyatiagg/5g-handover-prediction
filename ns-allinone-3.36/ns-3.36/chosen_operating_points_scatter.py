#!/usr/bin/env python3

import os
import pandas as pd
import matplotlib.pyplot as plt


# ---------------------------------
# EDIT THESE PATHS
# ---------------------------------
LR_SWEEP = "/home/ubuntu/cs297/ns-allinone-3.36/ns-3.36/model_dwell_time/model_results/100k/logreg_100k_extended_p1.0/logreg_threshold_sweep.csv"
EBM_SWEEP = "/home/ubuntu/cs297/ns-allinone-3.36/ns-3.36/model_dwell_time/model_results/100k/ebm_100k_extended_model/ebm_threshold_sweep.csv"
LGBM_SWEEP = "/home/ubuntu/cs297/ns-allinone-3.36/ns-3.36/model_dwell_time/model_results/100k/lgbm_100k_extended/lgbm_threshold_sweep.csv"
XGB_SWEEP = "/home/ubuntu/cs297/ns-allinone-3.36/ns-3.36/model_dwell_time/model_results/100k/xgb_100k_temporal/xgb_threshold_sweep.csv"

OUTDIR = "/home/ubuntu/cs297/ns-allinone-3.36/ns-3.36/model_dwell_time/model_results/final_plots_100k"
os.makedirs(OUTDIR, exist_ok=True)

# chosen operating points
CHOSEN_THRESHOLDS = {
    "Logistic Regression": 0.7,
    "EBM": 0.1,
    "LightGBM": 0.7,
    "XGBoost": 0.7,
}


def load_sweep(path, model_name):
    df = pd.read_csv(path, sep=None, engine="python")
    df["model"] = model_name
    return df


def get_row_at_threshold(df, threshold):
    row = df.loc[(df["threshold"] - threshold).abs() < 1e-9]
    if len(row) == 0:
        raise ValueError(f"Threshold {threshold} not found for model {df['model'].iloc[0]}")
    return row.iloc[0]


def main():
    lr = load_sweep(LR_SWEEP, "Logistic Regression")
    ebm = load_sweep(EBM_SWEEP, "EBM")
    lgbm = load_sweep(LGBM_SWEEP, "LightGBM")
    xgb = load_sweep(XGB_SWEEP, "XGBoost")

    sweeps = [lr, ebm, lgbm, xgb]

    chosen_rows = []
    for df in sweeps:
        model = df["model"].iloc[0]
        thr = CHOSEN_THRESHOLDS[model]
        r = get_row_at_threshold(df, thr)
        chosen_rows.append({
            "model": model,
            "threshold": thr,
            "stable_loss_pct": r["stable_loss_pct"],
            "unstable_reduction_pct": r["unstable_reduction_pct"],
            "ho_reduction_pct": r["ho_reduction_pct"],
            "reversal_reduction_pct": r["reversal_reduction_pct"],
        })

    chosen_df = pd.DataFrame(chosen_rows)
    chosen_df.to_csv(os.path.join(OUTDIR, "chosen_points_for_scatter.csv"), index=False)

    fig, ax = plt.subplots(figsize=(8, 6))

    ax.scatter(
        chosen_df["stable_loss_pct"],
        chosen_df["unstable_reduction_pct"],
        s=120
    )

    for _, row in chosen_df.iterrows():
        label = f"{row['model']} (thr={row['threshold']})"
        ax.annotate(
            label,
            (row["stable_loss_pct"], row["unstable_reduction_pct"]),
            xytext=(6, 6),
            textcoords="offset points"
        )

    ax.set_xlabel("Stable-HO loss (%)")
    ax.set_ylabel("Unstable-HO reduction (%)")
    ax.set_title("Chosen operating points: stable loss vs unstable reduction")
    ax.grid(alpha=0.3)

    plt.tight_layout()
    outpath = os.path.join(OUTDIR, "chosen_operating_points_scatter.png")
    plt.savefig(outpath, dpi=300, bbox_inches="tight")
    plt.close()

    print(f"Saved: {outpath}")
    print(chosen_df)


if __name__ == "__main__":
    main()
