#!/usr/bin/env python3

import os
import pandas as pd
import matplotlib.pyplot as plt


# -----------------------------
# EDIT THESE PATHS
# -----------------------------
LR_SWEEP = "/home/ubuntu/cs297/ns-allinone-3.36/ns-3.36/model_dwell_time/model_results/100k/logreg_100k_extended_p1.0/logreg_threshold_sweep.csv"
EBM_SWEEP = "/home/ubuntu/cs297/ns-allinone-3.36/ns-3.36/model_dwell_time/model_results/100k/ebm_100k_extended_model/ebm_threshold_sweep.csv"
LGBM_SWEEP = "/home/ubuntu/cs297/ns-allinone-3.36/ns-3.36/model_dwell_time/model_results/100k/lgbm_100k_extended_temporal_summary/lgbm_threshold_sweep.csv"
XGB_SWEEP = "/home/ubuntu/cs297/ns-allinone-3.36/ns-3.36/model_dwell_time/model_results/100k/xgb_100k_temporal/xgb_threshold_sweep.csv"

OUTDIR = "/home/ubuntu/cs297/ns-allinone-3.36/ns-3.36/model_dwell_time/model_results/final_plots_100k_latest"
os.makedirs(OUTDIR, exist_ok=True)

# chosen operating points for summary bars
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


def save_grouped_bar(summary_df, value_cols, title, ylabel, outpath):
    x = range(len(summary_df))
    width = 0.18

    fig, ax = plt.subplots(figsize=(10, 6))

    for i, col in enumerate(value_cols):
        offsets = [xi + (i - (len(value_cols) - 1) / 2) * width for xi in x]
        ax.bar(offsets, summary_df[col], width=width, label=col)

    ax.set_xticks(list(x))
    ax.set_xticklabels(summary_df["model"], rotation=15)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()
    ax.grid(axis="y", alpha=0.3)

    plt.tight_layout()
    plt.savefig(outpath, dpi=300, bbox_inches="tight")
    plt.close()


def main():
    lr = load_sweep(LR_SWEEP, "Logistic Regression")
    ebm = load_sweep(EBM_SWEEP, "EBM")
    lgbm = load_sweep(LGBM_SWEEP, "LightGBM")
    xgb = load_sweep(XGB_SWEEP, "XGBoost")

    sweeps = [lr, ebm, lgbm, xgb]

    # ---------------------------------------
    # 1) Summary table at chosen thresholds
    # ---------------------------------------
    rows = []
    for df in sweeps:
        model = df["model"].iloc[0]
        thr = CHOSEN_THRESHOLDS[model]
        r = get_row_at_threshold(df, thr)

        rows.append({
            "model": model,
            "threshold": thr,
            "ho_reduction_pct": r["ho_reduction_pct"],
            "unstable_reduction_pct": r["unstable_reduction_pct"],
            "stable_loss_pct": r["stable_loss_pct"],
            "reversal_reduction_pct": r["reversal_reduction_pct"],
            "short_dwell_reduction_pct": r["short_dwell_reduction_pct"],
        })

    summary_df = pd.DataFrame(rows)
    summary_csv = os.path.join(OUTDIR, "chosen_operating_points_summary.csv")
    summary_df.to_csv(summary_csv, index=False)
    print(f"Saved: {summary_csv}")

    # ---------------------------------------
    # 2) Balanced operating point comparison
    # ---------------------------------------
    save_grouped_bar(
        summary_df,
        ["unstable_reduction_pct", "stable_loss_pct", "reversal_reduction_pct"],
        "Balanced operating point comparison across models",
        "Percentage",
        os.path.join(OUTDIR, "balanced_operating_point_comparison.png"),
    )

    # ---------------------------------------
    # 3) Baseline vs proposed grouped bars
    # baseline = LR, proposed = LightGBM
    # ---------------------------------------
    lr_row = summary_df[summary_df["model"] == "Logistic Regression"].iloc[0]
    lgbm_row = summary_df[summary_df["model"] == "LightGBM"].iloc[0]

    baseline_vs_proposed = pd.DataFrame([
        {
            "approach": "Baseline (LR)",
            "ho_reduction_pct": lr_row["ho_reduction_pct"],
            "unstable_reduction_pct": lr_row["unstable_reduction_pct"],
            "stable_loss_pct": lr_row["stable_loss_pct"],
            "reversal_reduction_pct": lr_row["reversal_reduction_pct"],
        },
        {
            "approach": "Proposed (LightGBM)",
            "ho_reduction_pct": lgbm_row["ho_reduction_pct"],
            "unstable_reduction_pct": lgbm_row["unstable_reduction_pct"],
            "stable_loss_pct": lgbm_row["stable_loss_pct"],
            "reversal_reduction_pct": lgbm_row["reversal_reduction_pct"],
        },
    ])

    fig, ax = plt.subplots(figsize=(10, 6))
    metrics = [
        "ho_reduction_pct",
        "unstable_reduction_pct",
        "stable_loss_pct",
        "reversal_reduction_pct",
    ]
    x = range(len(metrics))
    width = 0.35

    ax.bar([i - width / 2 for i in x], baseline_vs_proposed.iloc[0][metrics], width=width, label=baseline_vs_proposed.iloc[0]["approach"])
    ax.bar([i + width / 2 for i in x], baseline_vs_proposed.iloc[1][metrics], width=width, label=baseline_vs_proposed.iloc[1]["approach"])

    ax.set_xticks(list(x))
    ax.set_xticklabels([
        "HO reduction",
        "Unstable reduction",
        "Stable loss",
        "Reversal reduction",
    ], rotation=15)
    ax.set_ylabel("Percentage")
    ax.set_title("Baseline vs proposed approach")
    ax.legend()
    ax.grid(axis="y", alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(OUTDIR, "baseline_vs_proposed_bars.png"), dpi=300, bbox_inches="tight")
    plt.close()

    # ---------------------------------------
    # 4) Tradeoff curve: stable loss vs unstable reduction
    # ---------------------------------------
    fig, ax = plt.subplots(figsize=(8, 6))
    for df in sweeps:
        ax.plot(df["stable_loss_pct"], df["unstable_reduction_pct"], marker="o", label=df["model"].iloc[0])

    ax.set_xlabel("Stable-HO loss (%)")
    ax.set_ylabel("Unstable-HO reduction (%)")
    ax.set_title("Offline gating tradeoff curve")
    ax.legend()
    ax.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(OUTDIR, "tradeoff_curve_stable_vs_unstable.png"), dpi=300, bbox_inches="tight")
    plt.close()

    # ---------------------------------------
    # 5) Threshold vs unstable reduction
    # ---------------------------------------
    fig, ax = plt.subplots(figsize=(8, 6))
    for df in sweeps:
        ax.plot(df["threshold"], df["unstable_reduction_pct"], marker="o", label=df["model"].iloc[0])

    ax.set_xlabel("Threshold")
    ax.set_ylabel("Unstable-HO reduction (%)")
    ax.set_title("Threshold sweep: unstable-handover reduction")
    ax.legend()
    ax.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(OUTDIR, "threshold_vs_unstable_reduction.png"), dpi=300, bbox_inches="tight")
    plt.close()

    # ---------------------------------------
    # 6) Threshold vs stable loss
    # ---------------------------------------
    fig, ax = plt.subplots(figsize=(8, 6))
    for df in sweeps:
        ax.plot(df["threshold"], df["stable_loss_pct"], marker="o", label=df["model"].iloc[0])

    ax.set_xlabel("Threshold")
    ax.set_ylabel("Stable-HO loss (%)")
    ax.set_title("Threshold sweep: stable-handover loss")
    ax.legend()
    ax.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(OUTDIR, "threshold_vs_stable_loss.png"), dpi=300, bbox_inches="tight")
    plt.close()

    print(f"All plots saved in: {OUTDIR}")


if __name__ == "__main__":
    main()
