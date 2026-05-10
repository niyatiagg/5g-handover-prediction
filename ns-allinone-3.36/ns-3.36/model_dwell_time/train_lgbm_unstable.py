#!/usr/bin/env python3

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

from sklearn.metrics import (
    average_precision_score,
    precision_score,
    recall_score,
    f1_score,
    roc_auc_score,
)
from sklearn.model_selection import GroupShuffleSplit

import lightgbm as lgb


def compute_threshold_metrics(df: pd.DataFrame, prob_col: str, y_col: str) -> pd.DataFrame:
    thresholds = np.arange(0.1, 1.0, 0.1)

    rows = []
    n_total = len(df)
    n_unstable = int(df[y_col].sum())
    n_stable = int((df[y_col] == 0).sum())
    n_reversal = int(df["reversal_15"].sum())
    n_short = int(df["short_dwell_15"].sum())

    for t in thresholds:
        gated = df[prob_col] >= t

        gated_total = int(gated.sum())
        gated_unstable = int(((gated) & (df[y_col] == 1)).sum())
        gated_stable = int(((gated) & (df[y_col] == 0)).sum())
        gated_reversal = int(((gated) & (df["reversal_15"] == 1)).sum())
        gated_short = int(((gated) & (df["short_dwell_15"] == 1)).sum())

        rows.append(
            {
                "threshold": round(float(t), 3),
                "n_total": n_total,
                "n_gate": gated_total,
                "n_unstable": n_unstable,
                "n_stable": n_stable,
                "n_reversal_15": n_reversal,
                "n_short_dwell_15": n_short,
                "ho_reduction_pct": 100.0 * gated_total / n_total if n_total else 0.0,
                "unstable_reduction_pct": 100.0 * gated_unstable / n_unstable if n_unstable else 0.0,
                "stable_loss_pct": 100.0 * gated_stable / n_stable if n_stable else 0.0,
                "reversal_reduction_pct": 100.0 * gated_reversal / n_reversal if n_reversal else 0.0,
                "short_dwell_reduction_pct": 100.0 * gated_short / n_short if n_short else 0.0,
            }
        )

    return pd.DataFrame(rows)


def main():
    parser = argparse.ArgumentParser(description="Train LightGBM for unstable_15 prediction")
    parser.add_argument("--csv", required=True, help="Path to labeled ho_train CSV")
    parser.add_argument("--outdir", required=True, help="Output directory")
    parser.add_argument("--test-size", type=float, default=0.2, help="Grouped test split fraction")
    parser.add_argument("--random-state", type=int, default=42, help="Random seed")
    args = parser.parse_args()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(args.csv)

    required_cols = [
        "event_id",
        "ue_imsi",
        "t_ho",
        "src_cell",
        "dst_cell",
        "dwell_s",
        "reversal_15",
        "short_dwell_15",
        "unstable_15",
        "pre_rsrp",
        "pre_sinr_db_calc",
        "pre_distance_ue",
        "pre_gap_db",
        "pre_hysteresis",
        "pre_speed_ue",
        "pre_trigger_a3",
        "pre_trigger_kpi",
        "pre_dx_ue",
        "pre_dy_ue",
        "delta_gap_1s",
        "delta_sinr_1s",
        "avg_gap_last3",
        "avg_sinr_last3",
        "gap_slope_last3",
        "sinr_slope_last3",
    ]
    missing = [c for c in required_cols if c not in df.columns]
    if missing:
        raise ValueError(f"Missing required columns: {missing}")

    feature_cols = [
        "pre_rsrp",
        "pre_sinr_db_calc",
        "pre_distance_ue",
        "pre_gap_db",
        "pre_hysteresis",
        "pre_speed_ue",
        "pre_trigger_a3",
        "pre_trigger_kpi",
        "pre_dx_ue",
        "pre_dy_ue",
        "delta_gap_1s",
        "delta_sinr_1s",
        "avg_gap_last3",
        "avg_sinr_last3",
        "gap_slope_last3",
        "sinr_slope_last3",
    ]

    target_col = "unstable_15"
    group_col = "ue_imsi"

    model_df = df.dropna(subset=feature_cols + [target_col, group_col]).copy().reset_index(drop=True)

    X = model_df[feature_cols].copy()
    y = model_df[target_col].astype(int).copy()
    groups = model_df[group_col].copy()

    splitter = GroupShuffleSplit(
        n_splits=1,
        test_size=args.test_size,
        random_state=args.random_state,
    )
    train_idx, test_idx = next(splitter.split(X, y, groups=groups))

    train_df = model_df.iloc[train_idx].copy().reset_index(drop=True)
    test_df = model_df.iloc[test_idx].copy().reset_index(drop=True)

    X_train = train_df[feature_cols]
    y_train = train_df[target_col].astype(int)
    X_test = test_df[feature_cols]
    y_test = test_df[target_col].astype(int)

    n_pos = int(y_train.sum())
    n_neg = int((y_train == 0).sum())
    scale_pos_weight = (n_neg / n_pos) if n_pos > 0 else 1.0

    clf = lgb.LGBMClassifier(
        objective="binary",
        boosting_type="gbdt",
        n_estimators=300,
        learning_rate=0.05,
        num_leaves=31,
        max_depth=-1,
        min_child_samples=20,
        subsample=0.8,
        colsample_bytree=0.8,
        reg_alpha=0.0,
        reg_lambda=0.0,
        random_state=args.random_state,
        scale_pos_weight=scale_pos_weight,
        n_jobs=-1,
        verbosity=-1,
    )

    clf.fit(X_train, y_train)

    y_prob = clf.predict_proba(X_test)[:, 1]
    y_pred_05 = (y_prob >= 0.5).astype(int)

    metrics = {
        "n_total": int(len(model_df)),
        "n_train": int(len(train_df)),
        "n_test": int(len(test_df)),
        "n_train_positive": int(y_train.sum()),
        "n_test_positive": int(y_test.sum()),
        "positive_rate_train": float(y_train.mean()),
        "positive_rate_test": float(y_test.mean()),
        "pr_auc": float(average_precision_score(y_test, y_prob)),
        "roc_auc": float(roc_auc_score(y_test, y_prob)),
        "precision_at_0.5": float(precision_score(y_test, y_pred_05, zero_division=0)),
        "recall_at_0.5": float(recall_score(y_test, y_pred_05, zero_division=0)),
        "f1_at_0.5": float(f1_score(y_test, y_pred_05, zero_division=0)),
        "features": feature_cols,
        "model": "LightGBM",
        "scale_pos_weight": float(scale_pos_weight),
    }

    pred_cols = [
        "event_id",
        "ue_imsi",
        "t_ho",
        "src_cell",
        "dst_cell",
        "dwell_s",
        "reversal_15",
        "short_dwell_15",
        "unstable_15",
    ]
    preds_df = test_df[pred_cols].copy()
    preds_df["y_true"] = y_test.values
    preds_df["y_prob"] = y_prob
    preds_df["y_pred_0.5"] = y_pred_05

    preds_path = outdir / "lgbm_test_predictions.csv"
    preds_df.to_csv(preds_path, index=False)

    sweep_df = compute_threshold_metrics(preds_df, prob_col="y_prob", y_col="y_true")
    sweep_path = outdir / "lgbm_threshold_sweep.csv"
    sweep_df.to_csv(sweep_path, index=False)

    metrics_path = outdir / "lgbm_metrics.json"
    with open(metrics_path, "w") as f:
        json.dump(metrics, f, indent=2)

    try:
        imp_df = pd.DataFrame(
            {"feature": feature_cols, "importance": clf.feature_importances_}
        ).sort_values("importance", ascending=False).reset_index(drop=True)
        imp_path = outdir / "lgbm_feature_importance.csv"
        imp_df.to_csv(imp_path, index=False)
        print(f"Saved feature importance: {imp_path}")
    except Exception as e:
        print(f"Warning: could not save feature importances: {e}")

    print("=== LightGBM results ===")
    for k, v in metrics.items():
        if isinstance(v, float):
            print(f"{k} = {v:.6f}")
        else:
            print(f"{k} = {v}")

    print(f"\nSaved predictions: {preds_path}")
    print(f"Saved threshold sweep: {sweep_path}")
    print(f"Saved metrics: {metrics_path}")


if __name__ == "__main__":
    main()
