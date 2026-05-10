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
from sklearn.calibration import CalibratedClassifierCV

from interpret.glassbox import ExplainableBoostingClassifier


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


def metric_block(y_true, y_prob, threshold=0.5):
    y_pred = (y_prob >= threshold).astype(int)
    return {
        "pr_auc": float(average_precision_score(y_true, y_prob)),
        "roc_auc": float(roc_auc_score(y_true, y_prob)),
        f"precision_at_{threshold}": float(precision_score(y_true, y_pred, zero_division=0)),
        f"recall_at_{threshold}": float(recall_score(y_true, y_pred, zero_division=0)),
        f"f1_at_{threshold}": float(f1_score(y_true, y_pred, zero_division=0)),
    }


def main():
    parser = argparse.ArgumentParser(description="Train EBM + calibration for unstable_15 prediction")
    parser.add_argument("--csv", required=True, help="Path to labeled ho_train CSV")
    parser.add_argument("--outdir", required=True, help="Output directory")
    parser.add_argument("--test-size", type=float, default=0.2, help="Grouped test split fraction")
    parser.add_argument("--random-state", type=int, default=42, help="Random seed")
    parser.add_argument(
        "--calibration",
        choices=["sigmoid", "isotonic"],
        default="sigmoid",
        help="Calibration method"
    )
    parser.add_argument(
        "--decision-threshold",
        type=float,
        default=0.5,
        help="Threshold used only for reporting point metrics"
    )
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

    base_ebm = ExplainableBoostingClassifier(
        random_state=args.random_state,
        interactions=10,
        outer_bags=8,
        inner_bags=0,
        learning_rate=0.01,
        max_rounds=5000,
        min_samples_leaf=2,
    )

    # fit raw EBM
    base_ebm.fit(X_train, y_train)
    y_prob_raw = base_ebm.predict_proba(X_test)[:, 1]

    # fit calibrated EBM on the same training set using internal CV
    calib_ebm = ExplainableBoostingClassifier(
        random_state=args.random_state,
        interactions=10,
        outer_bags=8,
        inner_bags=0,
        learning_rate=0.01,
        max_rounds=5000,
        min_samples_leaf=2,
    )

    calibrated_model = CalibratedClassifierCV(
        estimator=calib_ebm,
        method=args.calibration,
        cv=3,
    )
    calibrated_model.fit(X_train, y_train)
    y_prob_cal = calibrated_model.predict_proba(X_test)[:, 1]

    raw_metrics = metric_block(y_test, y_prob_raw, threshold=args.decision_threshold)
    cal_metrics = metric_block(y_test, y_prob_cal, threshold=args.decision_threshold)

    metrics = {
        "n_total": int(len(model_df)),
        "n_train": int(len(train_df)),
        "n_test": int(len(test_df)),
        "n_train_positive": int(y_train.sum()),
        "n_test_positive": int(y_test.sum()),
        "positive_rate_train": float(y_train.mean()),
        "positive_rate_test": float(y_test.mean()),
        "features": feature_cols,
        "model": "EBM + calibration",
        "calibration_method": args.calibration,
        "decision_threshold_for_point_metrics": float(args.decision_threshold),
        "raw_ebm": raw_metrics,
        "calibrated_ebm": cal_metrics,
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
    preds_df["y_prob_raw_ebm"] = y_prob_raw
    preds_df["y_prob_calibrated_ebm"] = y_prob_cal
    preds_df[f"y_pred_raw_at_{args.decision_threshold}"] = (y_prob_raw >= args.decision_threshold).astype(int)
    preds_df[f"y_pred_cal_at_{args.decision_threshold}"] = (y_prob_cal >= args.decision_threshold).astype(int)

    preds_path = outdir / "ebm_calibrated_test_predictions.csv"
    preds_df.to_csv(preds_path, index=False)

    raw_sweep_df = compute_threshold_metrics(preds_df, prob_col="y_prob_raw_ebm", y_col="y_true")
    raw_sweep_path = outdir / "ebm_raw_threshold_sweep.csv"
    raw_sweep_df.to_csv(raw_sweep_path, index=False)

    cal_sweep_df = compute_threshold_metrics(preds_df, prob_col="y_prob_calibrated_ebm", y_col="y_true")
    cal_sweep_path = outdir / "ebm_calibrated_threshold_sweep.csv"
    cal_sweep_df.to_csv(cal_sweep_path, index=False)

    metrics_path = outdir / "ebm_calibrated_metrics.json"
    with open(metrics_path, "w") as f:
        json.dump(metrics, f, indent=2)

    try:
        term_names = base_ebm.term_names_
        term_scores = base_ebm.term_importances()
        imp_df = pd.DataFrame({"term": term_names, "importance": term_scores})
        imp_df = imp_df.sort_values("importance", ascending=False).reset_index(drop=True)
        imp_path = outdir / "ebm_term_importance.csv"
        imp_df.to_csv(imp_path, index=False)
        print(f"Saved term importance: {imp_path}")
    except Exception as e:
        print(f"Warning: could not save term importances: {e}")

    print("=== Raw EBM results ===")
    for k, v in raw_metrics.items():
        print(f"{k} = {v:.6f}")

    print("\n=== Calibrated EBM results ===")
    for k, v in cal_metrics.items():
        print(f"{k} = {v:.6f}")

    print(f"\nSaved predictions: {preds_path}")
    print(f"Saved raw threshold sweep: {raw_sweep_path}")
    print(f"Saved calibrated threshold sweep: {cal_sweep_path}")
    print(f"Saved metrics: {metrics_path}")


if __name__ == "__main__":
    main()
