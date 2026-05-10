#!/usr/bin/env python3

import argparse
import json
import itertools
from pathlib import Path

import numpy as np
import pandas as pd

from sklearn.metrics import (
    average_precision_score,
    roc_auc_score,
    precision_score,
    recall_score,
    f1_score,
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


def point_metrics(y_true, y_prob, threshold=0.5):
    y_pred = (y_prob >= threshold).astype(int)
    return {
        "pr_auc": float(average_precision_score(y_true, y_prob)),
        "roc_auc": float(roc_auc_score(y_true, y_prob)),
        "precision_at_0.5": float(precision_score(y_true, y_pred, zero_division=0)),
        "recall_at_0.5": float(recall_score(y_true, y_pred, zero_division=0)),
        "f1_at_0.5": float(f1_score(y_true, y_pred, zero_division=0)),
    }


def main():
    parser = argparse.ArgumentParser(description="Small LightGBM hyperparameter sweep for unstable_15 prediction")
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

    n_pos = int(y_train.sum())
    n_neg = int((y_train == 0).sum())
    base_spw = (n_neg / n_pos) if n_pos > 0 else 1.0

    grid = {
        "num_leaves": [15, 31, 63],
        "max_depth": [3, 5, -1],
        "min_child_samples": [10, 20],
        "learning_rate": [0.03, 0.05],
        "n_estimators": [200, 400],
        "subsample": [0.8, 1.0],
        "colsample_bytree": [0.8, 1.0],
        "reg_lambda": [0.0, 1.0],
        "reg_alpha": [0.0],
        "scale_pos_weight_mult": [0.75, 1.0, 1.25],
    }

    keys = list(grid.keys())
    values = [grid[k] for k in keys]
    combos = list(itertools.product(*values))

    results = []
    best = None
    best_score = -1.0
    best_clf = None
    best_probs = None

    for i, combo in enumerate(combos, start=1):
        params = dict(zip(keys, combo))
        scale_pos_weight = base_spw * params.pop("scale_pos_weight_mult")

        clf = lgb.LGBMClassifier(
            objective="binary",
            boosting_type="gbdt",
            random_state=args.random_state,
            n_jobs=-1,
            verbosity=-1,
            scale_pos_weight=scale_pos_weight,
            **params,
        )

        clf.fit(X_train, y_train)
        y_prob = clf.predict_proba(X_test)[:, 1]
        metrics = point_metrics(y_test, y_prob, threshold=0.5)

        row = {
            "trial": i,
            "scale_pos_weight": float(scale_pos_weight),
            **params,
            **metrics,
        }
        results.append(row)

        if metrics["pr_auc"] > best_score:
            best_score = metrics["pr_auc"]
            best = row
            best_clf = clf
            best_probs = y_prob

        print(
            f"trial={i}/{len(combos)} "
            f"pr_auc={metrics['pr_auc']:.6f} "
            f"roc_auc={metrics['roc_auc']:.6f} "
            f"f1@0.5={metrics['f1_at_0.5']:.6f} "
            f"params={params} scale_pos_weight={scale_pos_weight:.4f}"
        )

    results_df = pd.DataFrame(results).sort_values(
        ["pr_auc", "roc_auc", "f1_at_0.5"],
        ascending=[False, False, False]
    ).reset_index(drop=True)

    results_path = outdir / "lgbm_tuning_results.csv"
    results_df.to_csv(results_path, index=False)

    metrics = {
        "n_total": int(len(model_df)),
        "n_train": int(len(train_df)),
        "n_test": int(len(test_df)),
        "n_train_positive": int(y_train.sum()),
        "n_test_positive": int(y_test.sum()),
        "positive_rate_train": float(y_train.mean()),
        "positive_rate_test": float(y_test.mean()),
        "features": feature_cols,
        "model": "LightGBM_tuned",
        "best_by": "pr_auc",
        "best_result": best,
    }

    metrics_path = outdir / "lgbm_tuned_best_metrics.json"
    with open(metrics_path, "w") as f:
        json.dump(metrics, f, indent=2)

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
    preds_df["y_prob"] = best_probs
    preds_df["y_pred_0.5"] = (best_probs >= 0.5).astype(int)

    preds_path = outdir / "lgbm_tuned_test_predictions.csv"
    preds_df.to_csv(preds_path, index=False)

    sweep_df = compute_threshold_metrics(preds_df, prob_col="y_prob", y_col="y_true")
    sweep_path = outdir / "lgbm_tuned_threshold_sweep.csv"
    sweep_df.to_csv(sweep_path, index=False)

    try:
        imp_df = pd.DataFrame(
            {"feature": feature_cols, "importance": best_clf.feature_importances_}
        ).sort_values("importance", ascending=False).reset_index(drop=True)
        imp_path = outdir / "lgbm_tuned_feature_importance.csv"
        imp_df.to_csv(imp_path, index=False)
        print(f"Saved feature importance: {imp_path}")
    except Exception as e:
        print(f"Warning: could not save feature importances: {e}")

    print("\n=== Best tuned LightGBM result ===")
    for k, v in best.items():
        if isinstance(v, float):
            print(f"{k} = {v:.6f}")
        else:
            print(f"{k} = {v}")

    print(f"\nSaved tuning table: {results_path}")
    print(f"Saved best metrics: {metrics_path}")
    print(f"Saved test predictions: {preds_path}")
    print(f"Saved threshold sweep: {sweep_path}")


if __name__ == "__main__":
    main()
