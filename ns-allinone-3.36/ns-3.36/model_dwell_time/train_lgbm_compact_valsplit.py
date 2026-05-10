#!/usr/bin/env python3

import argparse
import itertools
import json
from pathlib import Path

import numpy as np
import pandas as pd
import lightgbm as lgb

from sklearn.metrics import (
    average_precision_score,
    roc_auc_score,
    precision_score,
    recall_score,
    f1_score,
)
from sklearn.model_selection import GroupShuffleSplit


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
        y_pred = gated.astype(int)

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
                "precision": float(precision_score(df[y_col], y_pred, zero_division=0)),
                "recall": float(recall_score(df[y_col], y_pred, zero_division=0)),
                "f1": float(f1_score(df[y_col], y_pred, zero_division=0)),
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
        "precision": float(precision_score(y_true, y_pred, zero_division=0)),
        "recall": float(recall_score(y_true, y_pred, zero_division=0)),
        "f1": float(f1_score(y_true, y_pred, zero_division=0)),
    }


def choose_best_f1_threshold(sweep_df: pd.DataFrame):
    idx = sweep_df["f1"].idxmax()
    return sweep_df.loc[idx].to_dict()


def choose_best_threshold_under_stable_loss(sweep_df: pd.DataFrame, max_stable_loss=10.0):
    eligible = sweep_df[sweep_df["stable_loss_pct"] <= max_stable_loss].copy()
    if len(eligible) == 0:
        return None
    idx = eligible["unstable_reduction_pct"].idxmax()
    return eligible.loc[idx].to_dict()


def main():
    parser = argparse.ArgumentParser(description="Compact LightGBM tuning with grouped train/val/test split")
    parser.add_argument("--csv", required=True, help="Path to labeled ho_train CSV")
    parser.add_argument("--outdir", required=True, help="Output directory")
    parser.add_argument("--test-size", type=float, default=0.2, help="Grouped test fraction")
    parser.add_argument("--val-size", type=float, default=0.2, help="Grouped validation fraction from train pool")
    parser.add_argument("--random-state", type=int, default=42, help="Random seed")
    parser.add_argument("--max-stable-loss", type=float, default=10.0, help="Constraint for threshold choice")
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
    ]

    target_col = "unstable_15"
    group_col = "ue_imsi"

    model_df = df.dropna(subset=feature_cols + [target_col, group_col]).copy().reset_index(drop=True)

    X = model_df[feature_cols].copy()
    y = model_df[target_col].astype(int).copy()
    groups = model_df[group_col].copy()

    # Split off test
    gss_test = GroupShuffleSplit(n_splits=1, test_size=args.test_size, random_state=args.random_state)
    train_pool_idx, test_idx = next(gss_test.split(X, y, groups=groups))

    train_pool_df = model_df.iloc[train_pool_idx].copy().reset_index(drop=True)
    test_df = model_df.iloc[test_idx].copy().reset_index(drop=True)

    X_pool = train_pool_df[feature_cols]
    y_pool = train_pool_df[target_col].astype(int)
    groups_pool = train_pool_df[group_col]

    # Split train/val
    gss_val = GroupShuffleSplit(n_splits=1, test_size=args.val_size, random_state=args.random_state)
    train_idx, val_idx = next(gss_val.split(X_pool, y_pool, groups=groups_pool))

    train_df = train_pool_df.iloc[train_idx].copy().reset_index(drop=True)
    val_df = train_pool_df.iloc[val_idx].copy().reset_index(drop=True)

    X_train = train_df[feature_cols]
    y_train = train_df[target_col].astype(int)
    X_val = val_df[feature_cols]
    y_val = val_df[target_col].astype(int)
    X_test = test_df[feature_cols]
    y_test = test_df[target_col].astype(int)

    n_pos = int(y_train.sum())
    n_neg = int((y_train == 0).sum())
    base_spw = (n_neg / n_pos) if n_pos > 0 else 1.0

    grid = {
        "num_leaves": [31, 63],
        "max_depth": [5, -1],
        "min_child_samples": [10, 20],
        "learning_rate": [0.03, 0.05],
        "n_estimators": [300, 500],
        "subsample": [0.8, 1.0],
        "colsample_bytree": [0.8, 1.0],
        "reg_lambda": [0.0, 1.0],
        "scale_pos_weight_mult": [0.75, 1.0, 1.25],
    }

    keys = list(grid.keys())
    values = [grid[k] for k in keys]
    combos = list(itertools.product(*values))

    tuning_rows = []
    best_score = -1.0
    best_params = None
    best_clf = None
    best_val_prob = None

    for i, combo in enumerate(combos, start=1):
        params = dict(zip(keys, combo))
        scale_pos_weight = base_spw * params.pop("scale_pos_weight_mult")

        clf = lgb.LGBMClassifier(
            objective="binary",
            boosting_type="gbdt",
            random_state=args.random_state,
            n_jobs=-1,
            verbosity=-1,
            reg_alpha=0.0,
            scale_pos_weight=scale_pos_weight,
            **params,
        )

        clf.fit(X_train, y_train)
        val_prob = clf.predict_proba(X_val)[:, 1]

        pr_auc = average_precision_score(y_val, val_prob)
        roc_auc = roc_auc_score(y_val, val_prob)
        f1_05 = f1_score(y_val, (val_prob >= 0.5).astype(int), zero_division=0)

        row = {
            "trial": i,
            **params,
            "scale_pos_weight": float(scale_pos_weight),
            "val_pr_auc": float(pr_auc),
            "val_roc_auc": float(roc_auc),
            "val_f1_at_0.5": float(f1_05),
        }
        tuning_rows.append(row)

        if pr_auc > best_score:
            best_score = pr_auc
            best_params = row
            best_clf = clf
            best_val_prob = val_prob

        print(
            f"trial={i}/{len(combos)} "
            f"val_pr_auc={pr_auc:.6f} "
            f"val_roc_auc={roc_auc:.6f} "
            f"val_f1@0.5={f1_05:.6f}"
        )

    tuning_df = pd.DataFrame(tuning_rows).sort_values(
        ["val_pr_auc", "val_roc_auc", "val_f1_at_0.5"],
        ascending=[False, False, False]
    ).reset_index(drop=True)
    tuning_df.to_csv(outdir / "lgbm_compact_tuning_results.csv", index=False)

    val_preds = val_df[
        ["event_id", "ue_imsi", "t_ho", "src_cell", "dst_cell", "dwell_s", "reversal_15", "short_dwell_15", "unstable_15"]
    ].copy()
    val_preds["y_true"] = y_val.values
    val_preds["y_prob"] = best_val_prob

    val_sweep = compute_threshold_metrics(val_preds, prob_col="y_prob", y_col="y_true")
    val_sweep.to_csv(outdir / "lgbm_validation_threshold_sweep.csv", index=False)

    best_f1_thresh = choose_best_f1_threshold(val_sweep)
    best_constraint_thresh = choose_best_threshold_under_stable_loss(
        val_sweep, max_stable_loss=args.max_stable_loss
    )

    test_prob = best_clf.predict_proba(X_test)[:, 1]
    test_preds = test_df[
        ["event_id", "ue_imsi", "t_ho", "src_cell", "dst_cell", "dwell_s", "reversal_15", "short_dwell_15", "unstable_15"]
    ].copy()
    test_preds["y_true"] = y_test.values
    test_preds["y_prob"] = test_prob
    test_preds.to_csv(outdir / "lgbm_test_predictions.csv", index=False)

    test_sweep = compute_threshold_metrics(test_preds, prob_col="y_prob", y_col="y_true")
    test_sweep.to_csv(outdir / "lgbm_test_threshold_sweep.csv", index=False)

    raw_test_metrics_05 = point_metrics(y_test, test_prob, threshold=0.5)

    test_metrics_best_f1 = None
    test_metrics_constraint = None

    if best_f1_thresh is not None:
        thr = float(best_f1_thresh["threshold"])
        test_metrics_best_f1 = point_metrics(y_test, test_prob, threshold=thr)

    if best_constraint_thresh is not None:
        thr = float(best_constraint_thresh["threshold"])
        test_metrics_constraint = point_metrics(y_test, test_prob, threshold=thr)

    metrics = {
        "n_total": int(len(model_df)),
        "n_train": int(len(train_df)),
        "n_val": int(len(val_df)),
        "n_test": int(len(test_df)),
        "n_train_positive": int(y_train.sum()),
        "n_val_positive": int(y_val.sum()),
        "n_test_positive": int(y_test.sum()),
        "positive_rate_train": float(y_train.mean()),
        "positive_rate_val": float(y_val.mean()),
        "positive_rate_test": float(y_test.mean()),
        "features": feature_cols,
        "model": "LightGBM_compact_tuned_valsplit",
        "best_model_by_validation_pr_auc": best_params,
        "validation_best_f1_threshold": best_f1_thresh,
        "validation_best_threshold_under_stable_loss": best_constraint_thresh,
        "test_metrics_at_0.5": raw_test_metrics_05,
        "test_metrics_at_validation_best_f1_threshold": test_metrics_best_f1,
        "test_metrics_at_validation_constraint_threshold": test_metrics_constraint,
    }

    with open(outdir / "lgbm_compact_best_metrics.json", "w") as f:
        json.dump(metrics, f, indent=2)

    try:
        imp_df = pd.DataFrame(
            {"feature": feature_cols, "importance": best_clf.feature_importances_}
        ).sort_values("importance", ascending=False).reset_index(drop=True)
        imp_df.to_csv(outdir / "lgbm_compact_feature_importance.csv", index=False)
    except Exception as e:
        print(f"Warning: could not save feature importances: {e}")

    print("\n=== Best compact LightGBM model ===")
    for k, v in best_params.items():
        if isinstance(v, float):
            print(f"{k} = {v:.6f}")
        else:
            print(f"{k} = {v}")

    print("\n=== Test metrics at threshold 0.5 ===")
    for k, v in raw_test_metrics_05.items():
        print(f"{k} = {v:.6f}")

    if best_f1_thresh is not None:
        print("\n=== Validation-best-F1 threshold ===")
        print(best_f1_thresh)
        print("=== Test metrics at validation-best-F1 threshold ===")
        for k, v in test_metrics_best_f1.items():
            print(f"{k} = {v:.6f}")

    if best_constraint_thresh is not None:
        print("\n=== Validation-best threshold under stable-loss constraint ===")
        print(best_constraint_thresh)
        print("=== Test metrics at validation constraint threshold ===")
        for k, v in test_metrics_constraint.items():
            print(f"{k} = {v:.6f}")


if __name__ == "__main__":
    main()
