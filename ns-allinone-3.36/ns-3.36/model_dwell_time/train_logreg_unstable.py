#!/usr/bin/env python3

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

from sklearn.compose import ColumnTransformer
from sklearn.impute import SimpleImputer
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import (
    average_precision_score,
    precision_score,
    recall_score,
    f1_score,
    roc_auc_score,
)
from sklearn.model_selection import GroupShuffleSplit
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler


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
    parser = argparse.ArgumentParser(description="Train Logistic Regression for unstable_15 prediction")
    parser.add_argument("--csv", required=True, help="Path to labeled ho_train CSV")
    parser.add_argument("--outdir", required=True, help="Output directory")
    parser.add_argument("--test-size", type=float, default=0.2, help="Grouped test split fraction")
    parser.add_argument("--random-state", type=int, default=42, help="Random seed")
    parser.add_argument(
        "--C",
        type=float,
        default=1.0,
        help="Inverse regularization strength for Logistic Regression"
    )
    args = parser.parse_args()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(args.csv)

    required_cols = [
        "ue_imsi",
        "unstable_15",
        "reversal_15",
        "short_dwell_15",
        "pre_rsrp",
        "pre_sinr_db_calc",
        "pre_distance_ue",
        "pre_gap_db",
        "pre_hysteresis",
        "pre_speed_ue",
    ]
    missing = [c for c in required_cols if c not in df.columns]
    if missing:
        raise ValueError(f"Missing required columns: {missing}")

    # Core features only
    feature_cols = [
        "pre_rsrp",
        "pre_sinr_db_calc",
        "pre_distance_ue",
        "pre_gap_db",
        "pre_hysteresis",
        "pre_speed_ue",
    ]

    target_col = "unstable_15"
    group_col = "ue_imsi"

    X = df[feature_cols].copy()
    y = df[target_col].astype(int).copy()
    groups = df[group_col].copy()

    # Grouped train/test split by UE
    splitter = GroupShuffleSplit(
        n_splits=1,
        test_size=args.test_size,
        random_state=args.random_state
    )
    train_idx, test_idx = next(splitter.split(X, y, groups=groups))

    train_df = df.iloc[train_idx].copy().reset_index(drop=True)
    test_df = df.iloc[test_idx].copy().reset_index(drop=True)

    X_train = train_df[feature_cols]
    y_train = train_df[target_col].astype(int)
    X_test = test_df[feature_cols]
    y_test = test_df[target_col].astype(int)

    numeric_features = feature_cols

    numeric_transformer = Pipeline(
        steps=[
            ("imputer", SimpleImputer(strategy="median")),
            ("scaler", StandardScaler()),
        ]
    )

    preprocessor = ColumnTransformer(
        transformers=[
            ("num", numeric_transformer, numeric_features),
        ]
    )

    model = Pipeline(
        steps=[
            ("preprocessor", preprocessor),
            (
                "classifier",
                LogisticRegression(
                    C=args.C,
                    class_weight="balanced",
                    max_iter=2000,
                    solver="liblinear",
                    random_state=args.random_state,
                ),
            ),
        ]
    )

    model.fit(X_train, y_train)

    y_prob = model.predict_proba(X_test)[:, 1]
    y_pred_05 = (y_prob >= 0.5).astype(int)

    metrics = {
        "n_total": int(len(df)),
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
        "C": float(args.C),
        "features": feature_cols,
    }

    # Save predictions
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
    pred_cols = [c for c in pred_cols if c in test_df.columns]

    preds_df = test_df[pred_cols].copy()
    preds_df["y_true"] = y_test.values
    preds_df["y_prob"] = y_prob
    preds_df["y_pred_0.5"] = y_pred_05

    preds_path = outdir / "logreg_test_predictions.csv"
    preds_df.to_csv(preds_path, index=False)

    # Threshold sweep for policy metrics
    sweep_df = compute_threshold_metrics(preds_df, prob_col="y_prob", y_col="y_true")
    sweep_path = outdir / "logreg_threshold_sweep.csv"
    sweep_df.to_csv(sweep_path, index=False)

    # Save metrics JSON
    metrics_path = outdir / "logreg_metrics.json"
    with open(metrics_path, "w") as f:
        json.dump(metrics, f, indent=2)

    print("=== Logistic Regression results ===")
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
