# Machine Learning-Based Unstable Handover Prediction

This project studies **unstable handover prediction** in dense cellular networks using an **ns-3 + SUMO + Python** workflow.  
The goal is to identify handovers that are likely to become low-value or unstable shortly after execution, so they can be reduced through a predictive gating approach.

## Project Overview

Modern dense cellular deployments improve coverage and capacity, but they also increase the frequency of handovers.  
Many of these handovers may provide only short-lived benefit, causing:

- short dwell time in the target cell
- rapid reversal to the previous cell
- extra signaling overhead
- reduced mobility efficiency

This project formulates handover instability as a **machine learning classification problem** using pre-handover radio and mobility features.

## Main Idea

The system predicts whether a handover is likely to be unstable using a label defined as:

`unstable_15 = reversal_15 OR short_dwell_15`

Where:

- `reversal_15` = the UE returns to the previous cell within 15 seconds
- `short_dwell_15` = dwell time in the target cell is 15 seconds or less

In the final dataset, `short_dwell_15` acts as a superset of `reversal_15`, so instability is mainly driven by short dwell while reversal remains an important stricter subtype.

## Simulation and Dataset

The project uses **SUMO** for mobility generation and **ns-3** for radio and handover simulation.

### Final 100k dataset summary

- Per-second dataset rows: **4,999,950**
- Event-level handover rows (`ho_events.csv`): **50,997**
- Event-level training rows (`ho_train.csv`): **50,997**
- Number of UEs: **50**
- Number of serving cells: **8**
- Simulation duration: **99,998 s**
- Trigger A3 count: **1,530**
- Trigger KPI count: **34,662**
- `reversal_15` count: **897**
- `short_dwell_15` count: **1,476**
- `unstable_15` count: **1,476**
- Positive rate: **0.028943**

## Feature Set

The models use a combination of:

- radio features
- distance / geometry features
- mobility features
- trigger-state features
- short-term temporal trend features

### Main features used

- `pre_rsrp`
- `pre_sinr_db_calc`
- `pre_distance_ue`
- `pre_gap_db`
- `pre_hysteresis`
- `pre_speed_ue`
- `pre_trigger_a3`
- `pre_trigger_kpi`
- `pre_dx_ue`
- `pre_dy_ue`

### Temporal features

- `delta_gap_1s`
- `delta_sinr_1s`

## Models Evaluated

The following models were tested:

- **Logistic Regression** - baseline
- **EBM (Explainable Boosting Machine)** - interpretable nonlinear model
- **LightGBM** - best balanced practical model
- **XGBoost** - best ranking model

## Workflow

1. Generate mobility in **SUMO**
2. Export mobility traces for **ns-3**
3. Run LTE handover simulation in **ns-3**
4. Create per-second dataset
5. Extract handover events and pre-handover training rows
6. Add instability labels and temporal features
7. Train and evaluate multiple ML models
8. Compare models using:
   - PR-AUC
   - ROC-AUC
   - F1
   - offline & online gating tradeoffs
