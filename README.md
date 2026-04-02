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

The project uses **SUMO** for mobility generation and **ns-3 LTE** for radio and handover simulation.

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

## Final Results

### Logistic Regression
- PR-AUC: **0.1207**
- ROC-AUC: **0.8570**
- F1@0.5: **0.1526**

### EBM
- PR-AUC: **0.1815**
- ROC-AUC: **0.8892**

### LightGBM
- PR-AUC: **0.1774**
- ROC-AUC: **0.8849**
- F1@0.5: **0.2088**

### XGBoost
- PR-AUC: **0.1912**
- ROC-AUC: **0.8930**
- F1@0.5: **0.2018**

## Best Offline Gating Tradeoff

The project also evaluates models as **offline gating policies**, where predicted instability scores are used to suppress risky handovers.

### Best balanced practical result
**LightGBM at threshold 0.7**
- unstable-handover reduction: **48.42%**
- stable-handover loss: **7.59%**

### Strongest ranking model
**XGBoost**
- best PR-AUC and ROC-AUC on the final 100k dataset

## Repository Structure

Typical project files include:

- `make_dwell_tables.py` or mobility-aware variant for HO event extraction
- `add_final_labels.py` for event-level instability labels
- `add_temporal_features_to_hotrain.py` for temporal deltas
- training scripts for Logistic Regression, EBM, LightGBM, and XGBoost
- threshold sweep CSVs and result plots
- report / paper draft files

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
   - offline gating tradeoffs

## Key Takeaways

- Predicting post-handover instability is harder than reproducing threshold-based trigger logic.
- Boosted-tree models outperform the linear baseline on this task.
- **LightGBM** provides the best balanced policy behavior.
- **XGBoost** provides the strongest overall ranking performance.
- Short-term temporal radio trends help boosted-tree models more than Logistic Regression.

## Future Work

Possible next steps include:

- online deployment of the gating model inside the simulator
- richer temporal sequence modeling
- direct comparison with conditional handover or reinforcement learning approaches
- extension to 5G-native mobility procedures

## Author Notes

This work was implemented honestly as an **offline event-level instability prediction study** using simulator-generated mobility and radio traces.  
Any online deployment architecture should be treated as a future extension rather than a completed protocol-level contribution.
