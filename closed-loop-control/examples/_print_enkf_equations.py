#!/usr/bin/env python
"""Print EnKF state equations and fusion process diagram."""
print(r"""
╔══════════════════════════════════════════════════════════════════════════════╗
║              EnKF STATE ESTIMATION — Full Derivation                         ║
╚══════════════════════════════════════════════════════════════════════════════╝

1.  STATE VECTOR (dim=5,  augmented with wake params)
╔══════════════════════════════════════════════╗
║   x = [U_inf,  φ,  TI,  ka,  kb]^T         ║
║   U_inf  : free-stream wind speed (m/s)     ║
║   φ      : wind direction (°)               ║
║   TI     : turbulence intensity (-)         ║
║   ka, kb : FLORIDyn wake expansion params   ║
╚══════════════════════════════════════════════╝


2.  FORECAST STEP (eq 2.12) — random-walk + inflation
╔══════════════════════════════════════════════════════════════════════╗
║                                                                      ║
║   x_k^(e)-  =  x_{k-1}^(e)+  +  w_k^(e)      w ~ N(0, Q)           ║
║                                                                      ║
║   then:  x ← x̄ + λ · (x − x̄)     inflation λ=1.02                   ║
║                                                                      ║
║   Q = diag( q_U², q_φ², q_TI², q_ka², q_kb² )                      ║
║        q_U=0.05, q_φ=0.2, q_TI=0.002,                               ║
║        q_ka=0.002, q_kb=0.0002                                       ║
║                                                                      ║
╚══════════════════════════════════════════════════════════════════════╝


3.  OBSERVATION OPERATOR h(x) — predicted measurements
╔══════════════════════════════════════════════════════════════════════╗
║                                                                      ║
║   y_pred^{(e)} = h( x_k^{(e)-},  γ,  a )                             ║
║                                                                      ║
║   h(x):                                                              ║
║     surrogate.set_params({ka, kb})                                   ║
║     surrogate.set_conditions(U, φ, TI)                               ║
║     surrogate.set_controls(γ, a)                                     ║
║     surrogate._recompute_effective()   ← FLORIDyn 尾流计算           ║
║     → P_i = [P_1, P_2, ..., P_6]  (kW)                              ║
║                                                                      ║
║   维度:  y ∈ R^6  (per-turbine power only)                           ║
║   若 use_wind_meas=True: y ∈ R^12  (+ nacelle wind speed)            ║
║                                                                      ║
╚══════════════════════════════════════════════════════════════════════╝


4.  MEASUREMENT VECTOR y_meas
╔══════════════════════════════════════════════════════════════════════╗
║                                                                      ║
║   y_meas = [P_1(kW), P_2(kW), ..., P_6(kW)]^T  ← FAST.Farm         ║
║            genpwr from DISCON measurement files                      ║
║                                                                      ║
║   y_meas = y_true + ε      ε ~ N(0, R)                              ║
║                                                                      ║
║   R = diag( σ_1², σ_2², ..., σ_6² )                                 ║
║   σ_i = max( 0.03×P_i, 20.0 )  kW                                   ║
║         (3% relative  +  20kW absolute floor)                       ║
║                                                                      ║
╚══════════════════════════════════════════════════════════════════════╝


5.  ENSEMBLE STATISTICS — eq 2.13-2.16
╔══════════════════════════════════════════════════════════════════════╗
║                                                                      ║
║   40 ensemble members:  {x^(e)}  e=1..40                            ║
║                                                                      ║
║   Ensemble mean:      x̄  = 1/N Σ x^(e)       (5×1)                  ║
║                       ȳ  = 1/N Σ h(x^(e))    (6×1)                  ║
║                                                                      ║
║   Anomalies:          Ex = [x^(1)−x̄, ..., x^(40)−x̄]   (5×40)       ║
║                       Ey = [y_pred^(1)−ȳ, ..., ]       (6×40)       ║
║                                                                      ║
║   Covariances:        P_xy = (Ex·Ey^T) / 39         (5×6)           ║
║                       P_yy = (Ey·Ey^T) / 39         (6×6)           ║
║                                                                      ║
╚══════════════════════════════════════════════════════════════════════╝


6.  KALMAN GAIN & ANALYSIS — eq 2.17-2.18
╔══════════════════════════════════════════════════════════════════════╗
║                                                                      ║
║                          S = P_yy + R                                ║
║                          K = P_xy · S^{-1}                           ║
║                              ┌─ state-space  ─┐ ┌─ measurement ─┐   ║
║                              │  5 rows        │ │   6 columns    │   ║
║                              │ (ambient+param)│ │  (per-turbine) │   ║
║                                                                      ║
║   Analysis (perturbed-observation EnKF):                             ║
║     D^{(e)} = y_meas + η^{(e)}           η ~ N(0, R)                 ║
║                                                                      ║
║     x_k^{(e)+} = x_k^{(e)-} + K · ( D^{(e)} − y_pred^{(e)} )         ║
║                   ───────────   ─── ──────────────────                ║
║                   prior         K ×  innovation (6-dim)               ║
║                                                                      ║
║   Posterior mean:  x_post = 1/N Σ x_k^{(e)+}  → write into model    ║
║                                                                      ║
╚══════════════════════════════════════════════════════════════════════╝


7.  COMPLETE FUSION LOOP

  ┌───────────────────────────────────────────────────────────────────┐
  │                                                                   │
  │   FAST.Farm                    FLORIDyn (surrogate)               │
  │   ═══════════                  ═══════════════════                 │
  │   P_i_meas (kW)                h(x^e, γ, a) → P_i_pred (kW)      │
  │       │                              │                            │
  │       │   y_meas (6,)               │   Y_pred (6×40)            │
  │       │                              │                            │
  │       └──────────┬───────────────────┘                            │
  │                  │                                                │
  │                  ▼                                                │
  │     ╔══════════════════════════╗                                   │
  │     ║     KALMAN FUSION       ║                                   │
  │     ║                         ║                                   │
  │     ║  innovation =           ║                                   │
  │     ║    y_meas − ȳ  (dim 6)  ║                                   │
  │     ║                         ║                                   │
  │     ║  gain K = Pxy/(Pyy+R)   ║                                   │
  │     ║    (5×6 matrix)         ║                                   │
  │     ║                         ║                                   │
  │     ║  Δx = K × innovation(i) ║                                   │
  │     ║  ΔU_inf, Δφ, ΔTI,      ║                                   │
  │     ║  Δka, Δkb              ║                                   │
  │     ║                         ║                                   │
  │     ║  x_post = x_prior + Δx  ║                                   │
  │     ╚══════════╤══════════════╝                                   │
  │                │                                                  │
  │                ▼                                                  │
  │     ╔══════════════════════════╗                                   │
  │     ║   CORRECTED AMBIENT     ║                                   │
  │     ║   U_inf, φ, TI          ║                                   │
  │     ║   ka, kb                ║                                   │
  │     ╚══════════╤══════════════╝                                   │
  │                │                                                  │
  │                ▼                                                  │
  │         MPC.solve(U_inf, φ, TI)                                   │
  │         → (γ_opt, a_opt)                                          │
  │         → DISCON                                                 │
  │                                                                   │
  │   下一步:  random-walk drift → forecast → ...  (back to top)      │
  │                                                                   │
  └───────────────────────────────────────────────────────────────────┘
""")
