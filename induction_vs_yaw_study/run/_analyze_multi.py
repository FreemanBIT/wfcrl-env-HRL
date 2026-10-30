
import json
import numpy as np

with open(r"D:\HR_Project\wfcrl-env-HRL\induction_vs_yaw_study\results\_full_diag_data_7964.json", "r") as f:
    data = json.load(f)

multi = data["multi"]
baseline = data["baseline"]

print("MULTI scenario T1 analysis:")
print(f"{'Time':>6} {'T1_P':>8} {'T1_h':>8} {'T1_yaw_seq':>10} {'T1_ind_seq':>10}")
for i, (t, tb, sq) in enumerate(multi):
    if i % 5 == 0 or i >= len(multi) - 5:  # every 5 steps + last 5
        h, p = tb[0]
        ys, ins = sq[0]
        print(f"{t:>5}s {p:>8.0f} {np.degrees(h):>+7.1f}deg {ys:>10} {ins:>10}")

print("\nT1 power around t=240s:")
for i in range(75, min(85, len(multi))):
    t, tb, sq = multi[i]
    h, p = tb[0]
    ys, ins = sq[0]
    print(f"  t={t}s  P={p:.0f} kW  h={np.degrees(h):+.1f}deg  yaw_seq={ys}  ind_seq={ins}")

print("\nT2 state around t=240s:")
for i in range(75, min(85, len(multi))):
    t, tb, sq = multi[i]
    h, p = tb[1]
    ys, ins = sq[1]
    print(f"  t={t}s  P={p:.0f} kW  h={np.degrees(h):+.1f}deg  yaw_seq={ys}  ind_seq={ins}")

print("\nT3 power around t=240s:")
for i in range(75, min(85, len(multi))):
    t, tb, sq = multi[i]
    h, p = tb[2]
    ys, ins = sq[2]
    print(f"  t={t}s  P={p:.0f} kW  h={np.degrees(h):+.1f}deg  yaw_seq={ys}  ind_seq={ins}")

# Check if T1 heading goes crazy
print("\nT1 full heading history (every 10 steps):")
for i in range(0, len(multi), 10):
    t, tb, sq = multi[i]
    h, p = tb[0]
    print(f"  t={t:3d}s  h={np.degrees(h):+8.2f}deg  P={p:7.0f} kW")
