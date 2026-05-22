"""
HyCon/ROSCO-Style Farm-Level Control with FAST.Farm
====================================================
Uses FastFarmStandaloneInterface + FarmControllerBase from wfcrl.interface.
Each control segment runs as an independent FAST.Farm simulation with its own
yaw/pitch setpoint and TMax.

Usage:
    python examples/example_hycon_farm_control.py
"""
import os, sys, time, numpy as np, pandas as pd
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

SIM_DT = 3.0; SIM_TOTAL = 30.0; WIND_SPEED = 10.0; WIND_DIR = 270.0
CONTROL_SCHEDULE = [(10.0, 0.0, 0.0), (10.0, 15.0, 2.0), (10.0, -10.0, 0.0)]

import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt; import seaborn as sns; sns.set_theme(style="darkgrid")
from wfcrl.environments.data_cases import named_cases_dictionary
base = named_cases_dictionary["DafengH1_"][0]
from wfcrl.interface import FastFarmStandaloneInterface, FarmControllerBase


class ScheduleController(FarmControllerBase):
    """Schedule-based controller following a predefined yaw/pitch schedule."""
    def __init__(self, n_turbines, schedule):
        super().__init__(n_turbines)
        self.schedule = schedule
        self.seg_idx = 0
        self.t_in_seg = 0.0

    def compute_controls(self, meas):
        seg_dur = self.schedule[self.seg_idx][0]
        self.t_in_seg += meas.get('dt', SIM_DT)
        if self.t_in_seg >= seg_dur and self.seg_idx < len(self.schedule) - 1:
            self.seg_idx += 1; self.t_in_seg = 0.0
            print(f"  -> Seg {self.seg_idx+1}: yaw={self.schedule[self.seg_idx][1]:.0f} deg")
        _, y, p = self.schedule[self.seg_idx]
        return {'yaw': np.full(self.n_turbines, y), 'pitch': np.full(self.n_turbines, p)}


if __name__ == '__main__':
    print("="*70); print("HyCon/ROSCO-Style Farm-Level Control"); print("="*70)
    base = fastfarm_DafengH1; n_turb = base.num_turbines
    print(f"\nSchedule: {len(CONTROL_SCHEDULE)} segments, {SIM_TOTAL:.0f}s")
    for i,(d,y,p) in enumerate(CONTROL_SCHEDULE):
        print(f"  S{i+1}: {d:.0f}s | yaw={y:+.0f} deg pitch={p:.0f} deg")

    ts = time.time(); out_dir = os.path.join(os.path.dirname(__file__),"..",
        "__simul__","fastfarm","hycon_control",f"DafengH1_HyCon_{ts:.0f}")
    os.makedirs(out_dir, exist_ok=True)

    all_power = []; all_time = []; cum_t = 0.0
    for si, (sd, yc, pc) in enumerate(CONTROL_SCHEDULE):
        n_steps = int(np.ceil(sd / SIM_DT))
        actual_t = n_steps * SIM_DT
        print(f"\nSeg {si+1}: yaw={yc:+.0f} deg pitch={pc:+.0f} deg ({actual_t:.0f}s, {n_steps} steps)")

        config = base.dict()
        config["max_iter"] = n_steps; config["dt"] = SIM_DT
        config["speed"] = WIND_SPEED; config["t_init"] = 0

        seg_dir = os.path.join(out_dir, f"seg{si+1}")
        os.makedirs(seg_dir, exist_ok=True)
        ff = FastFarmStandaloneInterface(config, seg_dir)
        ff.setup()
        ff.set_yaw_pitch(yc, pc)
        meas = ff.run()

        if meas['power_mw'] is not None:
            t_seg = meas['time'] + cum_t if meas['time'] is not None else np.arange(n_steps)*SIM_DT + cum_t
            all_time.append(t_seg)
            all_power.append(meas['power_mw'])
            seg_mean = meas['power_mw'].sum(axis=1).mean()
            print(f"  -> {meas['power_mw'].shape[1]}/{n_turb} turbines, mean = {seg_mean:.2f} MW")
        cum_t += actual_t

    if not all_power: print("No power data!"); sys.exit(1)

    t_full = np.concatenate(all_time)
    p_full = np.vstack(all_power)
    tp_full = p_full.sum(axis=1)

    cols = [f"T{i+1}_power_MW" for i in range(n_turb)]
    pdf = pd.DataFrame(p_full, columns=cols)
    pdf.insert(0, "time_s", t_full); pdf.insert(1, "total_power_MW", tp_full)
    pdf.to_csv(os.path.join(out_dir, "powers_timeseries.csv"), index=False, float_format="%.6f")
    print(f"\nSaved: powers_timeseries.csv ({len(t_full)} steps)")

    # Plot: subplot 1 = total farm power, subplot 2 = first 6 turbine powers
    fig, axes = plt.subplots(2, 1, figsize=(14, 9))
    colors = ['#e6f3ff','#fff3e6','#e6ffe6']

    ax = axes[0]; ax.plot(t_full, tp_full, 'b-', lw=2)
    cum = 0
    for si, (sd, yc, _) in enumerate(CONTROL_SCHEDULE):
        ax.axvspan(cum, cum+sd, alpha=0.15, color=colors[si%3])
        if len(tp_full) > 0: y_pos = tp_full.max() * 0.92
        else: y_pos = 10
        ax.text(cum+sd/2, y_pos, f"S{si+1}\nyaw={yc:.0f} deg", ha='center', fontsize=9)
        cum += sd
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Total Farm Power (MW)")
    ax.set_title(f"DafengH1 — Farm Power Response (Wind: {WIND_SPEED} m/s, {WIND_DIR} deg)")
    ax.grid(True, alpha=0.3); ax.set_xlim(0, t_full[-1])

    ax = axes[1]; n_plot = min(6, n_turb)
    for ti in range(n_plot):
        ax.plot(t_full, p_full[:, ti], label=f"T{ti+1}", lw=1.5, alpha=0.8)
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Turbine Power (MW)")
    ax.set_title(f"Individual Turbine Power (first {n_plot} of {n_turb})")
    ax.legend(ncol=3, fontsize=8); ax.grid(True, alpha=0.3); ax.set_xlim(0, t_full[-1])

    plt.tight_layout()
    fp = os.path.join(out_dir, "farm_power_response.png")
    fig.savefig(fp, dpi=150, bbox_inches="tight"); plt.close(fig)
    print(f"Saved: farm_power_response.png")

    print(f"\n{'='*70}\nSUMMARY\n{'='*70}")
    print(f"Turbines: {n_turb} x NREL 5MW | Wind: {WIND_SPEED} m/s, {WIND_DIR} deg")
    print(f"Simulation: {t_full[-1]:.0f}s ({len(t_full)} steps)")
    cum = 0
    for si, (sd, yc, pc) in enumerate(CONTROL_SCHEDULE):
        mask = (t_full >= cum) & (t_full < cum+sd)
        if mask.any(): print(f"  S{si+1}: yaw={yc:+.0f} deg pitch={pc:.0f} deg -> {tp_full[mask].mean():.2f} MW")
        cum += sd
    print(f"Overall: {tp_full.mean():.2f} MW | Output: {out_dir}")
