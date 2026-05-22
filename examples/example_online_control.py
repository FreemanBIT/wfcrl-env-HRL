"""
Online Optimization Control with FAST.Farm
============================================
Demonstrates closed-loop online optimization architecture.
At each DT_low step, the controller measures current output and
decides the next control input.

Architecture:
    FastFarmOnlineInterface.step(yaw, pitch)
        -> runs 1 FAST.Farm time step
        -> returns per-turbine power measurements
    Controller.optimize(measurements)
        -> computes next yaw/pitch based on measurements

Usage:
    python examples/example_online_control.py
"""
import os, sys, time, numpy as np, pandas as pd
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

N_STEPS = 4; SIM_DT = 3.0; WIND_SPEED = 10.0; WIND_DIR = 270.0

import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt; import seaborn as sns; sns.set_theme(style="darkgrid")
from wfcrl.environments.data_cases import named_cases_dictionary
base = named_cases_dictionary["DafengH1_"][0]
from wfcrl.interface import FastFarmOnlineInterface, FarmControllerBase


class OnlineYawController(FarmControllerBase):
    """
    Example online controller: adjusts yaw based on power measurements.
    Simple rule-based optimization:
    - If farm power dropped from last step, reverse yaw direction
    - If farm power increased, continue in same direction
    """
    def __init__(self, n_turbines):
        super().__init__(n_turbines)
        self.prev_power = 0.0
        self.yaw = 0.0
        self.direction = 1.0

    def optimize(self, measurement):
        current_power = measurement.get('farm_power_mw', 0)
        if self.prev_power > 0 and current_power < self.prev_power * 0.98:
            self.direction *= -1
        self.yaw += self.direction * 5.0
        self.yaw = np.clip(self.yaw, -25, 25)
        self.prev_power = current_power
        return {'yaw': np.full(self.n_turbines, self.yaw),
                'pitch': np.zeros(self.n_turbines)}


if __name__ == '__main__':
    print("="*70)
    print("Online Optimization Control with FAST.Farm")
    print("="*70)

    base = fastfarm_DafengH1; n_turb = base.num_turbines
    print(f"\nTurbines: {n_turb} | DT: {SIM_DT}s | Steps: {N_STEPS}")

    ts = time.time()
    out_dir = os.path.join(os.path.dirname(__file__), "..",
        "__simul__", "fastfarm", "online_control", f"DafengH1_Online_{ts:.0f}")
    os.makedirs(out_dir, exist_ok=True)

    config = base.dict()
    config["max_iter"] = N_STEPS
    config["dt"] = SIM_DT; config["speed"] = WIND_SPEED; config["t_init"] = 0

    interface = FastFarmOnlineInterface(config, out_dir)
    interface.setup()

    controller = OnlineYawController(n_turb)

    # =====================================================================
    # Online control loop
    # =====================================================================
    print(f"\n{'='*70}")
    print("Online Control Loop:")
    print(f"{'='*70}")

    for step in range(N_STEPS):
        # Controller computes next action based on previous step's measurement
        controls = controller.optimize({
            'farm_power_mw': controller.prev_power,
            'step': step,
        }) if step > 0 else {'yaw': np.zeros(n_turb), 'pitch': np.zeros(n_turb)}

        yaw_cmd = controls['yaw'][0]
        pitch_cmd = controls['pitch'][0]

        print(f"\n  Step {step+1}/{N_STEPS}: yaw={yaw_cmd:+.1f} deg")
        meas = interface.step(float(yaw_cmd), float(pitch_cmd))
        print(f"    Farm power: {meas['farm_power_mw']:.2f} MW")
        print(f"    Turbine range: {meas['power_mw'].min():.2f} - {meas['power_mw'].max():.2f} MW")

    # =====================================================================
    # Results
    # =====================================================================
    history = interface.step_history
    steps = [h['step'] for h in history]
    times = [h['time'] for h in history]
    farm_p = [h['farm_power_mw'] for h in history]
    yaw_cmds = [h['yaw_cmd'] for h in history]

    pdf = pd.DataFrame({
        'step': steps, 'time_s': times,
        'farm_power_MW': farm_p, 'yaw_cmd_deg': yaw_cmds,
    })
    pdf.to_csv(os.path.join(out_dir, "online_control_results.csv"), index=False)
    print(f"\nSaved: online_control_results.csv")

    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    ax = axes[0]; ax.plot(times, farm_p, 'b-o', lw=2)
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Farm Power (MW)")
    ax.set_title(f"Online Optimization — Farm Power Response")
    ax.grid(True, alpha=0.3)

    ax = axes[1]; ax.step(times, yaw_cmds, 'r-', where='post', lw=2)
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Yaw Command (deg)")
    ax.set_title("Control Signal (updated every DT_low step)")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    fp = os.path.join(out_dir, "online_control_response.png")
    fig.savefig(fp, dpi=150, bbox_inches="tight"); plt.close(fig)
    print(f"Saved: online_control_response.png")

    print(f"\n{'='*70}\nSUMMARY\n{'='*70}")
    print(f"Controller: OnlineYawController (rule-based)")
    print(f"Steps: {len(history)} x {SIM_DT}s = {times[-1]+SIM_DT if times else 0:.0f}s")
    print(f"Mean farm power: {np.mean(farm_p):.2f} MW")
    print(f"Output: {out_dir}")
    print(f"{'='*70}")
