"""
B1 — FLORIDyn dynamic wake model.

Adds observation-point (OP) advection to the steady FLORIS surrogate so that
wake-propagation delay and inflow heterogeneity are represented explicitly
(derivation doc eqs 2.1-2.9).

Design (simplified, control-oriented):
  * Each turbine emits a chain of OPs. Each OP carries the emitting turbine's
    control state (yaw, induction) at emission time and advects downstream at
    the local advection speed (eq 2.1-2.4). Implemented as a per-turbine shift
    register of "control history" sampled at the model step.
  * To evaluate turbine i's effective inflow, we build a Temporary Wind Farm
    (TWF): for each upstream turbine j, we look up the control state that j had
    tau_{j->i} seconds ago (the OP that is now arriving at i), and evaluate the
    steady wake with that *delayed* control (eq 2.5-2.7). Superposition is
    sum-of-squares, reusing the analytical Gaussian wake in
    :class:`SurrogateModel`.

This captures the essential closed-loop-relevant dynamics (upstream action
affects downstream only after tau) at low cost, while delegating the spatial
wake shape to the same steady model Scheme A calibrates.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from ..surrogate import SurrogateModel
from ..induction import D_ROTOR


@dataclass
class FLORIDynConfig:
    dt: float = 2.0                 # model step (s), align with FAST.Farm DT
    max_delay_s: float = 240.0      # longest tracked delay (history buffer length)
    advection_factor: float = 1.0   # U_adv = factor * U_eff (approx free stream)


class FLORIDyn:
    """Dynamic wrapper around the steady surrogate with OP-advection delay.

    Parameters
    ----------
    surrogate:
        Shared steady surrogate (M5) providing the wake shape and superposition.
    layout_x, layout_y:
        Turbine coordinates (m).
    config:
        FLORIDyn configuration.
    """

    def __init__(
        self,
        surrogate: SurrogateModel,
        layout_x: list[float],
        layout_y: list[float],
        config: Optional[FLORIDynConfig] = None,
    ):
        self.sur = surrogate
        self.x = np.asarray(layout_x, float)
        self.y = np.asarray(layout_y, float)
        self.n = len(layout_x)
        self.cfg = config or FLORIDynConfig()
        self.D = D_ROTOR

        # state
        self._U_inf = 8.0
        self._phi = 270.0
        self._TI = 0.06
        self._yaw = np.zeros(self.n)
        self._a = np.full(self.n, 1.0 / 3.0)

        # per-turbine control history (shift register of (yaw, a) each dt)
        self._hist_len = max(2, int(self.cfg.max_delay_s / self.cfg.dt) + 2)
        self._yaw_hist: list[deque] = [
            deque([0.0] * self._hist_len, maxlen=self._hist_len) for _ in range(self.n)
        ]
        self._a_hist: list[deque] = [
            deque([1.0 / 3.0] * self._hist_len, maxlen=self._hist_len)
            for _ in range(self.n)
        ]
        # cached effective speeds (updated each step)
        self._u_eff = np.full(self.n, self._U_inf)

    # -- configuration ---------------------------------------------------------
    def set_conditions(self, U_inf: float, phi: float, TI: float) -> None:
        self._U_inf, self._phi, self._TI = float(U_inf), float(phi), float(TI)

    def set_controls(self, yaw: np.ndarray, induction: np.ndarray) -> None:
        self._yaw = np.asarray(yaw, float).reshape(self.n)
        self._a = np.asarray(induction, float).reshape(self.n)

    # -- geometry --------------------------------------------------------------
    def _streamwise(self, phi_deg: float):
        toward = math.radians((phi_deg + 180.0) % 360.0)
        dirx, diry = math.sin(toward), math.cos(toward)
        s = self.x * dirx + self.y * diry
        c = -self.x * diry + self.y * dirx
        return s, c

    def _delay_steps(self, dist: float, u_adv: float) -> int:
        """tau = dist / u_adv, in whole model steps (eq 2.5)."""
        u_adv = max(u_adv, 0.5)
        tau_s = dist / (self.cfg.advection_factor * u_adv)
        return int(round(tau_s / self.cfg.dt))

    # -- dynamics --------------------------------------------------------------
    def step(self, dt: Optional[float] = None) -> None:
        """Advance one model step: push current controls into the history
        registers (OP emission), then recompute effective speeds from the TWF
        with delayed upstream controls."""
        # emit OPs: record current control state at the queue head
        for i in range(self.n):
            self._yaw_hist[i].appendleft(float(self._yaw[i]))
            self._a_hist[i].appendleft(float(self._a[i]))
        self._recompute_effective()

    def _recompute_effective(self) -> None:
        """Build the TWF for every turbine using delayed upstream controls and
        evaluate the steady wake (eq 2.6-2.7)."""
        s, c = self._streamwise(self._phi)
        u_adv = max(self._U_inf, 0.5)

        # Build an effective yaw/induction field *as seen at each downstream
        # turbine*. Because the steady surrogate computes the full field from one
        # global control vector, we approximate per-target delay by using, for
        # each ordered upstream turbine, its delayed control. We evaluate the
        # surrogate once per target with the delayed upstream controls relevant
        # to that target.
        u_eff = np.full(self.n, float(self._U_inf))
        for i in range(self.n):
            # assemble a control vector where each upstream j uses its delayed
            # value (tau_{j->i}); turbine i uses its current value.
            yaw_delayed = self._yaw.copy()
            a_delayed = self._a.copy()
            for j in range(self.n):
                if j == i:
                    continue
                dx = s[i] - s[j]
                if dx <= 1e-3:
                    continue  # j not upstream of i
                d = self._delay_steps(dx, u_adv)
                d = min(d, self._hist_len - 1)
                yaw_delayed[j] = self._yaw_hist[j][d]
                a_delayed[j] = self._a_hist[j][d]
            self.sur.set_conditions(self._U_inf, self._phi, self._TI)
            self.sur.set_controls(yaw_delayed, a_delayed)
            u_eff[i] = float(self.sur.predict()["U_eff_i"][i])
        self._u_eff = u_eff

    # -- outputs ---------------------------------------------------------------
    def get_measurements(self) -> dict:
        """Return current per-turbine effective speed and power (for EnKF h())."""
        from ..induction import cp_from_induction, A_ROTOR, RHO_AIR, CP_YAW_EXP
        p_w = np.array([
            0.5 * RHO_AIR * A_ROTOR * cp_from_induction(self._a[i]) * (self._u_eff[i] ** 3)
            * (max(0.0, math.cos(math.radians(self._yaw[i]))) ** CP_YAW_EXP)
            for i in range(self.n)
        ])
        return dict(U_eff_i=self._u_eff.copy(), P_i=p_w / 1e3, P_farm=float(p_w.sum()) / 1e3)

    def predict(self, horizon_steps: int, control_seq) -> np.ndarray:
        """Roll the model forward ``horizon_steps`` under a control sequence and
        return the farm-power trajectory (kW). Used by the MPC.

        ``control_seq`` is a callable ``k -> (yaw[n], a[n])`` giving the controls
        applied at prediction step k (0-based). The model state is snapshotted
        and restored so prediction does not disturb the live state.
        """
        snap = self._snapshot()
        traj = np.zeros(horizon_steps)
        for k in range(horizon_steps):
            yaw_k, a_k = control_seq(k)
            self.set_controls(yaw_k, a_k)
            self.step()
            traj[k] = self.get_measurements()["P_farm"]
        self._restore(snap)
        return traj

    def steady_power(self, yaw: np.ndarray, a: np.ndarray) -> float:
        """Farm power (kW) the given *constant* control settles to.

        For a held control, every OP register eventually fills with that same
        control, so the settled FLORIDyn field is identical to the steady
        surrogate evaluated with that control (verified numerically). We
        therefore read it directly from the surrogate — O(1) instead of rolling
        tau_max steps — while still honouring the delay-coverage requirement
        (eq 2.22): the settled value is exactly the horizon->infinity limit.

        Transient effects of *changing* a control are handled separately by
        :meth:`transient_cost`.
        """
        self.sur.set_conditions(self._U_inf, self._phi, self._TI)
        self.sur.set_controls(yaw, a)
        return float(self.sur.predict()["P_farm"])

    def transient_cost(self, yaw: np.ndarray, a: np.ndarray, steps: int) -> float:
        """Mean farm-power *deficit* during the first ``steps`` of a control
        change relative to its settled value (a proxy for transient loss).
        Positive means the transient underperforms the steady target.
        """
        snap = self._snapshot()
        vals = []
        for _ in range(steps):
            self.set_controls(yaw, a)
            self.step()
            vals.append(self.get_measurements()["P_farm"])
        self._restore(snap)
        settled = vals[-1] if vals else 0.0
        return float(settled - np.mean(vals)) if vals else 0.0

    # -- state snapshot for observers / MPC ------------------------------------
    def _snapshot(self):
        return (
            self._yaw.copy(), self._a.copy(), self._u_eff.copy(),
            [list(h) for h in self._yaw_hist], [list(h) for h in self._a_hist],
            self._U_inf, self._phi, self._TI,
        )

    def _restore(self, snap) -> None:
        (self._yaw, self._a, self._u_eff, yh, ah,
         self._U_inf, self._phi, self._TI) = (
            snap[0].copy(), snap[1].copy(), snap[2].copy(),
            snap[3], snap[4], snap[5], snap[6], snap[7],
        )
        self._yaw_hist = [deque(h, maxlen=self._hist_len) for h in yh]
        self._a_hist = [deque(h, maxlen=self._hist_len) for h in ah]

    def get_state(self) -> np.ndarray:
        """Return the estimator-facing state vector: [U_inf, phi, TI]. (The OP
        registers are internal; the EnKF corrects the ambient state and wake
        params, which is what observability supports from power measurements.)"""
        return np.array([self._U_inf, self._phi, self._TI], float)

    def set_state(self, state: np.ndarray) -> None:
        self._U_inf, self._phi, self._TI = float(state[0]), float(state[1]), float(state[2])

    def max_delay_steps(self, U_inf: Optional[float] = None) -> int:
        """Largest tau (in steps) across all ordered pairs — used to size the MPC
        prediction horizon (eq 2.22)."""
        s, _ = self._streamwise(self._phi)
        u_adv = max(U_inf or self._U_inf, 0.5)
        dmax = 0
        for i in range(self.n):
            for j in range(self.n):
                dx = s[i] - s[j]
                if dx > 1e-3:
                    dmax = max(dmax, self._delay_steps(dx, u_adv))
        return dmax
