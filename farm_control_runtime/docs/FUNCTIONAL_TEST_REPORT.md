# FAST.Farm Farm Control Interface Functional Test Report

Date: 2026-10-16
Code: dev/farm-control-runtime (d686563 + WIN-05 fix)
Environment: Windows 10, FAST.Farm v2.10.6, ROSCO v2.10.6

---

## 1. Test Environment

| Item | Value |
|------|-------|
| FAST.Farm | v2.10.6 (Intel Fortran 2025) |
| ROSCO | v2.10.6 (gfortran 10.3.0) |
| DISCON_WT1.dll | 2,402,776 bytes |
| Compiler | TDM-GCC-64 10.3.0 |

## 2. Wind Condition

| Parameter | Value |
|-----------|-------|
| Wind Speed | 8 m/s |
| TI | 10% |
| Direction | 0 deg |
| Seed | 1 |
| Spacing | 6D |
| Duration | 300s |
| Control DT | 3s |

## 3. Layout

T1 (upstream) -> T2 (mid, T1 wake) -> T3 (downstream, T1+T2 wake)

---

## 4. Baseline Test (No Commands)

Command: none (pure ROSCO greedy)

| Metric | Value |
|--------|-------|
| Farm Total Power | 4366 kW |
| Steps | 100 (300s / 3s) |
| Wall Time | ~460s |

Per-Turbine Power:
| Turbine | Power (kW) | Note |
|---------|----------|------|
| T1 | 1633 | upstream, no wake |
| T2 | 1450 | mid, T1 wake |
| T3 | 1283 | downstream, T1+T2 wake |

Result: PASS - consistent with historical baseline

---

## 5. Induction Channel Test

Command: T1 induction a=0.20, T2/T3 no command
Mode: legacy (bridge_enable=0)

Farm Power: baseline 4366 kW -> test 4232 kW (-3.1%)

Per-Turbine:
| Turbine | Baseline (kW) | Test (kW) | Change | Note |
|---------|---------------|-----------|--------|------|
| T1 | 1633 | 1494 | -8.5% | derating active |
| T2 | 1450 | 1412 | -2.6% | reduced wake |
| T3 | 1283 | 1326 | +3.4% | wake recovery |

State Readback:
| Turbine | yaw_seq | ind_seq | Note |
|---------|---------|---------|------|
| T1 | 0 | 1 | induction applied |
| T2 | 0 | 0 | no cross-talk |
| T3 | 0 | 0 | no cross-talk |

Result: PASS - induction channel works independently

---

## 6. Yaw Channel Test

Command: T1 yaw_delta=+10deg, T2/T3 no command
Mode: legacy (bridge_enable=0)

Farm Power: baseline 4366 kW -> test 4367 kW (+0.0%)

Per-Turbine:
| Turbine | Baseline (kW) | Test (kW) | Change | Note |
|---------|---------------|-----------|--------|------|
| T1 | 1633 | 1686 | +3.2% | yaw misalignment |
| T2 | 1450 | 1354 | -6.6% | wake deflection |
| T3 | 1283 | 1327 | +3.4% | possible benefit |

Nacelle Heading:
| Turbine | Baseline (rad) | Test (rad) | Deflection (deg) |
|---------|---------------|------------|-----------------|
| T1 | ~0 | -0.174 | -10.0 deg (exact) |
| T2 | ~0 | ~0 | ~0 |
| T3 | ~0 | ~0 | ~0 |

State Readback:
| Turbine | yaw_seq | ind_seq |
|---------|---------|---------|
| T1 | 1 | 0 |
| T2 | 0 | 0 |
| T3 | 0 | 0 |

Result: PASS - yaw channel works independently, exact -10 deg deflection

---

## 7. Multi-Turbine Isolation Test

Command: T1 induction a=0.20, T2 yaw +10deg, T3 no command
Mode: legacy (bridge_enable=0)

Farm Power: baseline 4366 kW -> test 4262 kW (-2.4%)

Per-Turbine:
| Turbine | Type | Baseline (kW) | Test (kW) | Change |
|---------|------|---------------|-----------|--------|
| T1 | induction a=0.20 | 1633 | 1495 | -8.5% |
| T2 | yaw +10deg | 1450 | 1330 | -8.3% |
| T3 | no command | 1283 | 1437 | +12.0% |

State Readback:
| Turbine | yaw_seq | ind_seq | Note |
|---------|---------|---------|------|
| T1 | 0 | 1 | induction only |
| T2 | 1 | 0 | yaw only |
| T3 | 0 | 0 | no command, no cross-talk |

Result: PASS - isolation verified, no cross-talk

---

## 8. Bridge Mode Test (bridge_enable=1)

Command: T1 induction a=0.20, T2 yaw +10deg, T3 no command
Mode: bridge (bridge_enable=1, induction_mode=1)

Farm Power: 4262 kW (same as legacy)

Per-Turbine Comparison:
| Turbine | Legacy (kW) | Bridge (kW) | Deviation |
|---------|-------------|-------------|-----------|
| T1 | 1495 | 1496 | 0.07% |
| T2 | 1330 | 1329 | 0.08% |
| T3 | 1437 | 1437 | 0.00% |

Result: PASS - bridge mode matches legacy (deviation < 0.1%)

---

## 9. Test Summary

| Test | Status | Farm Power | Key Metric |
|------|--------|------------|------------|
| Baseline | PASS | 4366 kW | consistent with history |
| Induction a=0.20 | PASS | 4232 kW | T1 -8.5% derating |
| Yaw +10deg | PASS | 4367 kW | T1 -10.0 deg exact |
| Multi-turbine isolation | PASS | 4262 kW | no cross-talk |
| Bridge mode | PASS | 4262 kW | <0.1% vs legacy |

---

## 10. Key Fixes During Testing

1. WIN-05 ZMQ validation: changed from checking turbines[0].source_time_s to checking ANY valid command in frame
2. Python protocol sync: added yaw_absolute_heading_rad and yaw_absolute_valid to ctypes struct
3. Python source_time_s: auto-set in set_yaw_cmd and set_induction_cmd

---

## 11. Conclusion

All functional tests PASS:
- Induction channel: independent, -8.5% derating for a=0.20
- Yaw channel: independent, exact -10 deg deflection
- Multi-turbine isolation: no cross-talk between turbines
- Bridge mode: consistent with legacy mode (<0.1% deviation)
