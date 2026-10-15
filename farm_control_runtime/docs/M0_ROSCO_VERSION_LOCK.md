# M0: ROSCO Version Lock

## Windows Offline Build — Frozen Versions

| Component | Version / Commit | Compiler | Date |
|-----------|-----------------|----------|------|
| ROSCO | v2.10.6 (bundled in wfcrl) | gfortran 10.3.0 (TDM-GCC-64) | 2026-10-15 |
| FAST.Farm | v2.10.6 (wfcrl/simulators/fastfarm) | gfortran 10.3.0 | 2026-10-15 |
| farm_control_runtime | dev/farm-control-runtime branch | gcc 10.3.0 | 2026-10-15 |

## PRC Field Compatibility

The Fortran bridge (FarmControlInterface.f90) references these ROSCO v2.10.6 fields:
- `CntrPar%PRC_Mode` — PRC derating mode (set to 2 for external derating)
- `CntrPar%PRC_Comm` — PRC command type (set to 0 for Constant)
- `CntrPar%PRC_R_Speed` — speed reference ratio
- `CntrPar%PRC_R_Torque` — torque limit ratio  
- `CntrPar%PRC_R_Pitch` — pitch derating ratio
- `CntrPar%PS_Mode` — pitch saturation mode (must be >0 for PRC_Min_Pitch)
- `CntrPar%PS_BldPitchMin` — minimum blade pitch table
- `CntrPar%VS_ConstPower` — constant power mode flag
- `CntrPar%VS_RtPwr` — rated power
- `CntrPar%VS_MaxTq` — max torque
- `CntrPar%VS_RefSpd` — reference speed

## Verification

- DISCON_WT1.dll builds via `wfcrl/simulators/fastfarm/src/_compile.py`
- 300s regression baseline: 4366 kW (farm total)
- bridge_enable=1 isolation test: T1=1496 kW (-8.4% vs greedy 1633 kW)
- 57/57 unit tests pass

## Migration Path

If ROSCO version changes:
1. Verify all PRC_* fields exist in new ROSCO_Types.f90
2. Update this document with new version
3. Run 300s regression to confirm baseline
4. Run bridge_enable=1 isolation test
