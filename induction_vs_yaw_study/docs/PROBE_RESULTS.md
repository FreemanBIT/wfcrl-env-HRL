# Stage 0 Probe Results

```

======================================================================
Stage 0 Probe — induction_vs_yaw_study
======================================================================

======================================================================
1. wfcrl.config.ControlInput
======================================================================
ControlInput fields: ['mode', 'yaw', 'pitch', 'power', 'min_pitch']
ControlInput mode factories: ['mode0_yaw', 'mode1_power', 'mode2_pitch', 'mode3_pitch_yaw', 'mode4_power_yaw']
  -> supports absolute power target: True
  -> supports min_pitch constraint : True
  VERDICT: 控制接口已原生支持功率目标 + 偏航 → 无需修改接口。

======================================================================
2. wfcrl.interface 接口
======================================================================
FlorisInterface: ['setup', 'reset', 'step', 'close']
ContinuousFastFarmInterface: ['setup', 'reset', 'step', 'start', 'wait_step', 'stop', 'close']

======================================================================
3. wfcrl.simul_config
======================================================================
FlorisConfig fields: ['case_name', 'num_turbines', 'xcoords', 'ycoords', 'dt', 'max_iter', 'wind', 't_init', 'turbine_type', 'output_channels', 'output_dir', 'turbine_library_path', 'wake_model', 'solver_grid_points', 'enable_active_wake_mixing', 'enable_secondary_steering', 'enable_yaw_added_recovery', 'enable_transverse_velocities']
FastFarmConfig fields: ['case_name', 'num_turbines', 'xcoords', 'ycoords', 'dt', 'max_iter', 'wind', 't_init', 'turbine_type', 'output_channels', 'output_dir', 'fastfarm_exe', 'template_dir', 'dt_low', 'fstf_overrides', 'wind_time_series_file']

======================================================================
4. FLORIS 可用性
======================================================================
floris version: 4.6.4
  nrel_5MW 3T greedy powers (MW): [1.75395446 0.35638489 0.3444147 ]
  FLORIS OK.

======================================================================
5. FAST.Farm 可执行文件
======================================================================
FAST_FARM_EXE: D:\HR_Project\wfcrl-env-HRL\wfcrl\simulators\fastfarm\bin\FAST.Farm_x64_OMP.exe
  exists: True

======================================================================
6. 入流 .bts 模板 (按 风速×TI 匹配)
======================================================================
FarmInputs dir: D:\HR_Project\wfcrl-env-HRL\wfcrl\simulators\fastfarm\inputs\template\FarmInputs
  U=6.0 TI=0.05: inflow_06ms_TI05.bts  OK
  U=6.0 TI=0.1: inflow_06ms_TI10.bts  OK
  U=6.0 TI=0.15: inflow_06ms_TI15.bts  OK
  U=8.0 TI=0.05: inflow_08ms_TI05.bts  OK
  U=8.0 TI=0.1: inflow_08ms_TI10.bts  OK
  U=8.0 TI=0.15: inflow_08ms_TI15.bts  OK
  U=10.0 TI=0.05: inflow_10ms_TI05.bts  OK
  U=10.0 TI=0.1: inflow_10ms_TI10.bts  OK
  U=10.0 TI=0.15: inflow_10ms_TI15.bts  OK
  全部 9 个 .bts 就位 → FAST.Farm 可在全网格的真实 TI 下回放。
```
