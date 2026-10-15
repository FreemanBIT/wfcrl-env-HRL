! =============================================================================
! FarmControlInterface.f90 — ROSCO 侧的场控外部接口（Phase 2 无扰接入）
!
! 职责：
!   1. 每台机 DISCON 首次调用时初始化 runtime（同进程 DLL 单例）；
!   2. 每 10 ms：fcr_step() — 驱动 runtime 高速步（命令 refresh + 锁存 +
!      setpoint 生成），并把最新 setpoint 拷贝到本地 SAVE 副本；
!   3. 每 10 ms：fcr_publish_state() — 把 LocalVar/avrSWAP 快速状态发布到
!      runtime（共享内存，无网络/文件 I/O）；
!   4. 提供控制计算所需的查询（外部偏航目标/使能等）。
!
! 无扰原则：外部控制默认 disabled（farm_control_enable=0），
! 查询函数返回未启用状态时，ROSCO 原控制路径完全不变。
! =============================================================================
MODULE FarmControlInterface

   USE, INTRINSIC :: ISO_C_BINDING
   USE FarmControlCBindings
   IMPLICIT NONE

   PRIVATE

   INTEGER, PARAMETER :: FCR_MAX_TURBINES_LOCAL = 64

   ! 每台机 setpoint 本地副本（SAVE，DLL 进程内存）
   TYPE(FcrSetpointC), SAVE, TARGET :: setpoint_(FCR_MAX_TURBINES_LOCAL)
   INTEGER,            SAVE :: n_turbines_ = 0
   INTEGER,            SAVE :: current_turbine_id_ = 0
   LOGICAL,            SAVE :: init_done_ = .FALSE.
   LOGICAL,            SAVE :: api_ok_ = .FALSE.
   ! WIN-07: per-turbine nominal parameter arrays (multi-instance safe)
   REAL(8), SAVE :: VS_RtPwr_nom(FCR_MAX_TURBINES_LOCAL) = 0.0_8
   REAL(8), SAVE :: VS_MaxTq_nom(FCR_MAX_TURBINES_LOCAL) = 0.0_8
   REAL(8), SAVE :: VS_RefSpd_nom(FCR_MAX_TURBINES_LOCAL) = 0.0_8
   LOGICAL, SAVE :: nom_ok(FCR_MAX_TURBINES_LOCAL) = .FALSE.

   ! ---- 公开接口 ----
   PUBLIC :: FCR_Init, FCR_Step, FCR_PublishState, FCR_Shutdown
   PUBLIC :: FCR_IsExternalEnabled, FCR_IsYawExternal, FCR_GetYawTargetHeadingDeg, &
             FCR_GetYawSeqApplied, FCR_GetPowerRatio, FCR_GetMinPitchDeg, &
             FCR_GetCommandStatusFlags, FCR_Setpoint, FCR_CurrentTurbineId, FCR_PublishExtra, FCR_ProviderInit, FCR_ApplyReferences

CONTAINS

   ! ------------------------------------------------------------------
   ! FCR_Init — 初始化 runtime（每机调用一次；内部保证单例）
   ! ------------------------------------------------------------------
   SUBROUTINE FCR_Init(turbine_id, dt_high_s)
      INTEGER, INTENT(IN) :: turbine_id
      REAL(C_DOUBLE), INTENT(IN), OPTIONAL :: dt_high_s
      REAL(C_DOUBLE) :: dt
      INTEGER(C_INT) :: rc

      IF (init_done_) RETURN
      dt = 0.01_C_DOUBLE
      IF (PRESENT(dt_high_s)) dt = dt_high_s
      rc = fcr_rosco_api_init(FCR_MAX_TURBINES_LOCAL, dt)
      api_ok_ = (rc == 0)

      IF (api_ok_) THEN
         rc = fcr_rosco_api_register_turbine(turbine_id)
         IF (rc /= 0) api_ok_ = .FALSE.
      ENDIF
      n_turbines_ = MAX(n_turbines_, turbine_id)
      init_done_ = .TRUE.

   END SUBROUTINE FCR_Init

   ! ------------------------------------------------------------------
   ! FCR_Step — 每 ~10 ms 驱动 runtime 高速步并刷新本地 setpoint 副本
   ! ------------------------------------------------------------------
   SUBROUTINE FCR_Step(turbine_id, sim_time_s)
      INTEGER, INTENT(IN) :: turbine_id
      REAL(8), INTENT(IN) :: sim_time_s
      INTEGER(C_INT) :: rc
      IF (.NOT. init_done_) CALL FCR_Init(turbine_id)
      IF (.NOT. api_ok_) RETURN
      current_turbine_id_ = turbine_id
      rc = fcr_rosco_step_high(turbine_id, sim_time_s)
      IF (rc == 0 .AND. turbine_id >= 1 .AND. turbine_id <= n_turbines_) THEN
         rc = fcr_rosco_read_setpoint_fc(turbine_id, setpoint_(turbine_id))
      ENDIF
   END SUBROUTINE FCR_Step

   ! ------------------------------------------------------------------
   ! FCR_PublishState — 每 ~10 ms 发布 ROSCO 快速状态
   ! ------------------------------------------------------------------
   SUBROUTINE FCR_PublishState(turbine_id, LocalVar, avrSWAP)
      USE ROSCO_Types, ONLY : LocalVariables
      TYPE(LocalVariables), INTENT(IN) :: LocalVar
      REAL(C_FLOAT), INTENT(IN) :: avrSWAP(*)   ! Bladed DLL swap array（单精度）
      INTEGER, INTENT(IN) :: turbine_id
      TYPE(FcrFastStateC) :: st
      INTEGER(C_INT) :: rc
      IF (.NOT. api_ok_) RETURN
      st%turbine_id = turbine_id
      st%valid = 1
      st%source_time_s = LocalVar%Time
      st%source_seq = 0
      st%mech_gen_power_w = LocalVar%VS_MechGenPwr
      st%gen_power_w = LocalVar%VS_GenPwr
      st%gen_speed_rad_s = LocalVar%GenSpeed
      st%rotor_speed_rad_s = LocalVar%RotSpeed
      st%gen_torque_meas_nm = LocalVar%GenTqMeas
      ! P3 单位修复（2026-09-21）：LocalVar%NacHeading/NacVane 在 ROSCO 内为 [deg]
      ! （ReadSetParameters: NacHeading=avrSWAP(37)*R2D、NacVane=avrSWAP(24)*R2D），
      ! 本字段名义为 rad。此前直传度数值（deg-as-rad）导致：状态帧 T2/T3 显示
      ! 20610° 垃圾值；多命令重锁存时 C 侧 fcr_compute_yaw_target 把度当弧度
      ! 算出垃圾目标。×D2R 修正；首次锁存（heading≈0）行为不变。
      st%nacelle_heading_rad = LocalVar%NacHeading * (3.141592653589793_8 / 180.0_8)
      st%nacelle_vane_rad = LocalVar%NacVane * (3.141592653589793_8 / 180.0_8)
      st%hub_wind_speed_mps = LocalVar%HorWindV
      st%blade_pitch_rad(1) = LocalVar%BlPitch(1)
      st%blade_pitch_rad(2) = LocalVar%BlPitch(2)
      st%blade_pitch_rad(3) = LocalVar%BlPitch(3)
      st%blade_root_moop_nm(1) = LocalVar%rootMOOP(1)
      st%blade_root_moop_nm(2) = LocalVar%rootMOOP(2)
      st%blade_root_moop_nm(3) = LocalVar%rootMOOP(3)
      st%tower_top_fa_acc_mps2 = LocalVar%FA_AccF
      st%nacelle_imu_fa_acc_mps2 = LocalVar%NacIMU_FA_RAcc
      st%rotor_azimuth_rad = LocalVar%Azimuth
      ! 最终命令（avrSWAP 输出槽，控制计算完成后）：
      st%gen_torque_cmd_nm = avrSWAP(47)
      st%yaw_rate_cmd_rad_s = avrSWAP(48)
      st%pitch_cmd_rad(1) = avrSWAP(42)
      st%pitch_cmd_rad(2) = avrSWAP(43)
      st%pitch_cmd_rad(3) = avrSWAP(44)
      st%yaw_target_heading_rad = 0.0_C_DOUBLE
      st%yaw_seq_applied = 0
      st%induction_seq_applied = 0
      IF (turbine_id >= 1 .AND. turbine_id <= n_turbines_) THEN
         IF (setpoint_(turbine_id)%yaw_enable /= 0) THEN
            st%yaw_target_heading_rad = setpoint_(turbine_id)%yaw_target_heading_rad
            st%yaw_seq_applied = setpoint_(turbine_id)%yaw_seq
         END IF
         IF (setpoint_(turbine_id)%induction_enable /= 0) THEN
            st%induction_seq_applied = setpoint_(turbine_id)%induction_seq
         END IF
      END IF
      rc = fcr_rosco_publish_state_fc(turbine_id, st)
   END SUBROUTINE FCR_PublishState

   ! ------------------------------------------------------------------
   ! FCR_PublishExtra — 每 ~10 ms 发布 OpenFAST extra 状态（Offline Provider 入口）
   ! ------------------------------------------------------------------
   SUBROUTINE FCR_PublishExtra(turbine_id, sim_time_s, avrSWAP)
      INTEGER, INTENT(IN) :: turbine_id
      REAL(8), INTENT(IN) :: sim_time_s
      REAL(C_FLOAT), INTENT(IN) :: avrSWAP(*)   ! Bladed DLL swap array（单精度）
      INTEGER(C_INT) :: rc
      IF (.NOT. api_ok_) RETURN
      rc = fcr_offline_provider_publish_fast(turbine_id, sim_time_s, avrSWAP)
   END SUBROUTINE FCR_PublishExtra

   ! 初始化 Offline Provider（读取 env 配置；幂等）
   SUBROUTINE FCR_ProviderInit()
      INTEGER(C_INT) :: rc
      rc = fcr_offline_provider_init()
   END SUBROUTINE FCR_ProviderInit

   ! ------------------------------------------------------------------
   ! FCR_ApplyReferences — 把 induction setpoint 注入 ROSCO 执行参考
   !（Phase 6）：每 ~10 ms 调用；恢复标称后按 power_ratio/torque_limit_ratio/
   ! speed_ref_ratio/min_pitch_rad 修改 CntrPar；disabled 时完全回退。
   ! 无网络/文件 I/O（红线合规）。
   ! ------------------------------------------------------------------
   SUBROUTINE FCR_ApplyReferences(CntrPar)
      USE ROSCO_Types, ONLY : ControlParameters
      TYPE(ControlParameters), INTENT(INOUT) :: CntrPar
      TYPE(FcrSetpointC), POINTER :: sp
      INTEGER :: tid
      ! WIN-03: independent channel active flags
      LOGICAL :: pitch_active, speed_active, torque_active, any_active
      IF (.NOT. api_ok_) RETURN
      tid = current_turbine_id_
      IF (tid < 1 .OR. tid > n_turbines_) RETURN
      ! WIN-07: per-turbine nominal parameter capture
      IF (.NOT. nom_ok(tid)) THEN
         VS_RtPwr_nom(tid)  = CntrPar%VS_RtPwr
         VS_MaxTq_nom(tid)  = CntrPar%VS_MaxTq
         VS_RefSpd_nom(tid) = CntrPar%VS_RefSpd
         nom_ok(tid) = .TRUE.
      END IF
      ! reset to nominal every step (prevent accumulation)
      CntrPar%VS_RtPwr  = VS_RtPwr_nom(tid)
      CntrPar%VS_MaxTq  = VS_MaxTq_nom(tid)
      CntrPar%VS_RefSpd = VS_RefSpd_nom(tid)
      sp => FCR_Setpoint(tid)
      IF (.NOT. ASSOCIATED(sp)) RETURN

      ! WIN-02+WIN-03: independent channel evaluation
      ! pitch channel: power_ratio < 1 means derating via PRC_R_Pitch
      pitch_active  = sp%induction_enable /= 0 .AND. &
                      sp%power_ratio < 1.0_8 - 1.0e-6_8
      ! speed channel: speed_ref_ratio differs from 1
      speed_active  = sp%induction_enable /= 0 .AND. &
                      ABS(sp%speed_ref_ratio - 1.0_8) > 1.0e-6_8 .AND. &
                      sp%speed_ref_ratio > 0.0_8
      ! torque channel: torque_limit_ratio differs from 1
      torque_active = sp%induction_enable /= 0 .AND. &
                      ABS(sp%torque_limit_ratio - 1.0_8) > 1.0e-6_8 .AND. &
                      sp%torque_limit_ratio > 0.0_8
      any_active = pitch_active .OR. speed_active .OR. torque_active .OR. &
                   (sp%induction_enable /= 0 .AND. sp%min_pitch_rad > 0.001_8)

      IF (.NOT. any_active) THEN
         ! disabled or no-op: full ROSCO fallback
         CntrPar%VS_ConstPower = 0
         CntrPar%PS_Mode = 0
         CntrPar%PRC_Mode = 0
         CntrPar%PRC_R_Speed = 1.0_8
         CntrPar%PRC_R_Torque = 1.0_8
         CntrPar%PRC_R_Pitch = 1.0_8
         RETURN
      END IF

      ! --- pitch channel (PRC derating via PRC_R_Pitch -> PRC_Min_Pitch) ---
      IF (pitch_active) THEN
         CntrPar%PRC_Mode = 2
         CntrPar%PRC_Comm = 0
         CntrPar%PRC_R_Pitch = sp%power_ratio
         CntrPar%VS_ConstPower = 1
         CntrPar%VS_RtPwr = VS_RtPwr_nom(tid) * sp%power_ratio
         ! WIN-03: PS_Mode=1 required for PitchSaturation to apply PRC_Min_Pitch
         CntrPar%PS_Mode = 1
      END IF

      ! --- speed channel (PRC_R_Speed -> WSE_TSR scaling) ---
      IF (speed_active) THEN
         CntrPar%PRC_Mode = 2
         CntrPar%PRC_Comm = 0
         CntrPar%PRC_R_Speed = sp%speed_ref_ratio
      END IF

      ! --- torque channel (PRC_R_Torque -> VS_MaxTq scaling) ---
      IF (torque_active) THEN
         CntrPar%PRC_Mode = 2
         CntrPar%PRC_Comm = 0
         CntrPar%PRC_R_Torque = sp%torque_limit_ratio
         CntrPar%VS_MaxTq = VS_MaxTq_nom(tid) * sp%torque_limit_ratio
      END IF

      ! --- WIN-02: min_pitch channel independent of power_ratio ---
      ! Mode2 Ct feedforward sets min_pitch_rad even when power_ratio=1.
      ! This must NOT be swallowed by the greedy sentinel.
      IF (sp%induction_enable /= 0 .AND. sp%min_pitch_rad > 0.001_8) THEN
         CntrPar%PS_Mode = 1
         CntrPar%PS_BldPitchMin_N = 1
         IF (.NOT. ALLOCATED(CntrPar%PS_BldPitchMin)) &
            ALLOCATE(CntrPar%PS_BldPitchMin(1))
         IF (.NOT. ALLOCATED(CntrPar%PS_WindSpeeds)) &
            ALLOCATE(CntrPar%PS_WindSpeeds(1))
         CntrPar%PS_WindSpeeds(1)  = 0.0
         CntrPar%PS_BldPitchMin(1) = sp%min_pitch_rad
      END IF
   END SUBROUTINE FCR_ApplyReferences



   ! ------------------------------------------------------------------
   ! 查询函数（ROSCO 控制计算内调用；全部基于本地 setpoint 副本）
   ! ------------------------------------------------------------------
   LOGICAL FUNCTION FCR_IsExternalEnabled(turbine_id) RESULT(r)
      INTEGER, INTENT(IN) :: turbine_id
      r = .FALSE.
      IF (turbine_id >= 1 .AND. turbine_id <= n_turbines_) THEN
         r = (setpoint_(turbine_id)%farm_control_enable /= 0)
      ENDIF
   END FUNCTION FCR_IsExternalEnabled

   LOGICAL FUNCTION FCR_IsYawExternal(turbine_id) RESULT(r)
      INTEGER, INTENT(IN) :: turbine_id
      r = .FALSE.
      IF (turbine_id >= 1 .AND. turbine_id <= n_turbines_) THEN
         r = (setpoint_(turbine_id)%yaw_enable /= 0)
      ENDIF
   END FUNCTION FCR_IsYawExternal

   ! 外部偏航目标（度，OpenFAST 内部约定 CCW+；由 runtime 从 CW+ 增量换算并锁存）
   REAL(8) FUNCTION FCR_GetYawTargetHeadingDeg(turbine_id) RESULT(r)
      INTEGER, INTENT(IN) :: turbine_id
      r = 0.0_8
      IF (turbine_id >= 1 .AND. turbine_id <= n_turbines_) THEN
         r = setpoint_(turbine_id)%yaw_target_heading_rad * 180.0_8 / 3.141592653589793_8
      ENDIF
   END FUNCTION FCR_GetYawTargetHeadingDeg

   INTEGER(8) FUNCTION FCR_GetYawSeqApplied(turbine_id) RESULT(r)
      INTEGER, INTENT(IN) :: turbine_id
      r = 0_8
      IF (turbine_id >= 1 .AND. turbine_id <= n_turbines_) THEN
         r = setpoint_(turbine_id)%yaw_seq
      ENDIF
   END FUNCTION FCR_GetYawSeqApplied

   REAL(8) FUNCTION FCR_GetPowerRatio(turbine_id) RESULT(r)
      INTEGER, INTENT(IN) :: turbine_id
      r = 1.0_8
      IF (turbine_id >= 1 .AND. turbine_id <= n_turbines_) THEN
         r = setpoint_(turbine_id)%power_ratio
      ENDIF
   END FUNCTION FCR_GetPowerRatio

   REAL(8) FUNCTION FCR_GetMinPitchDeg(turbine_id) RESULT(r)
      INTEGER, INTENT(IN) :: turbine_id
      r = 0.0_8
      IF (turbine_id >= 1 .AND. turbine_id <= n_turbines_) THEN
         r = setpoint_(turbine_id)%min_pitch_rad * 180.0_8 / 3.141592653589793_8
      ENDIF
   END FUNCTION FCR_GetMinPitchDeg

   INTEGER FUNCTION FCR_GetCommandStatusFlags(turbine_id) RESULT(r)
      INTEGER, INTENT(IN) :: turbine_id
      r = 0
      IF (turbine_id >= 1 .AND. turbine_id <= n_turbines_) THEN
         r = setpoint_(turbine_id)%command_status_flags
      ENDIF
   END FUNCTION FCR_GetCommandStatusFlags

   SUBROUTINE FCR_Shutdown()
      INTEGER(C_INT) :: rc
      rc = fcr_rosco_api_shutdown()
      init_done_ = .FALSE.
      api_ok_ = .FALSE.
      ! WIN-07: reset per-turbine nominal arrays
      nom_ok = .FALSE.
      VS_RtPwr_nom = 0.0_8
      VS_MaxTq_nom = 0.0_8
      VS_RefSpd_nom = 0.0_8
   END SUBROUTINE FCR_Shutdown

   ! 当前（最近一次 FCR_Step 的）机组号 —— 供 ROSCO 控制子程序查询自身
   INTEGER FUNCTION FCR_CurrentTurbineId() RESULT(r)
      r = current_turbine_id_
   END FUNCTION FCR_CurrentTurbineId

   ! 供其他模块直接读取 setpoint 副本（只读约定）
   FUNCTION FCR_Setpoint(turbine_id) RESULT(sp)
      TYPE(FcrSetpointC), POINTER :: sp
      INTEGER, INTENT(IN) :: turbine_id
      sp => NULL()
      IF (turbine_id >= 1 .AND. turbine_id <= n_turbines_) THEN
         sp => setpoint_(turbine_id)
      ENDIF
   END FUNCTION FCR_Setpoint

END MODULE FarmControlInterface