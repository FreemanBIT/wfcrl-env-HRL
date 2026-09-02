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
   LOGICAL,            SAVE :: init_done_ = .FALSE.
   LOGICAL,            SAVE :: api_ok_ = .FALSE.

   ! ---- 公开接口 ----
   PUBLIC :: FCR_Init, FCR_Step, FCR_PublishState, FCR_Shutdown
   PUBLIC :: FCR_IsExternalEnabled, FCR_IsYawExternal, FCR_GetYawTargetHeadingDeg, &
             FCR_GetYawSeqApplied, FCR_GetPowerRatio, FCR_GetMinPitchDeg, &
             FCR_GetCommandStatusFlags, FCR_Setpoint

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
      st%nacelle_heading_rad = LocalVar%NacHeading
      st%nacelle_vane_rad = LocalVar%NacVane
      st%hub_wind_speed_mps = LocalVar%HorWindV
      st%blade_pitch_rad(1) = LocalVar%BlPitch(1)
      st%blade_pitch_rad(2) = LocalVar%BlPitch(2)
      st%blade_pitch_rad(3) = LocalVar%BlPitch(3)
      st%blade_root_moop_nm(1) = LocalVar%rootMOOP(1)
      st%blade_root_moop_nm(2) = LocalVar%rootMOOP(2)
      st%blade_root_moop_nm(3) = LocalVar%rootMOOP(3)
      st%tower_top_fa_acc_mps2 = LocalVar%FA_Acc
      st%nacelle_imu_fa_acc_mps2 = LocalVar%NacIMU_FA_Acc
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
      st%controller_status_flags = 0
      rc = fcr_rosco_publish_state_fc(turbine_id, st)
   END SUBROUTINE FCR_PublishState

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
   END SUBROUTINE FCR_Shutdown

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