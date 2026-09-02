! =============================================================================
! FarmControlCBindings.f90 — runtime ↔ ROSCO 的 C/Fortran ABI 绑定（Phase 2 冻结）
!
! 仅包含 bind(C) 接口声明；Fortran 语义层见 FarmControlInterface.f90.
! 对应 C 实现：farm_control_runtime/src/rosco_api.c
! =============================================================================
MODULE FarmControlCBindings

   USE, INTRINSIC :: ISO_C_BINDING
   IMPLICIT NONE

   ! C 结构镜像（字段顺序/类型与 fcr_rosco_api.h 完全一致；ABI 尺寸有测试锁定）
   ! FcrRoscoExternalSetpoint
   TYPE, BIND(C) :: FcrSetpointC
      INTEGER(C_INT32_T)   :: valid
      INTEGER(C_INT32_T)   :: farm_control_enable
      INTEGER(C_INT32_T)   :: yaw_enable
      INTEGER(C_INT64_T)   :: yaw_seq
      REAL(C_DOUBLE)       :: yaw_target_heading_rad
      INTEGER(C_INT32_T)   :: induction_enable
      INTEGER(C_INT64_T)   :: induction_seq
      REAL(C_DOUBLE)       :: ct_ref
      REAL(C_DOUBLE)       :: power_ratio
      REAL(C_DOUBLE)       :: speed_ref_ratio
      REAL(C_DOUBLE)       :: torque_limit_ratio
      REAL(C_DOUBLE)       :: min_pitch_rad
      INTEGER(C_INT32_T)   :: command_status_flags
   END TYPE FcrSetpointC

   ! FcrRoscoFastState
   TYPE, BIND(C) :: FcrFastStateC
      INTEGER(C_INT32_T)   :: turbine_id
      INTEGER(C_INT32_T)   :: valid
      REAL(C_DOUBLE)       :: source_time_s
      INTEGER(C_INT64_T)   :: source_seq
      REAL(C_DOUBLE)       :: mech_gen_power_w
      REAL(C_DOUBLE)       :: gen_power_w
      REAL(C_DOUBLE)       :: gen_speed_rad_s
      REAL(C_DOUBLE)       :: rotor_speed_rad_s
      REAL(C_DOUBLE)       :: gen_torque_meas_nm
      REAL(C_DOUBLE)       :: nacelle_heading_rad
      REAL(C_DOUBLE)       :: nacelle_vane_rad
      REAL(C_DOUBLE)       :: hub_wind_speed_mps
      REAL(C_DOUBLE)       :: blade_pitch_rad(3)
      REAL(C_DOUBLE)       :: blade_root_moop_nm(3)
      REAL(C_DOUBLE)       :: tower_top_fa_acc_mps2
      REAL(C_DOUBLE)       :: nacelle_imu_fa_acc_mps2
      REAL(C_DOUBLE)       :: rotor_azimuth_rad
      REAL(C_DOUBLE)       :: gen_torque_cmd_nm
      REAL(C_DOUBLE)       :: yaw_rate_cmd_rad_s
      REAL(C_DOUBLE)       :: pitch_cmd_rad(3)
      REAL(C_DOUBLE)       :: yaw_target_heading_rad
      INTEGER(C_INT64_T)   :: yaw_seq_applied
      INTEGER(C_INT64_T)   :: induction_seq_applied
      INTEGER(C_INT32_T)   :: controller_status_flags
   END TYPE FcrFastStateC

   INTERFACE
      ! 生命周期
      INTEGER(C_INT) FUNCTION fcr_rosco_api_init(n_turbines_max, dt_high_s) BIND(C, NAME='fcr_rosco_api_init')
         IMPORT :: C_INT, C_DOUBLE
         INTEGER(C_INT), VALUE :: n_turbines_max
         REAL(C_DOUBLE), VALUE :: dt_high_s
      END FUNCTION fcr_rosco_api_init

      INTEGER(C_INT) FUNCTION fcr_rosco_api_register_turbine(turbine_id) BIND(C, NAME='fcr_rosco_api_register_turbine')
         IMPORT :: C_INT
         INTEGER(C_INT), VALUE :: turbine_id
      END FUNCTION fcr_rosco_api_register_turbine

      INTEGER(C_INT) FUNCTION fcr_rosco_api_shutdown() BIND(C, NAME='fcr_rosco_api_shutdown')
         IMPORT :: C_INT
      END FUNCTION fcr_rosco_api_shutdown

      ! 10 ms 路径
      INTEGER(C_INT) FUNCTION fcr_rosco_step_high(turbine_id, sim_time_s) BIND(C, NAME='fcr_rosco_step_high')
         IMPORT :: C_INT, C_DOUBLE
         INTEGER(C_INT), VALUE :: turbine_id
         REAL(C_DOUBLE), VALUE :: sim_time_s
      END FUNCTION fcr_rosco_step_high

      INTEGER(C_INT) FUNCTION fcr_rosco_read_setpoint_fc(turbine_id, sp) BIND(C, NAME='fcr_rosco_read_setpoint')
         IMPORT :: C_INT, FcrSetpointC
         INTEGER(C_INT), VALUE :: turbine_id
         TYPE(FcrSetpointC)     :: sp
      END FUNCTION fcr_rosco_read_setpoint_fc

      INTEGER(C_INT) FUNCTION fcr_rosco_publish_state_fc(turbine_id, st) BIND(C, NAME='fcr_rosco_publish_state')
         IMPORT :: C_INT, FcrFastStateC
         INTEGER(C_INT), VALUE :: turbine_id
         TYPE(FcrFastStateC)    :: st
      END FUNCTION fcr_rosco_publish_state_fc

      ! ZMQ Transport 状态（Phase 7 诊断）
      INTEGER(C_INT) FUNCTION fcr_zmq_transport_status() BIND(C, NAME='fcr_zmq_transport_status')
         IMPORT :: C_INT
      END FUNCTION fcr_zmq_transport_status

      ! Offline Provider（Phase 4）：每 10 ms 发布 OpenFAST extra 状态
      INTEGER(C_INT) FUNCTION fcr_offline_provider_init() BIND(C, NAME='fcr_offline_provider_init')
         IMPORT :: C_INT
      END FUNCTION fcr_offline_provider_init

      INTEGER(C_INT) FUNCTION fcr_offline_provider_publish_fast(turbine_id, sim_time_s, avrSWAP) BIND(C, NAME='fcr_offline_provider_publish_fast')
         IMPORT :: C_INT, C_DOUBLE, C_FLOAT
         INTEGER(C_INT), VALUE :: turbine_id
         REAL(C_DOUBLE), VALUE :: sim_time_s
         REAL(C_FLOAT) :: avrSWAP(*)
      END FUNCTION fcr_offline_provider_publish_fast

      INTEGER(C_INT) FUNCTION fcr_rosco_step_low(sim_time_s) BIND(C, NAME='fcr_rosco_step_low')
         IMPORT :: C_INT, C_DOUBLE
         REAL(C_DOUBLE), VALUE :: sim_time_s
      END FUNCTION fcr_rosco_step_low
   END INTERFACE

END MODULE FarmControlCBindings