! =============================================================================
! FarmControlCBindings.f90 — runtime ↔ ROSCO 的 C/Fortran ABI 绑定（ICD v1.5）
!
! 仅包含 bind(C) 接口声明；Fortran 语义层见 FarmControlInterface.f90.
! 对应 C 实现：farm_control_runtime/src/rosco_api.c
! =============================================================================
MODULE FarmControlCBindings

   USE, INTRINSIC :: ISO_C_BINDING
   IMPLICIT NONE

   ! C 结构镜像（字段顺序/类型与 fcr_rosco_api.h 完全一致；ABI 尺寸有测试锁定）
   ! FcrRoscoExternalSetpoint (ICD v1.5 S0801-S0807 + S1101 保留接口)
   TYPE, BIND(C) :: FcrSetpointC
      INTEGER(C_INT32_T)   :: valid
      INTEGER(C_INT32_T)   :: farm_control_enable         ! S0104/S0704
      INTEGER(C_INT32_T)   :: external_yaw_enable          ! S0801
      INTEGER(C_INT64_T)   :: yaw_seq                      ! S0708
      REAL(C_DOUBLE)       :: yaw_target_heading_rad       ! S0802
      INTEGER(C_INT32_T)   :: induction_enable             ! S0107
      INTEGER(C_INT64_T)   :: induction_seq                ! S0706
      REAL(C_DOUBLE)       :: ct_ref                       ! S0904
      REAL(C_DOUBLE)       :: r_pitch                      ! S0803
      REAL(C_DOUBLE)       :: r_speed                      ! S0804
      REAL(C_DOUBLE)       :: r_torque                     ! S0805
      REAL(C_DOUBLE)       :: min_pitch_ref_rad            ! S0806
      REAL(C_DOUBLE)       :: collective_pitch_offset_rad  ! S0807
      INTEGER(C_INT32_T)   :: command_status_flags
      REAL(C_DOUBLE)       :: ct_tracking_error            ! S1101（仅保留接口）
   END TYPE FcrSetpointC

   ! FcrRoscoRawState (ICD v1.5 S0301-S0303 + 扩展)
   TYPE, BIND(C) :: FcrFastStateC
      INTEGER(C_INT32_T)   :: turbine_id
      INTEGER(C_INT32_T)   :: valid
      REAL(C_DOUBLE)       :: source_time_s
      INTEGER(C_INT64_T)   :: source_seq
      INTEGER(C_INT32_T)   :: rosco_controller_status_flags  ! S0301
      REAL(C_DOUBLE)       :: generator_torque_command_nm    ! S0302
      REAL(C_DOUBLE)       :: pitch_command_rad(3)           ! S0303
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
      REAL(C_DOUBLE)       :: yaw_rate_cmd_rad_s
      REAL(C_DOUBLE)       :: yaw_target_heading_rad
      INTEGER(C_INT64_T)   :: yaw_seq_applied              ! S0902
      INTEGER(C_INT64_T)   :: induction_seq_applied        ! S0901
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

      ! 诊断
      REAL(C_DOUBLE) FUNCTION fcr_rosco_debug_yaw_target(turbine_id) BIND(C, NAME='fcr_rosco_debug_yaw_target')
         IMPORT :: C_INT, C_DOUBLE
         INTEGER(C_INT), VALUE :: turbine_id
      END FUNCTION fcr_rosco_debug_yaw_target

      INTEGER(C_INT) FUNCTION fcr_zmq_transport_status() BIND(C, NAME='fcr_zmq_transport_status')
         IMPORT :: C_INT
      END FUNCTION fcr_zmq_transport_status

      ! Offline Provider
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
