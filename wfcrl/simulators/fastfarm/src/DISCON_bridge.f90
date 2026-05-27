! =============================================================================
! DISCON_bridge.f90 — 文件 I/O 桥接 DISCON DLL
! 编译: gfortran -shared -static -o DISCON_WT1.dll DISCON_bridge.f90
! =============================================================================
SUBROUTINE DISCON(avrSWap, aviFail, accINFILE, avrOutData, avrData) BIND (C, NAME='DISCON')
  USE, INTRINSIC :: ISO_C_Binding
  IMPLICIT NONE
  REAL(C_FLOAT),   INTENT(INOUT) :: avrSWap(*)
  INTEGER(C_INT),  INTENT(INOUT) :: aviFail
  CHARACTER(KIND=C_CHAR), INTENT(IN) :: accINFILE(*)
  REAL(C_FLOAT),   INTENT(INOUT) :: avrOutData(*)
  REAL(C_FLOAT),   INTENT(INOUT) :: avrData(*)

  INTEGER(4) :: turbine_id, io_status, i, read_step, c_len
  REAL(4)    :: cmd_yaw, cmd_pitch, cmd_torque
  REAL(4)    :: current_time, gen_pwr, gen_spd, gen_tq, rot_spd, blade1_pitch, nac_yaw
  REAL(4)    :: wind_x, root_mip1, root_moop1, root_mzb1
  CHARACTER(256) :: line, token, dll_in_file
  CHARACTER(32)  :: tstr
  LOGICAL        :: fexist, found
  INTEGER(4), SAVE :: my_id = 0, applied_step = -1, init_done = 0
  REAL(4),    SAVE :: prev_yaw = 0.0, prev_pitch = 0.0, prev_torque = 0.0
  INTEGER(4), SAVE :: call_count = 0

  ! 每次调用都写标记文件（调试：确认 DLL 被调用）
  call_count = call_count + 1
  OPEN(97, FILE='_discon_called.txt', STATUS='REPLACE')
  WRITE(97,'(A,I0,A,I0)') 'call=',call_count,' turbine=',my_id
  CLOSE(97)

  ! 初始化: 通过 accINFILE 读取 turbine ID
  ! accINFILE 由 ServoDyn 的 DLL_InFile 传入（如 "DISCON_T1.IN"）
  IF (init_done == 0) THEN
    ! 将 C 字符数组转换为 Fortran 字符串
    dll_in_file = ''
    DO c_len = 1, 255
      IF (accINFILE(c_len) == C_NULL_CHAR) EXIT
      dll_in_file(c_len:c_len) = accINFILE(c_len)
    END DO
    INQUIRE(FILE=TRIM(dll_in_file), EXIST=fexist)
    IF (fexist) THEN
      OPEN(93, FILE=TRIM(dll_in_file), STATUS='OLD')
      READ(93,*,IOSTAT=io_status) my_id; CLOSE(93)
      IF (io_status /= 0) my_id = 1
    ELSE
      my_id = 1
    END IF
    init_done = 1
  END IF
  turbine_id = my_id

  ! 确保返回成功
  aviFail = 0

  ! --- 提取测量值 (NREL 5MW avrData 索引) ---
  current_time = avrSWap(2)
  gen_pwr  = avrSWap(15)  ! GenPwr (W)
  gen_spd  = avrSWap(20)  ! GenSpeed (rad/s)
  gen_tq   = avrSWap(23)  ! GenTqMeas (Nm)
  rot_spd  = avrSWap(21)  ! RotSpeed (rad/s)
  blade1_pitch = avrSWap(4)  ! BlPitch(1) (rad)
  nac_yaw  = avrSWap(37)  ! NacHeading (rad)
  wind_x   = avrSWap(27)  ! HorWindV (m/s)
  root_mip1  = avrSWap(30) ! RootMIP (kNm)
  root_moop1 = avrSWap(31) ! RootMOoP (kNm)
  root_mzb1  = avrSWap(32) ! RootMzb (kNm)

  ! --- 设置默认控制器输出 ---
  ! Correct OpenFAST Bladed DLL output indices:
  ! avrSWap(42-44) = blade 1-3 pitch demand (rad)
  ! avrSWap(45)    = collective pitch demand (rad)
  ! avrSWap(47)    = generator torque demand (Nm)
  ! avrSWap(48)    = yaw rate demand (rad/s)
  avrSWap(42) = 0.0       ! pitch cmd blade 1 (rad)
  avrSWap(43) = 0.0       ! pitch cmd blade 2 (rad)
  avrSWap(44) = 0.0       ! pitch cmd blade 3 (rad)
  avrSWap(45) = 0.0       ! collective pitch (rad)
  avrSWap(47) = 43093.55  ! generator torque (Nm)
  avrSWap(48) = 0.0       ! yaw rate (rad/s)

  ! --- 写入测量文件 ---
  WRITE(tstr, '(I0)') turbine_id
  OPEN(94, FILE='measurements_T'//TRIM(tstr)//'.txt', STATUS='REPLACE')
  ! 使用显式格式写入，确保 "step=N" 等为连续字符（无分隔空格）
  WRITE(94,'(A,I0,A,F0.4,A,F0.2,A,F0.4,A,F0.4,A,F0.2)') &
    'step=',applied_step,' t=',current_time,' genpwr=',gen_pwr/1000.0, &
    ' genspd=',gen_spd*9.5493,' gentq=',gen_tq,' rotspd=',rot_spd*9.5493
  WRITE(94,'(A,F0.6)') ' wind_x=',wind_x
  WRITE(94,'(A,F0.6,A,F0.6)') ' blpitch=',blade1_pitch*57.29578,' nacyaw=',nac_yaw*57.29578
  WRITE(94,'(A,F0.2,A,F0.2,A,F0.2)') ' mip1=',root_mip1,' moop1=',root_moop1,' mzb1=',root_mzb1
  CLOSE(94)

  ! --- 读取控制文件 ---
  INQUIRE(FILE='controls.txt', EXIST=fexist)
  IF (.NOT. fexist) RETURN
  OPEN(95, FILE='controls.txt', STATUS='OLD', ACTION='READ')
  READ(95, '(A)', IOSTAT=io_status) line
  IF (io_status /= 0) THEN; CLOSE(95); RETURN; END IF

  i = INDEX(line, '='); IF (i == 0) THEN; CLOSE(95); RETURN; END IF
  READ(line(i+1:), *, IOSTAT=io_status) read_step
  IF (read_step <= applied_step) THEN; CLOSE(95); RETURN; END IF

  ! 查找本风机控制命令
  found = .FALSE.
  DO
    READ(95, '(A)', IOSTAT=io_status) line
    IF (io_status /= 0 .OR. TRIM(line) == 'END') EXIT
    i = INDEX(line, ' '); IF (i == 0) CYCLE
    token = line(1:i-1)
    IF (token(1:1) == 'T' .OR. token(1:1) == 't') THEN
      READ(token(2:), *, IOSTAT=io_status) turbine_id
      IF (turbine_id == my_id) THEN
        cmd_yaw = prev_yaw; cmd_pitch = prev_pitch; cmd_torque = prev_torque
        i = INDEX(line, 'yaw=');    IF (i > 0) READ(line(i+4:), *) cmd_yaw
        i = INDEX(line, 'pitch=');  IF (i > 0) READ(line(i+6:), *) cmd_pitch
        i = INDEX(line, 'torque='); IF (i > 0) READ(line(i+7:), *) cmd_torque
        prev_yaw = cmd_yaw; prev_pitch = cmd_pitch; prev_torque = cmd_torque
        found = .TRUE.; EXIT
      END IF
    END IF
  END DO
  CLOSE(95)
  IF (.NOT. found) RETURN

  ! --- 应用控制 (通过 avrSWap 输出到 ServoDyn) ---
  applied_step = read_step
  avrSWap(42) = cmd_pitch * 0.0174533   ! pitch cmd blade 1 (rad)
  avrSWap(43) = cmd_pitch * 0.0174533   ! blade 2
  avrSWap(44) = cmd_pitch * 0.0174533   ! blade 3
  avrSWap(45) = cmd_pitch * 0.0174533   ! collective pitch (rad)
  IF (cmd_torque > 0.01) THEN
    avrSWap(47) = cmd_torque             ! generator torque (Nm)
  END IF
  ! Apply yaw via yaw rate (avrSWap(48), rad/s)
  ! nac_yaw = current yaw from avrSWap(37) in rad, cmd_yaw in degrees
  avrSWap(48) = (cmd_yaw * 0.0174533 - nac_yaw) * 0.2  ! proportional rate
  prev_yaw = cmd_yaw  ! track yaw for next call
  prev_pitch = cmd_pitch
  prev_torque = cmd_torque

END SUBROUTINE DISCON
