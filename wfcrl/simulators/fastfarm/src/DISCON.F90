! Copyright 2019 NREL

! Licensed under the Apache License, Version 2.0 (the "License"); you may not use
! this file except in compliance with the License. You may obtain a copy of the
! License at http://www.apache.org/licenses/LICENSE-2.0

! Unless required by applicable law or agreed to in writing, software distributed
! under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
! CONDITIONS OF ANY KIND, either express or implied. See the License for the
! specific language governing permissions and limitations under the License.
! -------------------------------------------------------------------------------------------

! High level run script

!=======================================================================
SUBROUTINE DISCON(avrSWAP, aviFAIL, accINFILE, avcOUTNAME, avcMSG) BIND (C, NAME='DISCON')
! DO NOT REMOVE or MODIFY LINES starting with "!DEC$" or "!GCC$"
! !DEC$ specifies attributes for IVF and !GCC$ specifies attributes for gfortran

USE, INTRINSIC  :: ISO_C_Binding
USE             :: ROSCO_Types
USE             :: ReadSetParameters
USE             :: ControllerBlocks
USE             :: Controllers
USE             :: Constants
USE             :: Filters
USE             :: Functions
USE             :: ExtControl
USE             :: ROSCO_IO
USE             :: ZeroMQInterface

IMPLICIT NONE
! Enable .dll export
#ifndef IMPLICIT_DLLEXPORT
!DEC$ ATTRIBUTES DLLEXPORT :: DISCON
!GCC$ ATTRIBUTES DLLEXPORT :: DISCON
#endif

!------------------------------------------------------------------------------------------------------------------------------
! Variable declaration and initialization
!------------------------------------------------------------------------------------------------------------------------------

! Passed Variables:
!REAL(ReKi), INTENT(IN)      :: from_SC(*)       ! DATA from the super controller
!REAL(ReKi), INTENT(INOUT)   :: to_SC(*)         ! DATA to the super controller

REAL(ReKi),                  INTENT(INOUT)   :: avrSWAP(*)                       ! The swap array, used to pass data to, and receive data from, the DLL controller.
INTEGER(C_INT),                 INTENT(INOUT)   :: aviFAIL                          ! A flag used to indicate the success of this DLL call set as follows: 0 if the DLL call was successful, >0 if the DLL call was successful but cMessage should be issued as a warning messsage, <0 if the DLL call was unsuccessful or for any other reason the simulation is to be stopped at this point with cMessage as the error message.
CHARACTER(KIND=C_CHAR),         INTENT(IN   )   :: accINFILE(NINT(avrSWAP(50)))     ! The name of the parameter input file
CHARACTER(KIND=C_CHAR),         INTENT(IN   )   :: avcOUTNAME(NINT(avrSWAP(51)))    ! OUTNAME (Simulation RootName)
CHARACTER(KIND=C_CHAR),         INTENT(INOUT)   :: avcMSG(NINT(avrSWAP(49)))        ! MESSAGE (Message from DLL to simulation code [ErrMsg])  The message which will be displayed by the calling program if aviFAIL <> 0.
CHARACTER(SIZE(avcOUTNAME))                   :: RootName                         ! a Fortran version of the input C string (not considered an array here)    [subtract 1 for the C null-character]
CHARACTER(SIZE(avcMSG)-1)                       :: ErrMsg                           ! a Fortran version of the C string argument (not considered an array here) [subtract 1 for the C null-character]

TYPE(ControlParameters),        SAVE           :: CntrPar
TYPE(LocalVariables),           SAVE           :: LocalVar
TYPE(ObjectInstances),          SAVE           :: objInst
TYPE(PerformanceData),          SAVE           :: PerfData
TYPE(DebugVariables),           SAVE           :: DebugVar
TYPE(ErrorVariables),           SAVE           :: ErrVar
TYPE(ExtControlType),           SAVE           :: ExtDLL


CHARACTER(*),                   PARAMETER      :: RoutineName = 'ROSCO'

! ===== WFCRL BRIDGE: variable declarations =====
INTEGER(IntKi), SAVE                 :: wfcrl_turbine_id = 0
INTEGER(IntKi), SAVE                 :: wfcrl_applied_step = -1
LOGICAL, SAVE                        :: wfcrl_init_done = .FALSE.
! Farm control protocol: 0-yaw, 1-power+minPitch, 2-pitch, 3-pitch+yaw, 4-power+minPitch+yaw
INTEGER(IntKi), SAVE                 :: wfcrl_mode = -1
REAL(ReKi), SAVE                     :: wfcrl_cmd_yaw = 0.0          ! yaw absolute (deg)
REAL(ReKi), SAVE                     :: wfcrl_cmd_pitch = 0.0       ! pitch absolute (deg)
REAL(ReKi), SAVE                     :: wfcrl_cmd_power = 0.0       ! power target (MW)
REAL(ReKi), SAVE                     :: wfcrl_cmd_min_pitch = 0.0   ! min pitch limit (deg)
REAL(ReKi)                          :: wfcrl_meas_time
INTEGER(IntKi)                       :: wfcrl_io_stat, wfcrl_read_step, wfcrl_parse_id, wfcrl_i, wfcrl_c_len
CHARACTER(256)                       :: wfcrl_line, wfcrl_token, wfcrl_tmp, wfcrl_dll_infile
CHARACTER(32)                        :: wfcrl_tstr
LOGICAL                              :: wfcrl_fexist, wfcrl_found
REAL(ReKi)                          :: wfcrl_tmp_val
! ===== END WFCRL BRIDGE declarations =====

RootName = TRANSFER(avcOUTNAME, RootName)
CALL GetRoot(RootName,RootName)
!------------------------------------------------------------------------------------------------------------------------------
! Main control calculations
!------------------------------------------------------------------------------------------------------------------------------

! Check for restart
IF ( (NINT(avrSWAP(1)) == -9) .AND. (aviFAIL >= 0))  THEN ! Read restart files
    CALL ReadRestartFile(avrSWAP, LocalVar, CntrPar, objInst, PerfData, RootName, SIZE(avcOUTNAME), ErrVar)
    IF ( CntrPar%LoggingLevel > 0 ) THEN
        CALL Debug(LocalVar, CntrPar, DebugVar, ErrVar, avrSWAP, RootName, SIZE(avcOUTNAME))
    END IF 
END IF

! Read avrSWAP array into derived types/variables
CALL ReadAvrSWAP(avrSWAP, LocalVar, CntrPar, ErrVar)

! ===== WFCRL BRIDGE: Read turbine ID and external commands =====
IF (.NOT. wfcrl_init_done) THEN
    ! Read turbine ID from accINFILE (DISCON_T{i}.IN, passed via DLL_InFile)
    wfcrl_dll_infile = ''
    DO wfcrl_c_len = 1, 255
        IF (accINFILE(wfcrl_c_len) == C_NULL_CHAR) EXIT
        wfcrl_dll_infile(wfcrl_c_len:wfcrl_c_len) = accINFILE(wfcrl_c_len)
    END DO
    INQUIRE(FILE=TRIM(wfcrl_dll_infile), EXIST=wfcrl_fexist)
    IF (wfcrl_fexist) THEN
        OPEN(93, FILE=TRIM(wfcrl_dll_infile), STATUS='OLD'); READ(93,*,IOSTAT=wfcrl_io_stat) wfcrl_turbine_id; CLOSE(93)
        IF (wfcrl_io_stat /= 0) wfcrl_turbine_id = 1
    ELSE
        wfcrl_turbine_id = 1
    END IF
    wfcrl_init_done = .TRUE.
END IF

! Read farm commands from controls.txt (5-mode protocol)
INQUIRE(FILE='controls.txt', EXIST=wfcrl_fexist)
IF (wfcrl_fexist) THEN
    OPEN(95, FILE='controls.txt', STATUS='OLD', ACTION='READ')
    READ(95, '(A)', IOSTAT=wfcrl_io_stat) wfcrl_line
    IF (wfcrl_io_stat == 0) THEN
        wfcrl_i = INDEX(wfcrl_line, '=')
        IF (wfcrl_i > 0) THEN
            wfcrl_tmp = wfcrl_line(wfcrl_i+1:)
            READ(wfcrl_tmp, *, IOSTAT=wfcrl_io_stat) wfcrl_read_step
        END IF
        IF (wfcrl_read_step > wfcrl_applied_step) THEN
            wfcrl_found = .FALSE.
            DO
                READ(95, '(A)', IOSTAT=wfcrl_io_stat) wfcrl_line
                IF (wfcrl_io_stat /= 0) EXIT
                IF (TRIM(wfcrl_line) == 'END') EXIT
                wfcrl_i = INDEX(wfcrl_line, ' ')
                IF (wfcrl_i > 0) THEN
                    wfcrl_token = wfcrl_line(1:wfcrl_i-1)
                    IF (wfcrl_token(1:1) == 'T') THEN
                        wfcrl_tmp = wfcrl_token(2:)
                        READ(wfcrl_tmp, *, IOSTAT=wfcrl_io_stat) wfcrl_parse_id
                        IF (wfcrl_parse_id == wfcrl_turbine_id) THEN
                            wfcrl_i = INDEX(wfcrl_line, 'mode=');     IF (wfcrl_i > 0) THEN; wfcrl_tmp = wfcrl_line(wfcrl_i+5:); READ(wfcrl_tmp,*) wfcrl_tmp_val; wfcrl_mode = INT(wfcrl_tmp_val); END IF
                            wfcrl_i = INDEX(wfcrl_line, 'yaw=');      IF (wfcrl_i > 0) THEN; wfcrl_tmp = wfcrl_line(wfcrl_i+4:); READ(wfcrl_tmp,*) wfcrl_cmd_yaw; END IF
                            wfcrl_i = INDEX(wfcrl_line, 'pitch=');    IF (wfcrl_i > 0) THEN; wfcrl_tmp = wfcrl_line(wfcrl_i+6:); READ(wfcrl_tmp,*) wfcrl_cmd_pitch; END IF
                            wfcrl_i = INDEX(wfcrl_line, 'power=');    IF (wfcrl_i > 0) THEN; wfcrl_tmp = wfcrl_line(wfcrl_i+6:); READ(wfcrl_tmp,*) wfcrl_cmd_power; END IF
                            wfcrl_i = INDEX(wfcrl_line, 'minpitch='); IF (wfcrl_i > 0) THEN; wfcrl_tmp = wfcrl_line(wfcrl_i+9:); READ(wfcrl_tmp,*) wfcrl_cmd_min_pitch; END IF
                            wfcrl_found = .TRUE.
                            EXIT
                        END IF
                    END IF
                END IF
            END DO
            IF (wfcrl_found) wfcrl_applied_step = wfcrl_read_step
        END IF
    END IF
    CLOSE(95)
END IF
! ===== END WFCRL BRIDGE: external command read =====

! Set Control Parameters
IF (ErrVar%aviFAIL >= 0) THEN
    CALL SetParameters(avrSWAP, accINFILE, SIZE(avcMSG), CntrPar, LocalVar, objInst, PerfData, RootName, ErrVar)
ENDIF

! ===== WFCRL BRIDGE: Inject farm commands into ROSCO parameters =====
IF (wfcrl_applied_step >= 0 .AND. wfcrl_mode >= 0) THEN
    ! Modes 1 & 4: Power tracking with min pitch constraint
    IF (wfcrl_mode == 1 .OR. wfcrl_mode == 4) THEN
        IF (wfcrl_cmd_power > 0.01) THEN
            CntrPar%VS_ConstPower = 1
            CntrPar%VS_RtPwr = wfcrl_cmd_power * 1.0E6  ! MW -> W
        END IF
        IF (wfcrl_cmd_min_pitch > 0.001) THEN
            CntrPar%PS_Mode = 1
            CntrPar%PS_BldPitchMin_N = 1
            IF (.NOT. ALLOCATED(CntrPar%PS_BldPitchMin)) ALLOCATE(CntrPar%PS_BldPitchMin(1))
            IF (.NOT. ALLOCATED(CntrPar%PS_WindSpeeds))  ALLOCATE(CntrPar%PS_WindSpeeds(1))
            CntrPar%PS_WindSpeeds(1)  = 0.0
            CntrPar%PS_BldPitchMin(1) = wfcrl_cmd_min_pitch * 3.14159265 / 180.0  ! deg -> rad
        END IF
    END IF
    ! Modes with yaw: disable ROSCO internal yaw; WFCRL handles it
    IF (wfcrl_mode == 0 .OR. wfcrl_mode == 3 .OR. wfcrl_mode == 4) THEN
        CntrPar%Y_ControlMode = 0
    END IF
END IF
! ===== END WFCRL BRIDGE: parameter injection =====

! Call external controller, if desired
IF (CntrPar%Ext_Mode > 0 .AND. ErrVar%aviFAIL >= 0) THEN
    CALL ExtController(avrSWAP, CntrPar, LocalVar, ExtDLL, ErrVar)
    ! Data from external dll is in ExtDLL%avrSWAP, it's unused in the following code
END IF

! Filter signals
CALL PreFilterMeasuredSignals(CntrPar, LocalVar, DebugVar, objInst, ErrVar)

IF (((LocalVar%iStatus >= 0) .OR. (LocalVar%iStatus <= -8)) .AND. (ErrVar%aviFAIL >= 0))  THEN  ! Only compute control calculations if no error has occurred and we are not on the last time step
    IF ((LocalVar%iStatus == -8) .AND. (ErrVar%aviFAIL >= 0))  THEN ! Write restart files
        CALL WriteRestartFile(LocalVar, CntrPar, ErrVar, objInst, RootName, SIZE(avcOUTNAME))    
    ENDIF
    IF (CntrPar%ZMQ_Mode > 0) THEN
        CALL UpdateZeroMQ(LocalVar, CntrPar, ErrVar)
    ENDIF
    
    CALL WindSpeedEstimator(LocalVar, CntrPar, objInst, PerfData, DebugVar, ErrVar)
    CALL ComputeVariablesSetpoints(CntrPar, LocalVar, objInst, DebugVar, ErrVar)
    CALL StateMachine(CntrPar, LocalVar)
    CALL SetpointSmoother(LocalVar, CntrPar, objInst)
    CALL VariableSpeedControl(avrSWAP, CntrPar, LocalVar, objInst, ErrVar)
    CALL PitchControl(avrSWAP, CntrPar, LocalVar, objInst, DebugVar, ErrVar)
    
    IF (CntrPar%Y_ControlMode > 0) THEN
        CALL YawRateControl(avrSWAP, CntrPar, LocalVar, objInst, DebugVar, ErrVar)
    END IF
    
    IF (CntrPar%Flp_Mode > 0) THEN
        CALL FlapControl(avrSWAP, CntrPar, LocalVar, objInst)
    END IF

    ! Cable control
    IF (CntrPar%CC_Mode > 0) THEN
        CALL CableControl(avrSWAP,CntrPar,LocalVar, objInst, ErrVar)
    END IF

    ! Structural control
    IF (CntrPar%StC_Mode > 0) THEN
        CALL StructuralControl(avrSWAP,CntrPar,LocalVar, objInst, ErrVar)
    END IF
    
    IF ( CntrPar%LoggingLevel > 0 ) THEN
        CALL Debug(LocalVar, CntrPar, DebugVar, ErrVar, avrSWAP, RootName, SIZE(avcOUTNAME))
    END IF 
ELSEIF ((LocalVar%iStatus == -1) .AND. (CntrPar%ZMQ_Mode > 0)) THEN
        CALL UpdateZeroMQ(LocalVar, CntrPar, ErrVar)
END IF

! ===== WFCRL BRIDGE: Apply post-control overrides based on mode =====
IF (wfcrl_applied_step >= 0 .AND. wfcrl_mode >= 0) THEN
    ! Pitch override (modes 2, 3: absolute pitch command, direct overwrite)
    IF ((wfcrl_mode == 2 .OR. wfcrl_mode == 3) .AND. ABS(wfcrl_cmd_pitch) > 0.001) THEN
        LocalVar%PitCom(1) = wfcrl_cmd_pitch * 3.14159265 / 180.0
        LocalVar%PitCom(2) = wfcrl_cmd_pitch * 3.14159265 / 180.0
        LocalVar%PitCom(3) = wfcrl_cmd_pitch * 3.14159265 / 180.0
        LocalVar%BlPitch(1)  = wfcrl_cmd_pitch * 3.14159265 / 180.0
        LocalVar%BlPitch(2)  = wfcrl_cmd_pitch * 3.14159265 / 180.0
        LocalVar%BlPitch(3)  = wfcrl_cmd_pitch * 3.14159265 / 180.0
        LocalVar%BlPitchCMeas = wfcrl_cmd_pitch * 3.14159265 / 180.0
        avrSWAP(42) = wfcrl_cmd_pitch * 3.14159265 / 180.0
        avrSWAP(43) = wfcrl_cmd_pitch * 3.14159265 / 180.0
        avrSWAP(44) = wfcrl_cmd_pitch * 3.14159265 / 180.0
        avrSWAP(45) = wfcrl_cmd_pitch * 3.14159265 / 180.0
    END IF
    ! Yaw absolute override (modes 0, 3, 4: absolute yaw angle)
    ! Convention: yaw in degrees, OpenFAST NacHeading coordinates (0°=+X East, +CCW)
    IF ((wfcrl_mode == 0 .OR. wfcrl_mode == 3 .OR. wfcrl_mode == 4) &
        .AND. ABS(wfcrl_cmd_yaw) > 0.01) THEN
        LocalVar%NacHeading = wfcrl_cmd_yaw
        avrSWAP(48) = (wfcrl_cmd_yaw * 3.14159265 / 180.0 - avrSWAP(37)) * 0.2
    END IF
END IF
! ===== END WFCRL BRIDGE: control override =====


! Add RoutineName to error message
IF (ErrVar%aviFAIL < 0) THEN
    ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
    print * , TRIM(ErrVar%ErrMsg)
ENDIF
ErrMsg = ADJUSTL(TRIM(ErrVar%ErrMsg))
avcMSG = TRANSFER(ErrMsg//C_NULL_CHAR, avcMSG, LEN(ErrMsg)+1)
avcMSG = TRANSFER(ErrMsg//C_NULL_CHAR, avcMSG, SIZE(avcMSG))
aviFAIL = ErrVar%aviFAIL
ErrVar%ErrMsg = ''

! ===== WFCRL BRIDGE: Write measurements =====
wfcrl_meas_time = avrSWAP(2)
WRITE(wfcrl_tstr, '(I0)') wfcrl_turbine_id
OPEN(96, FILE='measurements_T'//TRIM(wfcrl_tstr)//'.txt', STATUS='REPLACE')
WRITE(96,'(A,I0,A,F10.4,A,I0)') 'step=',wfcrl_applied_step,' t=',wfcrl_meas_time,' mode=',wfcrl_mode
WRITE(96,'(A,F12.2,A,F12.4,A,F12.2)') ' genpwr=',LocalVar%VS_GenPwr/1000.0, &
    ' genspd=',LocalVar%GenSpeed*9.5493,' gentq=',LocalVar%GenTq
WRITE(96,'(A,F12.4,A,F12.4)') ' rotspd=',LocalVar%RotSpeed*9.5493, &
    ' wind_x=',LocalVar%HorWindV
WRITE(96,'(A,F10.4,A,F10.4)') ' blpitch=',LocalVar%BlPitchCMeas*57.29578, &
    ' nacyaw=',LocalVar%NacHeading
WRITE(96,'(A,F12.2,A,F12.2,A,F12.2)') ' mip1=',LocalVar%rootMOOP(1), &
    ' moop1=',LocalVar%rootMOOP(2),' mzb1=',LocalVar%rootMOOP(3)
CLOSE(96)
! ===== END WFCRL BRIDGE: measurements =====

RETURN
END SUBROUTINE DISCON
