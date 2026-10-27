! ============================================================
!  SysSubs.f90 — 最小 SysSubs 兼容模块（v2.10.6 迁移）
!
! 上游 ROSCO v2.10.6 的 ControllerBlocks/ExtControl/ReadSetParameters/
!  ROSCO_Helpers 引用 USE SysSubs，但上游不打包该模块（v2.10+ 通常随
!  OpenFAST NWTC_Library 构建提供）。独立 DISCON 构建只需少量常量：
!    NewLine : 换行符（CRLF）
!    PathSep : 路径分隔符（Windows = '\'）
!  注意：Tab 由 Constants.f90 提供（v2.10.6 上游），此处不定义避免歧义。
!  其余 NWTC 工具（GetNewUnit/MaxLineLength/ParseInput*）已在
!  ROSCO_Helpers.f90 本地定义，无需本模块提供。
! ============================================================
MODULE SysSubs
    IMPLICIT NONE

    CHARACTER(*), PARAMETER :: NewLine = ACHAR(13)//ACHAR(10)  ! CRLF
#ifdef _WIN32
    CHARACTER(*), PARAMETER :: PathSep = '\'
#else
    CHARACTER(*), PARAMETER :: PathSep = '/'
#endif

END MODULE SysSubs
