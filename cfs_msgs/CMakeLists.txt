cmake_minimum_required(VERSION 3.5)
project(cfs_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.

find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/CCSDSPrimaryHeader.msg"
	"msg/CCSDSExtendedHeader.msg"
	"msg/ESHKTlm.msg"
	"msg/EVSHKTlm.msg"
	"msg/SBHKTlm.msg"
	"msg/TBLHKTlm.msg"
	"msg/TIMEHKTlm.msg"
	"msg/TIMEDIAGTlm1.msg"
	"msg/TIMEDIAGTlm2.msg"
	"msg/SBSTATsTlm.msg"
	"msg/SBPipeDepthStatsTlm1.msg"
	"msg/SBPipeDepthStatsTlm2.msg"
	"msg/ESAPPTlm.msg"
	"msg/TBLREGTlm.msg"
	"msg/SBALLSUBsTlm.msg"
	"msg/SBOneSubTlm.msg"
	"msg/ESShellTlm.msg"
	"msg/ESMEMSTATSTlm.msg"
	"msg/ESBlockStatsTlm1.msg"
	"msg/EsCmd.msg"
	"msg/CfeEsRestartCc.msg"
	"msg/CfeEsShellCc.msg"
	"msg/CfeEsStartAppCc.msg"
	"msg/CfeEsStopAppCc.msg"
	"msg/CfeEsRestartAppCc.msg"
	"msg/CfeEsReloadAppCc.msg"
	"msg/CfeEsQueryOneCc.msg"
	"msg/CfeEsQueryAllCc.msg"
	"msg/CfeEsWriteSyslogCc.msg"
	"msg/CfeEsWriteErLogCc.msg"
	"msg/CfeEsStartPerfDataCc.msg"
	"msg/CfeEsStopPerfDataCc.msg"
	"msg/CfeEsSetPerfFilterMaskCc.msg"
	"msg/CfeEsSetPerfTriggerMaskCc.msg"
	"msg/CfeEsOverWriteSyslogCc.msg"
	"msg/CfeEsSetMaxPrCountCc.msg"
	"msg/CfeEsDeleteCdsCc.msg"
	"msg/CfeEsSendMemPoolStatsCc.msg"
	"msg/CfeEsDumpCdsRegistryCc.msg"
	"msg/CfeEsQueryAllTasksCc.msg"
	"msg/SbCmd.msg"
	"msg/CfeSbSendRoutingInfoCc.msg"
	"msg/CfeSbEnableRouteCc.msg"
	"msg/CfeSbDisableRouteCc.msg"
	"msg/CfeSbSendPipeInfoCc.msg"
	"msg/CfeSbSendMapInfoCc.msg"
	"msg/TblCmd.msg"
	"msg/CfeTblLoadCc.msg"
	"msg/CfeTblDumpCc.msg"
	"msg/CfeTblValidateCc.msg"
	"msg/CfeTblActivateCc.msg"
	"msg/CfeTblDumpRegistryCc.msg"
	"msg/CfeTblSendRegistryCc.msg"
	"msg/CfeTblDeleteCdsCc.msg"
	"msg/CfeTblAbortLoadCc.msg"
	"msg/TimeCmd.msg"
	"msg/CfeTimeSetSourceCc.msg"
	"msg/CfeTimeSetStateCc.msg"
	"msg/CfeTimeAddDelayCc.msg"
	"msg/CfeTimeSubDelayCc.msg"
	"msg/CfeTimeSetTimeCc.msg"
	"msg/CfeTimeSetMetCc.msg"
	"msg/CfeTimeSetStcfCc.msg"
	"msg/CfeTimeSetLeapSecondsCc.msg"
	"msg/CfeTimeAddAdjustCc.msg"
	"msg/CfeTimeSubAdjustCc.msg"
	"msg/CfeTimeAdd1HzAdjustmentCc.msg"
	"msg/CfeTimeSub1HzAdjustmentCc.msg"
	"msg/CfeTimeSetSignalCc.msg"
	"msg/EvsCmd.msg"
	"msg/CfeEvsEnableEventTypeCc.msg"
	"msg/CfeEvsDisableEventTypeCc.msg"
	"msg/CfeEvsSetEventFormatModeCc.msg"
	"msg/CfeEvsEnableAppEventTypeCc.msg"
	"msg/CfeEvsDisableAppEventTypeCc.msg"
	"msg/CfeEvsEnableAppEventsCc.msg"
	"msg/CfeEvsDisableAppEventsCc.msg"
	"msg/CfeEvsResetAppCounterCc.msg"
	"msg/CfeEvsSetFilterCc.msg"
	"msg/CfeEvsEnablePortsCc.msg"
	"msg/CfeEvsDisablePortsCc.msg"
	"msg/CfeEvsResetFilterCc.msg"
	"msg/CfeEvsResetAllFiltersCc.msg"
	"msg/CfeEvsAddEventFilterCc.msg"
	"msg/CfeEvsDeleteEventFilterCc.msg"
	"msg/CfeEvsWriteAppDataFileCc.msg"
	"msg/CfeEvsWriteLogDataFileCc.msg"
	"msg/CfeEvsSetLogModeCc.msg"
	"msg/CiLabCmd.msg"
	"msg/CiModifyPduFilesizeCc.msg"
	"msg/CiDropPdusCc.msg"
	"msg/CiCapturePdusCc.msg"
	"msg/ToLabCmd.msg"
	"msg/ToAddPktCc.msg"
	"msg/ToRemovePktCc.msg"
	"msg/ToOutputEnableCc.msg"
	"msg/SampleAppCmd.msg"
	"msg/EsMsg8H.msg"
	"msg/StructCFEESNoArgsCmdT.msg"
	"msg/StructCFEESDeleteCDSCmdT.msg"
	"msg/StructCFEESDumpCDSRegCmdT.msg"
	"msg/StructCFEESOverWriteSysLogCmdT.msg"
	"msg/StructCFEESPerfSetFilterMaskCmdT.msg"
	"msg/StructCFEESPerfSetTrigMaskCmdT.msg"
	"msg/StructCFEESPerfStartCmdT.msg"
	"msg/StructCFEESPerfStopCmdT.msg"
	"msg/StructCFEESQueryAllCmdT.msg"
	"msg/StructCFEESQueryAllTasksCmdT.msg"
	"msg/StructCFEESAppNameCmdT.msg"
	"msg/StructCFEESAppReloadCmdT.msg"
	"msg/StructCFEESRestartCmdT.msg"
	"msg/StructCFEESSetMaxPRCountCmdT.msg"
	"msg/StructCFEESShellCmdT.msg"
	"msg/StructCFEESStartAppCmdT.msg"
	"msg/StructCFEESTlmPoolStatsCmdT.msg"
	"msg/StructCFEESWriteERlogCmdT.msg"
	"msg/StructCFEESWriteSyslogCmdT.msg"
	"msg/SbMsg8H.msg"
	"msg/StructCFESBEnRoutCmdT.msg"
	"msg/StructCFESBCmdHdrT.msg"
	"msg/StructCFESBWriteFileInfoCmdT.msg"
	"msg/TblMsg8H.msg"
	"msg/StructCFETBLAbortLdCmdT.msg"
	"msg/StructCFETBLActivateCmdT.msg"
	"msg/StructCFETBLDelCDSCmdT.msg"
	"msg/StructCFETBLDumpCmdT.msg"
	"msg/StructCFETBLDumpRegCmdT.msg"
	"msg/StructCFETBLLoadCmdT.msg"
	"msg/StructCFETBLNoArgsCmdT.msg"
	"msg/StructCFETBLTlmRegCmdT.msg"
	"msg/StructCFETBLValidateCmdT.msg"
	"msg/TimeMsg8H.msg"
	"msg/StructCFETIME1HzAdjCmdT.msg"
	"msg/StructCFETIMETimeCmdT.msg"
	"msg/StructCFETIMENoArgsCmdT.msg"
	"msg/StructCFETIMESignalCmdT.msg"
	"msg/StructCFETIMESourceCmdT.msg"
	"msg/StructCFETIMEStateCmdT.msg"
	"msg/EvsMsg8H.msg"
	"msg/StructCFEEVSAppNameEventIDMaskCmdT.msg"
	"msg/StructCFEEVSAppNameBitMaskCmdT.msg"
	"msg/StructCFEEVSAppNameCmdT.msg"
	"msg/StructCFEEVSBitMaskCmdT.msg"
	"msg/StructCFEEVSAppDataCmdT.msg"
	"msg/StructCFEEVSLogFileCmdT.msg"
	"msg/StructCFEEVSAppNameEventIDCmdT.msg"
	"msg/StructCFEEVSModeCmdT.msg"
	"msg/CiLabMsg.msg"
	"msg/ToLabMsg.msg"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
