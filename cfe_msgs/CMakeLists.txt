cmake_minimum_required(VERSION 3.5)
project(cfe_msgs)

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
  "msg/CFEESAppInfo.msg"
  "msg/CFEESBlockStats.msg"
  "msg/CFEESMemPoolStats.msg"
  "msg/CFETIMESysTime.msg"
  "msg/CFEMSGTelemetryHeader.msg"
  "msg/CFEMSGTelemetrySecondaryHeadert.msg"
  "msg/CFEESOneAppTlmPayload.msg"
  "msg/CFEESOneAppTlm.msg"
  "msg/CFEESPoolStatsTlmPayload.msg"
  "msg/CFEESMemStatsTlm.msg"
  "msg/CFEESHousekeepingTlmPayload.msg"
  "msg/CFEESHousekeepingTlm.msg"
  "msg/CFEEVSPacketID.msg"
  "msg/CFEEVSLongEventTlmPayload.msg"
  "msg/CFEEVSLongEventTlm.msg"
  "msg/CFEMSGCommandHeader.msg"
  "msg/CFEMSGCommandSecondaryHeadert.msg"
  "msg/CFEESStartPerfCmdPayload.msg"
  "msg/CFEESStartPerfDataCmd.msg"
  "msg/CFEESStopPerfCmdPayload.msg"
  "msg/CFEESStopPerfDataCmd.msg"
  "msg/CFEESSetPerfFilterMaskCmdPayload.msg"
  "msg/CFEESSetPerfFilterMaskCmd.msg"
  "msg/CFEESSetPerfTrigMaskCmdPayload.msg"
  "msg/CFEESSetPerfTriggerMaskCmd.msg"
  "msg/CFESBMsgIdt.msg"
  "msg/CFEESNoArgsCmd.msg"
  "msg/CFEESRestartCmdPayload.msg"
  "msg/CFEESRestartCmd.msg"
  "msg/CFEESFileNameCmdPayload.msg"
  "msg/CFEESFileNameCmd.msg"
  "msg/CFEESOverWriteSysLogCmdPayload.msg"
  "msg/CFEESOverWriteSysLogCmd.msg"
  "msg/CFEESStartAppCmdPayload.msg"
  "msg/CFEESStartApp.msg"
  "msg/CFEESAppNameCmdPayload.msg"
  "msg/CFEESAppNameCmd.msg"
  "msg/CFEESAppReloadCmdPayload.msg"
  "msg/CFEESReloadAppCmd.msg"
  "msg/CFEESSetMaxPRCountCmdPayload.msg"
  "msg/CFEESSetMaxPRCountCmd.msg"
  "msg/CFEESDeleteCDSCmdPayload.msg"
  "msg/CFEESDeleteCDSCmd.msg"
  "msg/CFEESSendMemPoolStatsCmdPayload.msg"
  "msg/CFEESSendMemPoolStatsCmd.msg"
  "msg/CFEESDumpCDSRegistryCmdPayload.msg"
  "msg/CFEESDumpCDSRegistryCmd.msg"
  "msg/CFEEVSAppTlmData.msg"
  "msg/CFEEVSHousekeepingTlmPayload.msg"
  "msg/CFEEVSHousekeepingTlm.msg"
  "msg/CFEEVSLogFileCmdPayload.msg"
  "msg/CFEEVSWriteLogDataFileCmd.msg"
  "msg/CFEEVSSetLogModePayload.msg"
  "msg/CFEEVSSetLogModeCmd.msg"
  "msg/CFEEVSNoArgsCmd.msg"
  "msg/CFEEVSAppDataCmdPayload.msg"
  "msg/CFEEVSWriteAppDataFileCmd.msg"
  "msg/CFEEVSSetEventFormatCodePayload.msg"
  "msg/CFEEVSSetEventFormatModeCmd.msg"
  "msg/CFEEVSBitMaskCmdPayload.msg"
  "msg/CFEEVSBitMaskCmd.msg"
  "msg/CFEEVSAppNameCmdPayload.msg"
  "msg/CFEEVSAppNameCmd.msg"
  "msg/CFEEVSAppNameEventIDCmdPayload.msg"
  "msg/CFEEVSAppNameEventIDCmd.msg"
  "msg/CFEEVSAppNameBitMaskCmdPayload.msg"
  "msg/CFEEVSAppNameBitMaskCmd.msg"
  "msg/CFEEVSAppNameEventIDMaskCmdPayload.msg"
  "msg/CFEEVSAppNameEventIDMaskCmd.msg"
  "msg/CFEEVSShortEventTlmPayload.msg"
  "msg/CFEEVSShortEventTlm.msg"
  "msg/CFESBQost.msg"
  "msg/CFESBHousekeepingTlmPayload.msg"
  "msg/CFESBHousekeepingTlm.msg"
  "msg/CFESBPipeDepthStats.msg"
  "msg/CFESBStatsTlmPayload.msg"
  "msg/CFESBStatsTlm.msg"
  "msg/CFESBSubEntries.msg"
  "msg/CFESBAllSubscriptionsTlmPayload.msg"
  "msg/CFESBAllSubscriptionsTlm.msg"
  "msg/CFESBWriteFileInfoCmdPayload.msg"
  "msg/CFESBWriteFileInfoCmd.msg"
  "msg/CFESBRouteCmdPayload.msg"
  "msg/CFESBRouteCmd.msg"
  "msg/CFESBSingleSubscriptionTlmPayload.msg"
  "msg/CFESBSingleSubscriptionTlm.msg"
  "msg/CFETBLNotifyCmdPayload.msg"
  "msg/CFETBLNotifyCmd.msg"
  "msg/CFETBLHousekeepingTlmPayload.msg"
  "msg/CFETBLHousekeepingTlm.msg"
  "msg/CFETBLTblRegPacketPayload.msg"
  "msg/CFETBLTableRegistryTlm.msg"
  "msg/CFETBLNoArgsCmd.msg"
  "msg/CFETBLLoadCmdPayload.msg"
  "msg/CFETBLLoadCmd.msg"
  "msg/CFETBLDumpCmdPayload.msg"
  "msg/CFETBLDumpCmd.msg"
  "msg/CFETBLValidateCmdPayload.msg"
  "msg/CFETBLValidateCmd.msg"
  "msg/CFETBLActivateCmdPayload.msg"
  "msg/CFETBLActivateCmd.msg"
  "msg/CFETBLDumpRegistryCmdPayload.msg"
  "msg/CFETBLDumpRegistryCmd.msg"
  "msg/CFETBLSendRegistryCmdPayload.msg"
  "msg/CFETBLSendRegistryCmd.msg"
  "msg/CFETBLDelCDSCmdPayload.msg"
  "msg/CFETBLDeleteCDSCmd.msg"
  "msg/CFETBLAbortLoadCmdPayload.msg"
  "msg/CFETBLAbortLoadCmd.msg"
  "msg/CFETIMENoArgsCmd.msg"
  "msg/CFETIMEToneDataCmdPayload.msg"
  "msg/CFETIMEToneDataCmd.msg"
  "msg/CFETIMEHousekeepingTlmPayload.msg"
  "msg/CFETIMEHousekeepingTlm.msg"
  "msg/CFETIMEDiagnosticTlmPayload.msg"
  "msg/CFETIMEDiagnosticTlm.msg"
  "msg/CFETIMELeapsCmdPayload.msg"
  "msg/CFETIMESetLeapSecondsCmd.msg"
  "msg/CFETIMEStateCmdPayload.msg"
  "msg/CFETIMESetStateCmd.msg"
  "msg/CFETIMESourceCmdPayload.msg"
  "msg/CFETIMESetSourceCmd.msg"
  "msg/CFETIMESignalCmdPayload.msg"
  "msg/CFETIMESetSignalCmd.msg"
  "msg/CFETIMETimeCmdPayload.msg"
  "msg/CFETIMETimeCmd.msg"
  "msg/CFETIMEOneHzAdjustmentCmdPayload.msg"
  "msg/CFETIMEOneHzAdjustmentCmd.msg"
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
