cmake_minimum_required(VERSION 3.8)
project(hsl_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

set(msg_files
  "msg/ContractHeader.msg"
  "msg/Point2.msg"
  "msg/Polygon2.msg"
  "msg/EgoState.msg"
  "msg/CoverageGrid.msg"
  "msg/Obstacle2.msg"
  "msg/LocalObstacleSnapshot.msg"
  "msg/BeliefCell.msg"
  "msg/OpponentBelief.msg"
  "msg/MatchState.msg"
  "msg/GoalZone.msg"
  "msg/TopologyNode.msg"
  "msg/TopologyEdge.msg"
  "msg/ExecutionState.msg"
  "msg/PathPlan.msg"
  "msg/MotionCandidate.msg"
  "msg/OpponentTrack.msg"
  "msg/OptionFeedback.msg"
  "msg/OptionGoal.msg"
  "msg/OptionResult.msg"
  "msg/SafetyStatus.msg"
  "msg/SupervisorHeartbeat.msg"
  "msg/WatchdogHealth.msg"
  "msg/RuleEvent.msg"
  "msg/WorldSnapshot.msg"
)

set(srv_files
  "srv/StartStage.srv"
  "srv/SetGoalZone.srv"
  "srv/ResetStage.srv"
  "srv/ResolveEvent.srv"
  "srv/RearmSafety.srv"
)

set(action_files
  "action/ExecuteOption.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(contract_schema_test test/test_contract_schema.py)
endif()

ament_package()
