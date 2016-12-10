cmake_minimum_required(VERSION 2.8.3)
project(bassbot)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  sound_play
)

find_package(Threads REQUIRED)

find_package(ALSA REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  FILES
  MidiNote.msg
  CallFoundNotification.msg
  Note.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM RtMidi)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

add_library(RtMidi RtMidi/RtMidi.cpp)
target_link_libraries(RtMidi ${CMAKE_THREAD_LIBS_INIT} ${ALSA_LIBRARIES})
set_target_properties(RtMidi PROPERTIES COMPILE_FLAGS "-D__LINUX_ALSA__")

add_executable(MIDIListener src/MIDIListener.cpp)
target_link_libraries(MIDIListener ${catkin_LIBRARIES} RtMidi)

add_executable(CallFinder src/CallFinder.cpp src/PatternUtilities.cpp)
target_link_libraries(CallFinder ${catkin_LIBRARIES})

add_executable(ResponsePlayer src/ResponsePlayer.cpp src/PatternUtilities.cpp)
target_link_libraries(ResponsePlayer ${catkin_LIBRARIES})

add_executable(RobotInterface src/RobotInterface.cpp src/SerialPort.cpp)
target_link_libraries(RobotInterface ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(SynthInterface src/SynthInterface.cpp)
target_link_libraries(SynthInterface ${catkin_LIBRARIES})

add_executable(ArduinoCommDelayTimer src/ArduinoCommDelayTimer.cpp src/SerialPort.cpp)
target_link_libraries(ArduinoCommDelayTimer ${Boost_LIBRARIES})
