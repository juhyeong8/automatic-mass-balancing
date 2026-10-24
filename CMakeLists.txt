cmake_minimum_required(VERSION 3.10)
project(AMBS LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 11)

# Xsens wrapper lib
add_library(xsens_wrap STATIC xsens_interface.cpp)
target_include_directories(xsens_wrap PUBLIC xspublic)
target_link_directories(xsens_wrap PUBLIC
  ${CMAKE_SOURCE_DIR}/xspublic/xscontroller
  ${CMAKE_SOURCE_DIR}/xspublic/xscommon
  ${CMAKE_SOURCE_DIR}/xspublic/xstypes
)
target_link_libraries(xsens_wrap PUBLIC xscontroller xscommon xstypes pthread rt dl)

# ---- 공통 소스 묶음 ----
set(AMBS_HW_SRCS i2c_bus.cpp tca9548a.cpp pololu_tic_i2c.cpp)

# executables
add_executable(main main.cpp realTimeHandler.cpp ${AMBS_HW_SRCS})
target_link_libraries(main PRIVATE xsens_wrap)

add_executable(mainCalibrateAMBS mainCalibrateAMBS.cpp ${AMBS_HW_SRCS})

# ★ vertical 쪽도 i2c 쓰면 동일하게 (안 쓰면 ${AMBS_HW_SRCS} 빼세요)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/main_vertical_balancing.cpp")
  add_executable(main_vertical_balancing main_vertical_balancing.cpp realTimeHandler.cpp ${AMBS_HW_SRCS})
  target_link_libraries(main_vertical_balancing PRIVATE xsens_wrap)
endif()

