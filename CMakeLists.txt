cmake_minimum_required(VERSION 3.10)
project(AMBS)
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

# ★ I2C 소스 추가 (원래 빠져 있던 부분)
add_executable(main_manual_balancing main_manual_balancing.cpp realTimeHandler.cpp ${AMBS_HW_SRCS})
target_link_libraries(main_manual_balancing PRIVATE xsens_wrap)

# ★ vertical 쪽도 i2c 쓰면 동일하게 (안 쓰면 ${AMBS_HW_SRCS} 빼세요)
add_executable(main_vertical_balancing main_vertical_balancing.cpp realTimeHandler.cpp ${AMBS_HW_SRCS})
target_link_libraries(main_vertical_balancing PRIVATE xsens_wrap)

function(add_ascl_cbh_hils_experiment target_name mode_value)
    add_executable(${target_name}
      ASCL_CBH_HILS_main_imcl_amimcl_experiment.cpp
      ASCL_CBH_HILS_ambs_controller.cpp
      ASCL_CBH_HILS_imcl_estimator.cpp
      ASCL_CBH_HILS_amimcl_estimator.cpp
      realTimeHandler.cpp
      ${AMBS_HW_SRCS}
    )
    target_compile_definitions(${target_name} PRIVATE
      EXPERIMENT_ESTIMATOR_MODE=${mode_value})
    target_link_libraries(${target_name} PRIVATE xsens_wrap)
endfunction()

add_ascl_cbh_hils_experiment(ASCL_CBH_HILS_main_imcl_amimcl_experiment 0)
add_ascl_cbh_hils_experiment(ASCL_CBH_HILS_main_imcl_experiment 1)
add_ascl_cbh_hils_experiment(ASCL_CBH_HILS_main_amimcl_experiment 2)