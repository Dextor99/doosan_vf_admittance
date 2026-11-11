CXX ?= g++
CXXFLAGS ?= -O2 -g -std=c++17 -Wall -Wextra -Wpedantic
CPPFLAGS += -Iinclude
SDK_ROOT ?= ../BINARY_GL010118_20230324A
SDK_LIB := $(abspath $(SDK_ROOT)/library/Linux/64bits/20.04)

.PHONY: all check clean
all: build/ft_calibration build/ft_compensate_example build/ft_acquire build/ft_auto build/ft_admittance_x build/ft_admittance_xyz build/ft_admittance_xyz_return build/ft_admittance_y_trajectory build/ft_record_poses build/ft_movej_test build/doosan_movej_autoinit build/doosan_movejx_python_match

build:
	mkdir -p build

build/motion_completion_test: tests/motion_completion_test.cpp include/motion_completion.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@

build/joint_steps_test: tests/joint_steps_test.cpp include/joint_steps.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@

build/servo_enable_policy_test: tests/servo_enable_policy_test.cpp include/servo_enable_policy.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@

build/session_status_test: tests/session_status_test.cpp include/session_status.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@

build/matrix_condition_test: tests/matrix_condition_test.cpp include/matrix_condition.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@

build/acquisition_offline_test: tests/acquisition_offline_test.cpp include/ft_acquisition.hpp include/ft_compensator.hpp include/pose_math.hpp include/xjc_serial.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Wno-unused-function -isystem "$(SDK_ROOT)/include" $< -pthread -o $@

build/admittance_x_test: tests/admittance_x_test.cpp include/admittance_x.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@

check: build/motion_completion_test build/joint_steps_test build/servo_enable_policy_test build/session_status_test build/matrix_condition_test build/acquisition_offline_test build/admittance_x_test build/ft_calibration
	./build/motion_completion_test
	./build/joint_steps_test
	./build/servo_enable_policy_test
	./build/session_status_test
	./build/matrix_condition_test
	./build/acquisition_offline_test
	./build/admittance_x_test
	sh tests/calibration_session_cli_test.sh

build/ft_calibration: src/ft_calibration.cpp include/matrix_condition.hpp include/session_status.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@

build/ft_compensate_example: src/compensate_example.cpp include/ft_compensator.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@

build/ft_acquire: src/ft_acquire.cpp include/ft_acquisition.hpp include/ft_compensator.hpp include/pose_math.hpp include/xjc_serial.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -isystem "$(SDK_ROOT)/include" $< \
		"$(SDK_LIB)/libDRFL.a" "$(SDK_LIB)/libPocoNet.so.62" \
		"$(SDK_LIB)/libPocoFoundation.so.62" -pthread \
		-Wl,--disable-new-dtags,-rpath,'$$ORIGIN/../../BINARY_GL010118_20230324A/library/Linux/64bits/20.04' -o $@

build/ft_acquire_debug: CXXFLAGS := -O0 -g -std=c++17 -Wall -Wextra -Wpedantic
build/ft_acquire_debug: src/ft_acquire.cpp include/ft_acquisition.hpp include/ft_compensator.hpp include/pose_math.hpp include/xjc_serial.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -isystem "$(SDK_ROOT)/include" $< \
		"$(SDK_LIB)/libDRFL.a" "$(SDK_LIB)/libPocoNet.so.62" \
		"$(SDK_LIB)/libPocoFoundation.so.62" -pthread \
		-Wl,--disable-new-dtags,-rpath,'$$ORIGIN/../../BINARY_GL010118_20230324A/library/Linux/64bits/20.04' -o $@

build/ft_auto: src/ft_auto.cpp include/ft_acquisition.hpp include/ft_compensator.hpp include/pose_math.hpp include/xjc_serial.hpp include/joint_steps.hpp include/motion_completion.hpp include/servo_enable_policy.hpp include/session_status.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -isystem "$(SDK_ROOT)/include" $< \
		"$(SDK_LIB)/libDRFL.a" "$(SDK_LIB)/libPocoNet.so.62" \
		"$(SDK_LIB)/libPocoFoundation.so.62" -pthread \
		-Wl,--disable-new-dtags,-rpath,'$$ORIGIN/../../BINARY_GL010118_20230324A/library/Linux/64bits/20.04' -o $@

build/ft_admittance_x: src/ft_admittance_x.cpp include/admittance_x.hpp include/ft_acquisition.hpp include/ft_compensator.hpp include/pose_math.hpp include/xjc_serial.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Wno-unused-function -isystem "$(SDK_ROOT)/include" $< \
		"$(SDK_LIB)/libDRFL.a" "$(SDK_LIB)/libPocoNet.so.62" \
		"$(SDK_LIB)/libPocoFoundation.so.62" -pthread \
		-Wl,--disable-new-dtags,-rpath,'$$ORIGIN/../../BINARY_GL010118_20230324A/library/Linux/64bits/20.04' -o $@

build/ft_admittance_xyz: src/ft_admittance_xyz.cpp include/admittance_xyz.hpp include/ft_acquisition.hpp include/ft_compensator.hpp include/pose_math.hpp include/xjc_serial.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Wno-unused-function -isystem "$(SDK_ROOT)/include" $< \
		"$(SDK_LIB)/libDRFL.a" "$(SDK_LIB)/libPocoNet.so.62" \
		"$(SDK_LIB)/libPocoFoundation.so.62" -pthread \
		-Wl,--disable-new-dtags,-rpath,'$$ORIGIN/../../BINARY_GL010118_20230324A/library/Linux/64bits/20.04' -o $@

build/ft_admittance_xyz_return: src/ft_admittance_xyz_return.cpp include/admittance_xyz_return.hpp include/ft_acquisition.hpp include/ft_compensator.hpp include/pose_math.hpp include/xjc_serial.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Wno-unused-function -isystem "$(SDK_ROOT)/include" $< \
		"$(SDK_LIB)/libDRFL.a" "$(SDK_LIB)/libPocoNet.so.62" \
		"$(SDK_LIB)/libPocoFoundation.so.62" -pthread \
		-Wl,--disable-new-dtags,-rpath,'$$ORIGIN/../../BINARY_GL010118_20230324A/library/Linux/64bits/20.04' -o $@

build/ft_admittance_y_trajectory: src/ft_admittance_y_trajectory.cpp include/admittance_xyz_return.hpp include/ft_acquisition.hpp include/ft_compensator.hpp include/pose_math.hpp include/xjc_serial.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Wno-unused-function -isystem "$(SDK_ROOT)/include" $< \
		"$(SDK_LIB)/libDRFL.a" "$(SDK_LIB)/libPocoNet.so.62" \
		"$(SDK_LIB)/libPocoFoundation.so.62" -pthread \
		-Wl,--disable-new-dtags,-rpath,'$$ORIGIN/../../BINARY_GL010118_20230324A/library/Linux/64bits/20.04' -o $@

build/ft_auto_debug: CXXFLAGS := -O0 -g -std=c++17 -Wall -Wextra -Wpedantic
build/ft_auto_debug: src/ft_auto.cpp include/ft_acquisition.hpp include/ft_compensator.hpp include/pose_math.hpp include/xjc_serial.hpp include/joint_steps.hpp include/motion_completion.hpp include/servo_enable_policy.hpp include/session_status.hpp build/ft_calibration | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -isystem "$(SDK_ROOT)/include" $< \
		"$(SDK_LIB)/libDRFL.a" "$(SDK_LIB)/libPocoNet.so.62" \
		"$(SDK_LIB)/libPocoFoundation.so.62" -pthread \
		-Wl,--disable-new-dtags,-rpath,'$$ORIGIN/../../BINARY_GL010118_20230324A/library/Linux/64bits/20.04' -o $@

build/ft_record_poses: src/ft_record_poses.cpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -isystem "$(SDK_ROOT)/include" $< \
		"$(SDK_LIB)/libDRFL.a" "$(SDK_LIB)/libPocoNet.so.62" \
		"$(SDK_LIB)/libPocoFoundation.so.62" -pthread \
		-Wl,--disable-new-dtags,-rpath,'$$ORIGIN/../../BINARY_GL010118_20230324A/library/Linux/64bits/20.04' -o $@

build/ft_record_poses_debug: CXXFLAGS := -O0 -g -std=c++17 -Wall -Wextra -Wpedantic
build/ft_record_poses_debug: src/ft_record_poses.cpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -isystem "$(SDK_ROOT)/include" $< \
		"$(SDK_LIB)/libDRFL.a" "$(SDK_LIB)/libPocoNet.so.62" \
		"$(SDK_LIB)/libPocoFoundation.so.62" -pthread \
		-Wl,--disable-new-dtags,-rpath,'$$ORIGIN/../../BINARY_GL010118_20230324A/library/Linux/64bits/20.04' -o $@

build/ft_movej_test: src/ft_movej_test.cpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -isystem "$(SDK_ROOT)/include" $< \
		"$(SDK_LIB)/libDRFL.a" "$(SDK_LIB)/libPocoNet.so.62" \
		"$(SDK_LIB)/libPocoFoundation.so.62" -pthread \
		-Wl,--disable-new-dtags,-rpath,'$$ORIGIN/../../BINARY_GL010118_20230324A/library/Linux/64bits/20.04' -o $@

build/ft_movej_test_debug: CXXFLAGS := -O0 -g -std=c++17 -Wall -Wextra -Wpedantic
build/ft_movej_test_debug: src/ft_movej_test.cpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -isystem "$(SDK_ROOT)/include" $< \
		"$(SDK_LIB)/libDRFL.a" "$(SDK_LIB)/libPocoNet.so.62" \
		"$(SDK_LIB)/libPocoFoundation.so.62" -pthread \
		-Wl,--disable-new-dtags,-rpath,'$$ORIGIN/../../BINARY_GL010118_20230324A/library/Linux/64bits/20.04' -o $@

build/doosan_movej_minimal: src/doosan_movej_minimal.cpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -isystem "$(SDK_ROOT)/include" $< \
		"$(SDK_LIB)/libDRFL.a" "$(SDK_LIB)/libPocoNet.so.62" \
		"$(SDK_LIB)/libPocoFoundation.so.62" -pthread \
		-Wl,--disable-new-dtags,-rpath,'$$ORIGIN/../../BINARY_GL010118_20230324A/library/Linux/64bits/20.04' -o $@

build/doosan_movej_autoinit: src/doosan_movej_autoinit.cpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -isystem "$(SDK_ROOT)/include" $< \
		"$(SDK_LIB)/libDRFL.a" "$(SDK_LIB)/libPocoNet.so.62" \
		"$(SDK_LIB)/libPocoFoundation.so.62" -pthread \
		-Wl,--disable-new-dtags,-rpath,'$$ORIGIN/../../BINARY_GL010118_20230324A/library/Linux/64bits/20.04' -o $@

build/doosan_movej_autoinit_debug: CXXFLAGS := -O0 -g -std=c++17 -Wall -Wextra -Wpedantic
build/doosan_movej_autoinit_debug: src/doosan_movej_autoinit.cpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -isystem "$(SDK_ROOT)/include" $< \
		"$(SDK_LIB)/libDRFL.a" "$(SDK_LIB)/libPocoNet.so.62" \
		"$(SDK_LIB)/libPocoFoundation.so.62" -pthread \
		-Wl,--disable-new-dtags,-rpath,'$$ORIGIN/../../BINARY_GL010118_20230324A/library/Linux/64bits/20.04' -o $@

build/doosan_movejx_python_match: src/doosan_movejx_python_match.cpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -isystem "$(SDK_ROOT)/include" $< \
		"$(SDK_LIB)/libDRFL.a" "$(SDK_LIB)/libPocoNet.so.62" \
		"$(SDK_LIB)/libPocoFoundation.so.62" -pthread \
		-Wl,--disable-new-dtags,-rpath,'$$ORIGIN/../../BINARY_GL010118_20230324A/library/Linux/64bits/20.04' -o $@

build/doosan_movejx_python_match_debug: CXXFLAGS := -O0 -g -std=c++17 -Wall -Wextra -Wpedantic
build/doosan_movejx_python_match_debug: src/doosan_movejx_python_match.cpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -isystem "$(SDK_ROOT)/include" $< \
		"$(SDK_LIB)/libDRFL.a" "$(SDK_LIB)/libPocoNet.so.62" \
		"$(SDK_LIB)/libPocoFoundation.so.62" -pthread \
		-Wl,--disable-new-dtags,-rpath,'$$ORIGIN/../../BINARY_GL010118_20230324A/library/Linux/64bits/20.04' -o $@

clean:
	rm -f build/ft_calibration build/ft_compensate_example build/ft_acquire build/ft_acquire_debug build/ft_auto build/ft_auto_debug build/ft_admittance_x build/ft_admittance_xyz build/ft_admittance_xyz_return build/ft_admittance_y_trajectory build/ft_record_poses build/ft_record_poses_debug build/ft_movej_test build/ft_movej_test_debug build/doosan_movej_minimal build/doosan_movej_autoinit build/doosan_movej_autoinit_debug build/doosan_movejx_python_match build/doosan_movejx_python_match_debug build/motion_completion_test build/joint_steps_test build/servo_enable_policy_test build/session_status_test build/matrix_condition_test build/acquisition_offline_test build/admittance_x_test
