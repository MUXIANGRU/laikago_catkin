# CMake generated Testfile for 
# Source directory: /home/mxr/dep/any_node/param_io
# Build directory: /home/mxr/dep/any_node/param_io/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_param_io_rostest_test_param_io.test "/home/mxr/dep/any_node/param_io/build/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/home/mxr/dep/any_node/param_io/build/test_results/param_io/rostest-test_param_io.xml" "--return-code" "/opt/ros/kinetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/mxr/dep/any_node/param_io --package=param_io --results-filename test_param_io.xml --results-base-dir \"/home/mxr/dep/any_node/param_io/build/test_results\" /home/mxr/dep/any_node/param_io/test/param_io.test ")
subdirs(gtest)
