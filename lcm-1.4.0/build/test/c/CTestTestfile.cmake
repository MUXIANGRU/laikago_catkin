# CMake generated Testfile for 
# Source directory: /home/mxr/catkin_laikago/src/lcm-1.4.0/test/c
# Build directory: /home/mxr/catkin_laikago/src/lcm-1.4.0/build/test/c
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(C::memq_test "/home/mxr/catkin_laikago/src/lcm-1.4.0/build/test/c/test-c-memq_test")
add_test(C::eventlog_test "/home/mxr/catkin_laikago/src/lcm-1.4.0/build/test/c/test-c-eventlog_test")
add_test(C::client_server "/usr/bin/python" "/home/mxr/catkin_laikago/src/lcm-1.4.0/test/c/../run_client_server_test.py" "/home/mxr/catkin_laikago/src/lcm-1.4.0/build/test/c/test-c-server" "/home/mxr/catkin_laikago/src/lcm-1.4.0/build/test/c/test-c-client")
