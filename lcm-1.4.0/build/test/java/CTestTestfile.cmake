# CMake generated Testfile for 
# Source directory: /home/mxr/catkin_laikago/src/lcm-1.4.0/test/java
# Build directory: /home/mxr/catkin_laikago/src/lcm-1.4.0/build/test/java
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(Java::client_server "/usr/bin/python" "/home/mxr/catkin_laikago/src/lcm-1.4.0/test/java/../run_client_server_test.py" "/home/mxr/catkin_laikago/src/lcm-1.4.0/build/test/c/test-c-server" "/usr/bin/java" "-cp" "/home/mxr/catkin_laikago/src/lcm-1.4.0/build/test/java/lcm-test.jar:/home/mxr/catkin_laikago/src/lcm-1.4.0/build/test/types/lcm-test-types.jar:/home/mxr/catkin_laikago/src/lcm-1.4.0/build/lcm-java/lcm.jar:/home/mxr/catkin_laikago/src/lcm-1.4.0/build/test/java/hamcrest-core-1.3/hamcrest-core-1.3.jar:/home/mxr/catkin_laikago/src/lcm-1.4.0/build/test/java/junit-4.11/junit-4.11.jar" "LcmTestClient")
subdirs("hamcrest-core-1.3")
subdirs("junit-4.11")
