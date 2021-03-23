# CMake generated Testfile for 
# Source directory: /home/mxr/catkin_laikago/src/lcm-1.4.0/test/python
# Build directory: /home/mxr/catkin_laikago/src/lcm-1.4.0/build/test/python
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(Python::bool_test "/usr/local/bin/cmake" "-E" "env" "PYTHONPATH=/home/mxr/catkin_laikago/src/lcm-1.4.0/build/test/types:/home/mxr/catkin_laikago/src/lcm-1.4.0/build/lib/python2.7/dist-packages" "/usr/bin/python" "/home/mxr/catkin_laikago/src/lcm-1.4.0/test/python/bool_test.py")
add_test(Python::byte_array_test "/usr/local/bin/cmake" "-E" "env" "PYTHONPATH=/home/mxr/catkin_laikago/src/lcm-1.4.0/build/test/types:/home/mxr/catkin_laikago/src/lcm-1.4.0/build/lib/python2.7/dist-packages" "/usr/bin/python" "/home/mxr/catkin_laikago/src/lcm-1.4.0/test/python/byte_array_test.py")
add_test(Python::lcm_file_test "/usr/local/bin/cmake" "-E" "env" "PYTHONPATH=/home/mxr/catkin_laikago/src/lcm-1.4.0/build/test/types:/home/mxr/catkin_laikago/src/lcm-1.4.0/build/lib/python2.7/dist-packages" "/usr/bin/python" "/home/mxr/catkin_laikago/src/lcm-1.4.0/test/python/lcm_file_test.py")
add_test(Python::lcm_memq_test "/usr/local/bin/cmake" "-E" "env" "PYTHONPATH=/home/mxr/catkin_laikago/src/lcm-1.4.0/build/test/types:/home/mxr/catkin_laikago/src/lcm-1.4.0/build/lib/python2.7/dist-packages" "/usr/bin/python" "/home/mxr/catkin_laikago/src/lcm-1.4.0/test/python/lcm_memq_test.py")
add_test(Python::lcm_thread_test "/usr/local/bin/cmake" "-E" "env" "PYTHONPATH=/home/mxr/catkin_laikago/src/lcm-1.4.0/build/test/types:/home/mxr/catkin_laikago/src/lcm-1.4.0/build/lib/python2.7/dist-packages" "/usr/bin/python" "/home/mxr/catkin_laikago/src/lcm-1.4.0/test/python/lcm_thread_test.py")
add_test(Python::client_server "/usr/local/bin/cmake" "-E" "env" "PYTHONPATH=/home/mxr/catkin_laikago/src/lcm-1.4.0/build/test/types:/home/mxr/catkin_laikago/src/lcm-1.4.0/build/lib/python2.7/dist-packages" "/usr/bin/python" "/home/mxr/catkin_laikago/src/lcm-1.4.0/test/run_client_server_test.py" "/home/mxr/catkin_laikago/src/lcm-1.4.0/build/test/c/test-c-server" "/usr/bin/python" "/home/mxr/catkin_laikago/src/lcm-1.4.0/test/python/client.py")
