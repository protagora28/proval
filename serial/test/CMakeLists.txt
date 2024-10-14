add_executable(test_serial_message unit/message/main.cpp)
target_link_libraries(test_serial_message serial test_utils)
target_include_directories(test_serial_message PRIVATE ../include)
target_link_libraries(test_serial_message argparse serial test_utils plog-helper)

add_executable(test_serial_encoder unit/encoder/main.cpp)
target_link_libraries(test_serial_encoder serial test_utils)
target_include_directories(test_serial_encoder PRIVATE ../include)
target_link_libraries(test_serial_encoder argparse serial test_utils plog-helper)

add_executable(test_serial_end_to_end integration/end_to_end/main.cpp)
target_link_libraries(test_serial_end_to_end pthread serial test_utils plog-helper)
target_include_directories(test_serial_end_to_end PRIVATE ../include integration/common/include)
target_link_libraries(test_serial_end_to_end argparse serial test_utils plog-helper)

add_executable(test_serial_standalone_client integration/standalone_client/main.cpp)
target_include_directories(test_serial_standalone_client PRIVATE ../include)
target_link_libraries(test_serial_standalone_client argparse serial test_utils plog-helper)

set_source_files_properties(integration/standalone_server_c/main.c PROPERTIES LANGUAGE CXX)
add_executable(test_serial_standalone_server_c integration/standalone_server_c/main.c)
target_include_directories(test_serial_standalone_server_c PRIVATE ../include)

add_executable(test_serial_standalone_server_cpp integration/standalone_server_cpp/main.cpp)
target_include_directories(test_serial_standalone_server_cpp PRIVATE ../include integration/common/include)
target_link_libraries(test_serial_standalone_server_cpp argparse serial test_utils plog-helper)

add_test(NAME serial/unit/serial_message COMMAND test_serial_message --log-level debug)
add_test(NAME serial/unit/serial_encoder COMMAND test_serial_encoder --log-level debug)
add_test(NAME serial/integration COMMAND test_serial_end_to_end --log-level debug)
