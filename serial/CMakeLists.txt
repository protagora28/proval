set(SOURCES
    src/aws103_regs.cpp
    src/client.cpp
    src/message.cpp
    src/safe_io.cpp
)

set(HEADERS
    include/aws103_regs.hpp
    include/client.hpp
    include/message.hpp
    include/safe_io.hpp
    include/serial_comm_setup.hpp
)

add_library(serial STATIC ${SOURCES} ${HEADERS})
target_include_directories(serial PUBLIC include)
target_link_libraries(serial PUBLIC antenna argparse plog)

add_subdirectory(test)
