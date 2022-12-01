include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../src)

# If pkg-config is not installed on the system, then the
# CHECK_INSTALL_DIR variable must be set to the install
# location of Check. For example, on Windows, this may
# be: C:/Program Files/check
# set(CHECK_INSTALL_DIR "C:/Program Files/check")

find_package(Check REQUIRED)
include_directories(${CHECK_INCLUDE_DIRS})
link_directories(${CHECK_LIBRARY_DIRS})

# enable coverage flags in all CMakeList files to let the generation of .gcno files
SET(CMAKE_CXX_FLAGS "-g -O0 -Wall -fprofile-arcs -ftest-coverage")
SET(CMAKE_C_FLAGS "-g -O0 -Wall -W -fprofile-arcs -ftest-coverage")
SET(CMAKE_EXE_LINKER_FLAGS "-fprofile-arcs -ftest-coverage")

# Code Coverage Configuration
option(CODE_COVERAGE "Enable coverage reporting" ON)

set(CHECK_SOURCES
  check_money.c
)

set(FIFOPARAM_SOURCES
  check_fifoparam.c
)


add_executable(check_money ${CHECK_SOURCES})
target_link_libraries(check_money libby 
${CHECK_LIBRARIES})

add_executable(check_fifo ${FIFOPARAM_SOURCES})
target_link_libraries(check_fifo libby ${CHECK_LIBRARIES})


