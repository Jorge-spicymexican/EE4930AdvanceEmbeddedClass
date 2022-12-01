# Adding gcov flags to the gcc compliier
SET(CMAKE_CXX_FLAGS "-g -O0 -Wall -fprofile-arcs -ftest-coverage")
SET(CMAKE_C_FLAGS "-g -O0 -Wall -W -fprofile-arcs -ftest-coverage")
SET(CMAKE_EXE_LINKER_FLAGS "-fprofile-arcs -ftest-coverage")
set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE ON)

set(LIB_SOURCES
  money.c
  fifoparam.c
)

set(MAIN_SOURCES
  UnitTestBSPmain.c
)

set(HEADERS 
  ${CONFIG_HEADER}
  money.h
  fifoparam.h
  defines.h
)

add_library(libby STATIC ${LIB_SOURCES} ${HEADERS})

add_executable(main ${HEADERS} ${MAIN_SOURCES})
target_link_libraries(main libby)

install(TARGETS libby
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/money.h DESTINATION include)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/fifoparam.h 
DESTINATION include)
