cmake_minimum_required(VERSION 3.14)
project(multiplex)

# 1. Fetch the community pd.cmake engine
include(FetchContent)
FetchContent_Declare(
  pd_cmake
  GIT_REPOSITORY https://github.com/pure-data/pd.cmake.git
  GIT_TAG main
)
FetchContent_MakeAvailable(pd_cmake)

# 2. Include the engine logic
include("${pd_cmake_SOURCE_DIR}/pd.cmake")

# 3. Add the standalone external
pd_add_external(sine~ 
    src/sine~.c
)

# 4. Manual Documentation Install (for 'make install')
install(FILES 
    help/sine~-help.pd 
    DESTINATION .
)

# 5. Flat Copy for Local Testing (for 'make')
# This copies it directly into the build folder next to the binary
add_custom_command(TARGET sine_tilde POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${CMAKE_CURRENT_SOURCE_DIR}/help/sine~-help.pd"
    "${CMAKE_CURRENT_BINARY_DIR}/sine~-help.pd"
    COMMENT "Moving help file next to binary..."
)
