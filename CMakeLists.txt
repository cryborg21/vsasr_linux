cmake_minimum_required (VERSION 2.8)

project (vsasr)

link_directories(
  /usr/local/lib/silabs
)

## position_to_axis
include_directories(position_to_axis
  /usr/local/include
)
add_executable (position_to_axis
  src/position_to_axis.cpp
)
target_link_libraries(position_to_axis
  slabhidtouart
)

## get_position
include_directories(get_position
  /usr/local/include
)
add_executable (get_position
  src/get_position.cpp
  src/vsasr.cpp
)
target_link_libraries(get_position
  slabhidtouart
)
