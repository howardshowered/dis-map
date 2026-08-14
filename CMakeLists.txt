cmake_minimum_required(VERSION 3.16)
project(dis_map LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT WIN32)
    message(FATAL_ERROR "This application targets Windows (Win32 + Winsock + GDI+).")
endif()

# ---- Main GUI application -----------------------------------------------
add_executable(dis_map WIN32
    src/main.cpp
)
target_link_libraries(dis_map PRIVATE ws2_32 gdiplus user32 gdi32 shell32)
target_compile_definitions(dis_map PRIVATE UNICODE _UNICODE WIN32_LEAN_AND_MEAN NOMINMAX)
if(MSVC)
    target_compile_options(dis_map PRIVATE /utf-8)   # source/exec charset = UTF-8
endif()

# ---- Test sender (console) — broadcasts moving DIS entities -------------
add_executable(dis_sender
    src/test_sender.cpp
)
target_link_libraries(dis_sender PRIVATE ws2_32)
target_compile_definitions(dis_sender PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)

# Put both binaries in the same output folder so an optional world.png
# background next to dis_map.exe is easy to drop in.
set_target_properties(dis_map dis_sender PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
