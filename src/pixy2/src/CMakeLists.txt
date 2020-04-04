list(APPEND demos
        chirp_command_cpp_demo
        get_blocks_cpp_demo
        get_lines_cpp_demo
        get_raw_frame
        get_rgb_demo
        pan_tilt_demo)

foreach (demo IN LISTS demos)
    add_executable(${demo} ${demo}.cpp)
    target_link_libraries(${demo} PRIVATE pixy2)
endforeach ()
