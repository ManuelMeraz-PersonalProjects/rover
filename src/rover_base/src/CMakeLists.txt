list(APPEND nodes rover_node talker)

foreach(node IN LISTS nodes)
    add_executable(${node} ${node}.cpp)
    ament_target_dependencies(${node} rclcpp std_msgs)
    target_include_directories(${node} PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)

    install(TARGETS ${node}
            EXPORT export_${PROJECT_NAME}
            DESTINATION lib/${PROJECT_NAME})
endforeach()
