set(nodes semio_deixis_node)

add_definitions( -std=c++11 )
foreach(node ${nodes})
  add_executable(${node} ${node}.cpp)
  add_dependencies(${node} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${node}
    ${catkin_LIBRARIES}
    semio
  )
endforeach()

install(TARGETS
  ${nodes}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
