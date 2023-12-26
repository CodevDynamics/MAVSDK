target_sources(mavsdk
    PRIVATE
    param_server.cpp
    param_server_impl.cpp
)

target_include_directories(mavsdk PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/mavsdk>
    )

file(GLOB_RECURSE HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
install(FILES
    ${HEADERS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mavsdk/plugins/param_server
)
