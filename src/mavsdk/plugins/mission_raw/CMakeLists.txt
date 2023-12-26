target_sources(mavsdk
    PRIVATE
    mission_raw.cpp
    mission_raw_impl.cpp
    mission_import.cpp
)

target_include_directories(mavsdk PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/mavsdk>
    )

file(GLOB_RECURSE HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
install(FILES
    ${HEADERS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mavsdk/plugins/mission_raw
)

list(APPEND UNIT_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/mission_import_test.cpp
)
set(UNIT_TEST_SOURCES ${UNIT_TEST_SOURCES} PARENT_SCOPE)
