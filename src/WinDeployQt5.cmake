find_package(Qt5Core REQUIRED)

get_target_property(QMAKE_EXECUTABLE Qt5::qmake IMPORTED_LOCATION)
get_filename_component(QMAKE_DIR ${QMAKE_EXECUTABLE} DIRECTORY)

find_program(WINDEPLOYQT_EXECUTABLE windeployqt HINTS ${QMAKE_DIR})

if(WIN32 AND ${WINDEPLOYQT_EXECUTABLE} STREQUAL "WINDEPLOYQT_EXECUTABLE-NOTFOUND")
    message(FATAL_ERROR "windeployqt not found.")
endif()

function(WinDeployQt dir tgt)
add_custom_command(TARGET ${tgt}
                   COMMAND ${CMAKE_COMMAND} -E
                       env PATH=${QMAKE_DIR}
                       ${WINDEPLOYQT_EXECUTABLE}
                       --verbose 0
                       --no-compiler-runtime
                       --dir ${dir}
                       $<TARGET_FILE:${tgt}>)

set(CMAKE_INSTALL_UCRT_LIBRARIES TRUE)

include(InstallRequiredSystemLibraries)

endfunction()
