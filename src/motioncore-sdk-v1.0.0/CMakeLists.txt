cmake_minimum_required(VERSION 3.20)
project(MotionCoreSDK VERSION 1.0.0 LANGUAGES C CXX)

include(GNUInstallDirs)

option(MOTIONCORE_BUILD_EXAMPLES "Build MotionCore SDK example projects" ON)

list(PREPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
find_package(MotionCore CONFIG REQUIRED)

if(NOT TARGET MotionCore::motioncore_c)
    add_library(MotionCore::motioncore_c SHARED IMPORTED GLOBAL)
    set_target_properties(MotionCore::motioncore_c PROPERTIES
        IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/libmotioncore.so"
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
    )
endif()

if(MOTIONCORE_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY lib/
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
    FILES_MATCHING
        PATTERN "libmotioncore.so*"
)

install(DIRECTORY lib/cmake/MotionCore/
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/MotionCore
)

install(DIRECTORY share/
    DESTINATION ${CMAKE_INSTALL_DATADIR}
)

install(FILES README.md README_zh.md LICENSE
    DESTINATION .
)

if(MOTIONCORE_BUILD_EXAMPLES)
    install(DIRECTORY examples/
        DESTINATION ${CMAKE_INSTALL_DATADIR}/motioncore/examples
        FILES_MATCHING PATTERN "*"
    )
endif()
