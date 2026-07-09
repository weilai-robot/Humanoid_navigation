cmake_minimum_required(VERSION 3.8)
project(open3d_loc)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
        set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_BUILD_TYPE "Release")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(urdf REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

# ===========================================================================
# Open3D 自动检测 (4 层 fallback)
# 优先级: 环境变量 Open3D_DIR > ~/open3d_install (C++ 预编译包约定路径)
#         > pip 包自动探测 (仅 0.18 及以下) > CMake 默认搜索
#
# pip open3d >= 0.19 不再包含 C++ CMake 配置文件，需单独下载 devel 包:
#   wget https://github.com/isl-org/Open3D/releases/download/v0.19.0/open3d-devel-linux-x86_64-cxx11-abi-0.19.0.tar.xz
#   mkdir -p ~/open3d_install
#   tar xf open3d-devel-linux-x86_64-cxx11-abi-0.19.0.tar.xz -C ~/open3d_install --strip-components=1
# ===========================================================================
if(DEFINED ENV{Open3D_DIR})
    # 方式 1: 用户通过环境变量显式指定
    set(Open3D_DIR "$ENV{Open3D_DIR}")
    message(STATUS "Open3D: using Open3D_DIR from environment: ${Open3D_DIR}")

elseif(EXISTS "$ENV{HOME}/open3d_install/lib/cmake/Open3D/Open3DConfig.cmake")
    # 方式 2: 约定路径 ~/open3d_install (C++ 预编译包解压到此)
    set(Open3D_DIR "$ENV{HOME}/open3d_install/lib/cmake/Open3D")
    message(STATUS "Open3D: using conventional path ~/open3d_install")

else()
    # 方式 3: 自动探测 pip 安装的 Open3D (仅 0.18 及以下版本有 CMake 文件)
    set(_open3d_script
        "import open3d, os, sys, glob;"
        "pkg_dir=os.path.dirname(open3d.__file__);"
        "print('PYTHON:'+sys.executable);"
        "print('PKGDIR:'+pkg_dir);"
        "cm=os.path.join(pkg_dir,'lib','cmake','Open3D');"
        "print('CMAKEDIR:'+cm);"
        "print('EXISTS:'+str(os.path.isfile(os.path.join(cm,'Open3DConfig.cmake'))));"
        "if not os.path.isfile(os.path.join(cm,'Open3DConfig.cmake')):"
        "  hits=glob.glob(os.path.join(pkg_dir,'**','Open3DConfig.cmake'),recursive=True);"
        "  print('GLOB_HITS:'+str(hits));"
        "  if hits: print('CMAKE_FALLBACK:'+os.path.dirname(hits[0]));"
    )

    execute_process(
        COMMAND python3 -c ${_open3d_script}
        OUTPUT_VARIABLE _open3d_diag
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_VARIABLE _open3d_err
        ERROR_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE _open3d_import_result
    )

    message(STATUS "Open3D diagnostic (exit=${_open3d_import_result}):\n${_open3d_diag}")
    if(_open3d_err AND NOT _open3d_err STREQUAL "")
        message(STATUS "Open3D stderr: ${_open3d_err}")
    endif()

    set(_open3d_cmake_dir "")
    if(_open3d_import_result EQUAL 0)
        string(REGEX MATCH "CMAKE_FALLBACK:([^\n]+)" _match "${_open3d_diag}")
        if(CMAKE_MATCH_1)
            set(_open3d_cmake_dir "${CMAKE_MATCH_1}")
        else()
            string(REGEX MATCH "EXISTS:True" _exists "${_open3d_diag}")
            if(_exists)
                string(REGEX MATCH "CMAKEDIR:([^\n]+)" _match "${_open3d_diag}")
                if(CMAKE_MATCH_1)
                    set(_open3d_cmake_dir "${CMAKE_MATCH_1}")
                endif()
            endif()
        endif()
    endif()

    if(_open3d_cmake_dir)
        set(Open3D_DIR "${_open3d_cmake_dir}")
        message(STATUS "Open3D: auto-detected from pip package: ${Open3D_DIR}")
    else()
        message(FATAL_ERROR
            "Open3D: not found!\n"
            "  pip open3d 0.19+ 不含 C++ CMake 文件，需下载 C++ 开发包:\n"
            "    wget https://github.com/isl-org/Open3D/releases/download/v0.19.0/open3d-devel-linux-x86_64-cxx11-abi-0.19.0.tar.xz\n"
            "    mkdir -p ~/open3d_install\n"
            "    tar xf open3d-devel-linux-x86_64-cxx11-abi-0.19.0.tar.xz -C ~/open3d_install --strip-components=1\n"
            "  然后重新构建。或手动指定: export Open3D_DIR=/path/to/lib/cmake/Open3D"
        )
    endif()
endif()

find_package(Open3D REQUIRED)
message(STATUS "Open3D: found at ${Open3D_DIR}")

# Fix Open3D libraries - remove problematic .so files
# In Ubuntu 22.04+, some libraries (librt, libc++) have different paths or are merged
# Open3D's CMake config files may have hardcoded paths that don't exist

# After finding Open3D, patch ALL its imported targets
# We need to patch multiple targets that may have these dependencies
foreach(target Open3D::Open3D Open3D::3rdparty_filament Open3D::3rdparty_glfw Open3D::3rdparty_rply Open3D::3rdparty_glew)
        if(TARGET ${target})
                get_target_property(LINK_LIBS ${target} INTERFACE_LINK_LIBRARIES)

                if(LINK_LIBS)
                        # Create a new list with problematic libraries removed
                        set(FILTERED_LIBS "")

                        foreach(lib ${LINK_LIBS})
                                # Skip problematic libraries
                                if(NOT "${lib}" MATCHES ".*librt\\.so.*" AND
                                        NOT "${lib}" MATCHES ".*libc\\+\\+\\.so.*" AND
                                        NOT "${lib}" MATCHES ".*libc\\+\\+abi\\.so.*")
                                        list(APPEND FILTERED_LIBS ${lib})
                                endif()
                        endforeach()

                        # Set the filtered list back
                        set_target_properties(${target} PROPERTIES INTERFACE_LINK_LIBRARIES "${FILTERED_LIBS}")
                endif()
        endif()
endforeach()

# Also filter the Open3D_LIBRARIES variable
if(Open3D_LIBRARIES)
        set(FILTERED_O3D_LIBS "")

        foreach(lib ${Open3D_LIBRARIES})
                if(NOT "${lib}" MATCHES ".*librt\\.so.*" AND
                        NOT "${lib}" MATCHES ".*libc\\+\\+\\.so.*" AND
                        NOT "${lib}" MATCHES ".*libc\\+\\+abi\\.so.*")
                        list(APPEND FILTERED_O3D_LIBS ${lib})
                endif()
        endforeach()

        set(Open3D_LIBRARIES ${FILTERED_O3D_LIBS})
endif()

include_directories(
        include
        ${Open3D_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${YAMLCPP_INCLUDE_DIRS}
)

# Build open3d_registration library
add_library(open3d_registration src/open3d_registration/open3d_registration.cpp)
target_link_libraries(open3d_registration ${Open3D_LIBRARIES})
ament_target_dependencies(open3d_registration
        rclcpp
        sensor_msgs
        geometry_msgs
)

# Build open3d_conversions library
add_library(open3d_conversions src/open3d_conversions/open3d_conversions.cpp)
target_link_libraries(open3d_conversions ${Open3D_LIBRARIES})
ament_target_dependencies(open3d_conversions
        rclcpp
        sensor_msgs
        geometry_msgs
        std_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        tf2_eigen
        urdf
)

# Build global_localization_node
add_executable(global_localization_node src/global_localization.cpp)
target_link_libraries(global_localization_node
        ${Open3D_LIBRARIES}
        open3d_registration
        open3d_conversions
)
ament_target_dependencies(global_localization_node
        rclcpp
        std_msgs
        geometry_msgs
        nav_msgs
        sensor_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        tf2_eigen
        pcl_ros
        pcl_conversions
)

# Install targets
install(TARGETS
        open3d_registration
        open3d_conversions
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS
        global_localization_node

        DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
        launch
        config
        rviz_cfg
        maps
        DESTINATION share/${PROJECT_NAME}/
)

# Install include directory
install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
endif()

ament_package()
