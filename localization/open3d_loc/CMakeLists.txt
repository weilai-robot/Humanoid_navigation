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
# Open3D 自动检测 (支持 pip install open3d 或 CMake 原生安装)
# 优先级: 环境变量 Open3D_DIR > pip 包自动探测 > CMake 默认搜索
#
# 用法:
#   方式 A (推荐): pip install open3d  →  自动探测
#   方式 B: export Open3D_DIR=/path/to/open3d/lib/cmake/Open3D  →  手动指定
# ===========================================================================
if(DEFINED ENV{Open3D_DIR})
    # 方式 B: 用户通过环境变量显式指定
    set(Open3D_DIR "$ENV{Open3D_DIR}")
    message(STATUS "Open3D: using Open3D_DIR from environment: ${Open3D_DIR}")

else()
    # 方式 A: 自动探测 pip 安装的 Open3D
    # 不用 find_package(Python3) — conda+ROS2 环境下它可能找到错误的 Python。
    # 直接用 PATH 中的 python3 (构建时已激活 conda env, 即装了 open3d 的那个)。
    set(_open3d_script
        "import open3d, os, sys, glob;"
        "pkg_dir=os.path.dirname(open3d.__file__);"
        "print('PYTHON:'+sys.executable);"
        "print('PKGDIR:'+pkg_dir);"
        # 标准 pip 包路径
        "cm=os.path.join(pkg_dir,'lib','cmake','Open3D');"
        "print('CMAKEDIR:'+cm);"
        "print('EXISTS:'+str(os.path.isfile(os.path.join(cm,'Open3DConfig.cmake'))));"
        # 如果标准路径不存在, 用 glob 搜整个包目录
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

    # 打印完整诊断 (帮助排查)
    message(STATUS "Open3D diagnostic (exit=${_open3d_import_result}):\n${_open3d_diag}")
    if(_open3d_err AND NOT _open3d_err STREQUAL "")
        message(STATUS "Open3D stderr: ${_open3d_err}")
    endif()

    # 从诊断输出中提取 CMake 目录
    set(_open3d_cmake_dir "")
    if(_open3d_import_result EQUAL 0)
        # 优先取 CMAKE_FALLBACK (glob 搜到的)
        string(REGEX MATCH "CMAKE_FALLBACK:([^\n]+)" _match "${_open3d_diag}")
        if(CMAKE_MATCH_1)
            set(_open3d_cmake_dir "${CMAKE_MATCH_1}")
        else()
            # 取标准路径, 但要检查 EXISTS:true
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
        message(WARNING
            "Open3D: auto-detection failed.\n"
            "  Possible causes:\n"
            "    1. pip open3d not installed in the build-time Python (try: python3 -c 'import open3d')\n"
            "    2. pip open3d version has no C++ CMake files (C++ dev package needed)\n"
            "  Fix: export Open3D_DIR=/path/to/open3d/lib/cmake/Open3D before building\n"
            "       or: pip install open3d==0.18.0 (ensures CMake configs included)"
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
