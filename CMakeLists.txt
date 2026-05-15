cmake_minimum_required(VERSION 3.21)

# Project Name
project(avbd_demo3d)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Statically link the MSVC C++ runtime. The `avbd` shared library is loaded into
# host processes (Blender) that ship their own, possibly older, runtime DLLs;
# static linking makes avbd.dll self-contained and avoids ABI mismatches that
# otherwise crash inside MSVCP140.dll. The whole project uses /MT for consistency.
if(MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    set(SDL_FORCE_STATIC_VCRT ON CACHE BOOL "" FORCE)
endif()

# ---------------------------------------------------------------------------
# avbd_core : headless physics core (static library)
#
# This is the AVBD solver with no windowing/OpenGL/UI dependencies. It is linked
# both by the demo executable and by the "avbd" shared library (C API for Python).
# ---------------------------------------------------------------------------
set(AVBD_CORE_SOURCES
    source/solver.cpp
    source/rigid.cpp
    source/joint.cpp
    source/spring.cpp
    source/manifold.cpp
    source/collide.cpp
    source/convex.cpp
    source/force.cpp
    source/parallel.cpp
)
set(AVBD_CORE_HEADERS
    source/solver.h
    source/maths.h
    source/parallel.h
)

add_library(avbd_core STATIC ${AVBD_CORE_SOURCES} ${AVBD_CORE_HEADERS})
target_include_directories(avbd_core PUBLIC source)
# Position-independent code so avbd_core can be linked into the shared library.
set_target_properties(avbd_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The solver thread pool needs the platform threading library (pthread on Linux).
find_package(Threads REQUIRED)
target_link_libraries(avbd_core PUBLIC Threads::Threads)

# ---------------------------------------------------------------------------
# avbd : flat C API shared library (loaded from Python via ctypes)
# ---------------------------------------------------------------------------
add_library(avbd SHARED source/avbd_api.cpp source/avbd_api.h)
target_link_libraries(avbd PRIVATE avbd_core)
set_target_properties(avbd PROPERTIES
    OUTPUT_NAME "avbd"
    C_VISIBILITY_PRESET hidden
    CXX_VISIBILITY_PRESET hidden)

# ---------------------------------------------------------------------------
# avbd_test : headless smoke tests for the core (convex collision, OBB path)
# ---------------------------------------------------------------------------
add_executable(avbd_test tests/test_convex.cpp)
target_link_libraries(avbd_test PRIVATE avbd_core)

# ---------------------------------------------------------------------------
# avbd_demo3d : interactive demo / visual test bed (executable)
#
# Built only when the SDL/ImGui submodules are checked out. The headless
# avbd_core / avbd library targets do not need them, so the project still
# configures for library-only work via `git clone` without submodules.
# ---------------------------------------------------------------------------
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/external/SDL/CMakeLists.txt")
    add_executable(${PROJECT_NAME} source/main.cpp source/scenes.h)
    target_link_libraries(${PROJECT_NAME} PRIVATE avbd_core)

    # Include SDL2 first (ImGui needs it)
    add_subdirectory(external/SDL)

    # Add ImGui as a separate project
    set(IMGUI_PROJECT_NAME "imgui")

    file(GLOB IMGUI_SRC
        external/imgui/*.cpp
        external/imgui/backends/imgui_impl_sdl2.cpp
        external/imgui/backends/imgui_impl_opengl3.cpp
    )

    add_library(${IMGUI_PROJECT_NAME} STATIC ${IMGUI_SRC})

    # Set include directories for ImGui
    target_include_directories(${IMGUI_PROJECT_NAME} PUBLIC
        external/imgui
        external/imgui/backends
        external/SDL/include
    )

    # Link ImGui with SDL2
    target_link_libraries(${IMGUI_PROJECT_NAME} PUBLIC SDL2::SDL2 SDL2::SDL2main)

    # Link the main project with SDL2 and ImGui
    target_include_directories(${PROJECT_NAME} PRIVATE external/SDL/include)
    target_link_libraries(${PROJECT_NAME} PRIVATE SDL2::SDL2 SDL2::SDL2main ${IMGUI_PROJECT_NAME})

    if(CMAKE_SYSTEM_NAME STREQUAL "Emscripten")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -s USE_SDL=2 -s USE_WEBGL2=1 -s ALLOW_MEMORY_GROWTH=1 -s SINGLE_FILE=1 -s LEGACY_GL_EMULATION=1 --shell-file ../source/shell.html")
        set(CMAKE_EXECUTABLE_SUFFIX ".html")
    else()
        # Find and Link OpenGL
        find_package(OpenGL REQUIRED)
        target_link_libraries(${PROJECT_NAME} PRIVATE OpenGL::GL)
    endif()

    # Copy SDL2.dll to the output folder on Windows
    if(WIN32)
        add_custom_command(
            TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:SDL2::SDL2>    # Path to the built SDL2.dll
                $<TARGET_FILE_DIR:${PROJECT_NAME}> # Output directory of the executable
        )
    endif()
else()
    message(STATUS "avbd: SDL/ImGui submodules not found - skipping avbd_demo3d demo target. "
                   "Run 'git submodule update --init --recursive' to enable it.")
endif()
