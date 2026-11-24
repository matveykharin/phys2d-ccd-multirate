cmake_minimum_required(VERSION 3.16)
project(phys2d_ccd_multirate LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(PHYS2D_BUILD_APP "Build the SDL2 demo application" ON)

# ---------------------------------------------------------------------
# Физическое ядро (не зависит от SDL2)
# ---------------------------------------------------------------------
add_library(phys_core STATIC
    src/core/Collision.cpp
    src/core/World.cpp
    src/core/Scenes.cpp
)
target_include_directories(phys_core PUBLIC src)

if(MSVC)
    # Исходники в UTF-8 (русские комментарии)
    target_compile_options(phys_core PUBLIC /utf-8 /W3)
else()
    target_compile_options(phys_core PRIVATE -Wall -Wextra)
endif()

# Консольные замеры для отчёта
add_executable(phys_bench src/bench/bench.cpp)
target_link_libraries(phys_bench PRIVATE phys_core)

# ---------------------------------------------------------------------
# Демо-приложение на SDL2
# ---------------------------------------------------------------------
if(PHYS2D_BUILD_APP)
    # 1) Пытаемся найти установленный SDL2 (vcpkg, системный пакет)...
    find_package(SDL2 CONFIG QUIET)
    if(NOT SDL2_FOUND)
        # 2) ...иначе скачиваем исходники SDL2 и собираем статически.
        message(STATUS "SDL2 not found - downloading SDL2 2.30.9 via FetchContent")
        include(FetchContent)
        set(SDL_SHARED OFF CACHE BOOL "" FORCE)
        set(SDL_STATIC ON  CACHE BOOL "" FORCE)
        set(SDL_TEST   OFF CACHE BOOL "" FORCE)
        set(SDL2_DISABLE_SDL2MAIN ON CACHE BOOL "" FORCE)
        FetchContent_Declare(SDL2
            URL https://github.com/libsdl-org/SDL/releases/download/release-2.30.9/SDL2-2.30.9.tar.gz
            URL_HASH SHA256=24b574f71c87a763f50704bbb630cbe38298d544a1f890f099a4696b1d6beba4
        )
        FetchContent_MakeAvailable(SDL2)
    endif()

    add_executable(phys_demo
        src/app/main.cpp
        src/app/Render.cpp
    )
    target_link_libraries(phys_demo PRIVATE phys_core)
    if(TARGET SDL2::SDL2-static)
        target_link_libraries(phys_demo PRIVATE SDL2::SDL2-static)
    elseif(TARGET SDL2-static)
        target_link_libraries(phys_demo PRIVATE SDL2-static)
    else()
        target_link_libraries(phys_demo PRIVATE SDL2::SDL2)
    endif()

    # Visual Studio: запускать из папки сборки, рабочая папка = папка exe
    set_property(TARGET phys_demo PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:phys_demo>")
    if(MSVC)
        set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT phys_demo)
    endif()
endif()
