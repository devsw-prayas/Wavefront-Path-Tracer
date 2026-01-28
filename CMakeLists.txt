cmake_minimum_required(VERSION 3.20)
project(WavefrontPT)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/$<CONFIG>)
link_directories(${CMAKE_SOURCE_DIR}/bin/$<CONFIG>)

file(GLOB_RECURSE WAVEFRONT_HEADERS CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/src/Public/*.h")
file(GLOB_RECURSE WAVEFRONT_SOURCE CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/src/Private/*.cpp")
file(GLOB_RECURSE WAVEFRONT_INL CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/src/Public/*.inl")

add_executable(WavefrontPT ${WAVEFRONT_HEADERS} ${WAVEFRONT_SOURCE} ${WAVEFRONT_INL})

target_include_directories(WavefrontPT PRIVATE 
    ${CMAKE_SOURCE_DIR}/src/Public
)

target_precompile_headers(WavefrontPT PRIVATE ${CMAKE_SOURCE_DIR}/src/Public/Core.h)

if (MSVC)
    target_compile_options(WavefrontPT PRIVATE
        /W4
        /permissive-
        /Zc:__cplusplus
        /arch:AVX2
    )
else()
    target_compile_options(WavefrontPT PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -mavx2
    )
endif()
