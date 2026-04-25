cmake_minimum_required(VERSION 3.16)
project(Resamp VERSION 0.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ── 컴파일러 플래그 ─────────────────────────────────────
if(MSVC)
  add_compile_options(/O2 /W3 /fp:fast /arch:SSE2)
  add_compile_options(/utf-8)
else()
  add_compile_options(-O3 -Wall -Wextra -ffast-math -msse2 -mfpmath=sse)
  link_libraries(m)
endif()

# ── WORLD 라이브러리 (vocoder) ─────────────────────────
set(WORLD_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(WORLD_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
add_subdirectory(external/World)

# ── 소스 목록 ──────────────────────────────────────────
set(SOURCES
  src/main.cpp
  src/args/arg_parser.cpp
  src/io/wav_reader.cpp
  src/io/wav_writer.cpp
  src/io/frq_reader.cpp
  src/synth/pitch_mapper.cpp
  src/synth/world_synth.cpp
  src/flags/flag_parser.cpp
  src/post/volume.cpp
  src/util/math_util.cpp
)

add_executable(resamp ${SOURCES})
target_include_directories(resamp PRIVATE src)
target_link_libraries(resamp PRIVATE world)

if(WIN32)
  target_compile_definitions(resamp PRIVATE
    _UNICODE UNICODE NOMINMAX WIN32_LEAN_AND_MEAN
  )
endif()

install(TARGETS resamp RUNTIME DESTINATION bin)
