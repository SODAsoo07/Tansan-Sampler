cmake_minimum_required(VERSION 3.16)
project(TansanSampler VERSION 0.2.0 LANGUAGES CXX)

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
set(RESAMPLER_BASENAME "Tansan-Sampler")
set(RESAMPLER_PRESET_FILE "tansanSampler.txt")
set_target_properties(resamp PROPERTIES OUTPUT_NAME "${RESAMPLER_BASENAME}")
add_custom_command(TARGET resamp POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          "${CMAKE_SOURCE_DIR}/${RESAMPLER_PRESET_FILE}"
          "$<TARGET_FILE_DIR:resamp>/${RESAMPLER_PRESET_FILE}"
  COMMENT "Copying ${RESAMPLER_PRESET_FILE} next to ${RESAMPLER_BASENAME}.exe")

# ── Wavtool Probe (M0: 호출 인자 캡처 + 최소 누적 동작) ───────────────
set(WAVTOOL_PROBE_SOURCES
  src/wavtool/wavtool_probe.cpp
  src/io/wav_reader.cpp
  src/io/wav_writer.cpp
)

add_executable(wavtool_probe ${WAVTOOL_PROBE_SOURCES})
target_include_directories(wavtool_probe PRIVATE src)

# ── Resamp Wavtool (M1~M3: 경계 품질 개선용 누적 합성기) ───────────────
set(WAVTOOL_SOURCES
  src/wavtool/wavtool.cpp
  src/io/wav_reader.cpp
  src/io/wav_writer.cpp
)

add_executable(wavtool ${WAVTOOL_SOURCES})
target_include_directories(wavtool PRIVATE src)
set_target_properties(wavtool PROPERTIES OUTPUT_NAME "V_wavtool")
if(MSVC)
  target_compile_options(wavtool PRIVATE /Oi /Gy /Gw /GL)
  target_link_options(wavtool PRIVATE /LTCG /OPT:REF /OPT:ICF)
endif()

# ── OpenUtau 테스트용 자동 배포 ─────────────────────────
option(RESAMP_AUTO_COPY_TO_OPENUTAU "Copy built resamp.exe to OpenUtau Resamplers folder." ON)
set(OPENUTAU_RESAMPLERS_DIR
  "C:/Users/oyh57/SODAsoo1/VocalSynth/OpenUtauV-win-x64/Resamplers"
  CACHE PATH
  "OpenUtau Resamplers directory for post-build deployment.")
set(OPENUTAU_WAVTOOLS_DIR
  "C:/Users/oyh57/SODAsoo1/VocalSynth/OpenUtauV-win-x64/Wavtools"
  CACHE PATH
  "OpenUtau Wavtools directory for post-build deployment.")

if(WIN32 AND RESAMP_AUTO_COPY_TO_OPENUTAU)
  add_custom_command(TARGET resamp POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${OPENUTAU_RESAMPLERS_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:resamp>"
            "${OPENUTAU_RESAMPLERS_DIR}/${RESAMPLER_BASENAME}.exe"
    COMMAND ${CMAKE_COMMAND} -E copy
            "${CMAKE_SOURCE_DIR}/${RESAMPLER_BASENAME}.yaml"
            "${OPENUTAU_RESAMPLERS_DIR}/${RESAMPLER_BASENAME}.yaml"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_SOURCE_DIR}/${RESAMPLER_PRESET_FILE}"
            "${OPENUTAU_RESAMPLERS_DIR}/${RESAMPLER_PRESET_FILE}"
    COMMENT "Deploying ${RESAMPLER_BASENAME}.exe/${RESAMPLER_BASENAME}.yaml/${RESAMPLER_PRESET_FILE} to ${OPENUTAU_RESAMPLERS_DIR}")
endif()

option(WAVTOOL_AUTO_COPY_TO_OPENUTAU "Copy built wavtool_probe.exe to OpenUtau Resamplers folder." ON)
if(WIN32 AND WAVTOOL_AUTO_COPY_TO_OPENUTAU)
  add_custom_command(TARGET wavtool_probe POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${OPENUTAU_RESAMPLERS_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:wavtool_probe>"
            "${OPENUTAU_RESAMPLERS_DIR}/wavtool_probe.exe"
    COMMENT "Deploying wavtool_probe.exe to ${OPENUTAU_RESAMPLERS_DIR}")
  add_custom_command(TARGET wavtool POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${OPENUTAU_WAVTOOLS_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:wavtool>"
            "${OPENUTAU_WAVTOOLS_DIR}/V_wavtool.exe"
    COMMENT "Deploying V_wavtool.exe to ${OPENUTAU_WAVTOOLS_DIR}")
endif()

if(WIN32)
  target_compile_definitions(resamp PRIVATE
    _UNICODE UNICODE NOMINMAX WIN32_LEAN_AND_MEAN
  )
  target_compile_definitions(wavtool_probe PRIVATE
    _UNICODE UNICODE NOMINMAX WIN32_LEAN_AND_MEAN
  )
  target_compile_definitions(wavtool PRIVATE
    _UNICODE UNICODE NOMINMAX WIN32_LEAN_AND_MEAN
  )
endif()

install(TARGETS resamp wavtool_probe wavtool RUNTIME DESTINATION bin)
install(FILES "${RESAMPLER_PRESET_FILE}" DESTINATION bin)
