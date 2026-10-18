cmake_minimum_required(VERSION 3.22)

project(TwinModeDist VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_OSX_DEPLOYMENT_TARGET "11.0" CACHE STRING "")

# Use a local JUCE checkout with -DJUCE_DIR=/path/to/JUCE, otherwise fetch it.
if(DEFINED JUCE_DIR)
    add_subdirectory(${JUCE_DIR} juce)
else()
    include(FetchContent)
    FetchContent_Declare(JUCE
        GIT_REPOSITORY https://github.com/juce-framework/JUCE.git
        GIT_TAG        9.0.4
        GIT_SHALLOW    ON)
    FetchContent_MakeAvailable(JUCE)
endif()

# Formats this platform can't build (AU off macOS) are skipped by JUCE.
set(TMD_FORMATS VST3 AU Standalone CACHE STRING "Plugin formats to build")

juce_add_plugin(TwinModeDist
    COMPANY_NAME                "YourName"
    BUNDLE_ID                   com.yourname.twinmodedist
    PLUGIN_MANUFACTURER_CODE    Ynme
    PLUGIN_CODE                 Tmd2
    FORMATS                     ${TMD_FORMATS}
    PRODUCT_NAME                "Twin Mode Dist"
    IS_SYNTH                    FALSE
    NEEDS_MIDI_INPUT            FALSE
    NEEDS_MIDI_OUTPUT           FALSE
    IS_MIDI_EFFECT              FALSE
    EDITOR_WANTS_KEYBOARD_FOCUS FALSE
    COPY_PLUGIN_AFTER_BUILD     FALSE
    VST3_CATEGORIES             Fx Distortion
    AU_MAIN_TYPE                kAudioUnitType_Effect)

target_sources(TwinModeDist PRIVATE
    Source/PluginProcessor.cpp)

target_include_directories(TwinModeDist PRIVATE Source)

target_compile_definitions(TwinModeDist PUBLIC
    JUCE_WEB_BROWSER=0
    JUCE_USE_CURL=0
    JUCE_VST3_CAN_REPLACE_VST2=0)

target_link_libraries(TwinModeDist
    PRIVATE
        juce::juce_audio_utils
        juce::juce_dsp
    PUBLIC
        juce::juce_recommended_config_flags
        juce::juce_recommended_lto_flags
        juce::juce_recommended_warning_flags)

# Offline renderer used by tests/analyze.py (no JUCE needed)
add_executable(ds2_render EXCLUDE_FROM_ALL tests/render.cpp)
