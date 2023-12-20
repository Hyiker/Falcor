find_package(Boost COMPONENTS system REQUIRED)
link_directories(${Boost_LIBRARY_DIRS})

add_falcor_executable(Mogwai)

target_sources(Mogwai PRIVATE
    AppData.cpp
    AppData.h
    WebServer.cpp
    WebServer.h
    Mogwai.cpp
    Mogwai.h
    MogwaiScripting.cpp
    MogwaiSettings.cpp
    MogwaiSettings.h

    Extensions/Capture/CaptureTrigger.cpp
    Extensions/Capture/CaptureTrigger.h
    Extensions/Capture/FrameCapture.cpp
    Extensions/Capture/FrameCapture.h
    Extensions/Profiler/TimingCapture.cpp
    Extensions/Profiler/TimingCapture.h
)


target_include_directories(Mogwai PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(Mogwai PRIVATE ${Boost_SYSTEM_LIBRARY})

target_link_libraries(Mogwai PRIVATE args zlib)

target_source_group(Mogwai "/")
