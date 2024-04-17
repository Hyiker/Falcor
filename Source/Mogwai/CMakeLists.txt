add_falcor_executable(Mogwai)

target_sources(Mogwai PRIVATE
    AppData.cpp
    AppData.h
    Mogwai.cpp
    Mogwai.h
    MogwaiScripting.cpp
    MogwaiSettings.cpp
    MogwaiSettings.h
    Network.h
    Network.cpp

    Extensions/Capture/CaptureTrigger.cpp
    Extensions/Capture/CaptureTrigger.h
    Extensions/Capture/FrameCapture.cpp
    Extensions/Capture/FrameCapture.h
    Extensions/Profiler/TimingCapture.cpp
    Extensions/Profiler/TimingCapture.h
)

target_link_libraries(Mogwai PRIVATE args websockets ${LIBWEBSOCKETS_DEP_LIBS})

target_source_group(Mogwai "/")
