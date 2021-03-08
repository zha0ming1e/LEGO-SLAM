add_library(legoslam SHARED
        algorithm.cpp
        frame.cpp
        mappoint.cpp
        map.cpp
        camera.cpp
        config.cpp
        feature.cpp

        # lego
#        frontend_lego.cpp
#        backend_lego.cpp

        # g2o
        frontend_g2o.cpp
        backend_g2o.cpp

        viewer.cpp
        visual_odometry.cpp
        dataset.cpp)

target_link_libraries(legoslam ${THIRD_PARTY_LIBS})
