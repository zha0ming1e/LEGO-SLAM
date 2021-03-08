# app
# feature method
add_executable(legoslam_kitti_feature legoslam_kitti_feature.cpp)
target_link_libraries(legoslam_kitti_feature legoslam ${THIRD_PARTY_LIBS})

# direct method
#add_executable(legoslam_kitti_direct legoslam_kitti_direct.cpp)
#target_link_libraries(legoslam_kitti_direct legoslam ${THIRD_PARTY_LIBS})
