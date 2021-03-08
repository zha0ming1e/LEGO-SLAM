#ifndef LEGOSLAM_BACKEND_H
#define LEGOSLAM_BACKEND_H

#include "legoslam/common_include.h"
#include "legoslam/frame.h"
#include "legoslam/map.h"

namespace legoslam {

    class Map;

    /*
     * backend
     * there is a independent thread, will launch optimization when map update
     * map update will be triggered by frontend
     */
    class Backend {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Backend> Ptr;

        // constructor will launch optimization thread and hang it up
        Backend();

        // set left and right cameras, in- and extrinsics
        void SetCameras(Camera::Ptr left, Camera::Ptr right) {
            cam_left_ = left;
            cam_right_ = right;
        }
        // set map
        void SetMap(Map::Ptr map) { map_ = map; }
        // update map and launch optimization
        void UpdateMap();
        // hang the backend thread
        void Hang();
        // restart the backend thread
        void Restart();
        // stop the backend thread
        void Stop();

    private:
        // backend thread
        void BackendLoop();
        // optimize the given keyframes and landmarks
        void Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks);

        // data members
        // map
        Map::Ptr map_ = nullptr;
        // backend thread
        std::thread backend_thread_;
        // mutex
        std::mutex data_mutex_;
        // conditional variable
        std::condition_variable map_update_;
        // atomic
        std::atomic<bool> backend_running_{};
        // camera: left and right
        Camera::Ptr cam_left_= nullptr, cam_right_ = nullptr;
    };
}
#endif  // LEGOSLAM_BACKEND_H
