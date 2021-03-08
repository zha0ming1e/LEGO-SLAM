#ifndef LEGOSLAM_VIEWER_H
#define LEGOSLAM_VIEWER_H

#include <thread>
#include <pangolin/pangolin.h>

#include "legoslam/common_include.h"
#include "legoslam/frame.h"
#include "legoslam/map.h"

namespace legoslam {

    /*
     * visualization
     */
    class Viewer {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Viewer> Ptr;

        Viewer();

        // set the map
        void SetMap(Map::Ptr map) { map_ = map; }
        // set follow frame
        void SetFollowFrame(int follow_frame) {
            if (follow_frame) follow_frame_ = true;
            else follow_frame_ = false;
        }
        void SetFollowFrame(bool follow_frame) { follow_frame_ = follow_frame; }
        // close the viewer
        void Close();
        // add the current frame
        void AddCurrentFrame(Frame::Ptr current_frame);
        // update the map
        void UpdateMap();

    private:
        void ThreadLoop();
        void DrawFrame(const Frame::Ptr &frame, const float *color);
        void DrawMapPointsFramesTraj(bool drawTraj=true);
        void FollowCurrentFrame(pangolin::OpenGlRenderState &vis_camera, bool follow= true);
        // plot the features in current frame into an image
        cv::Mat PlotFrameImage();

        // data member
        std::mutex viewer_data_mutex_;
        Frame::Ptr current_frame_ = nullptr;
        Map::Ptr map_ = nullptr;

        std::thread viewer_thread_;
        bool viewer_running_ = true;

        //std::unordered_map<unsigned long, Frame::Ptr> keyframes_to_display_;
        std::map<unsigned long, Frame::Ptr> keyframes_to_display_;
        std::unordered_map<unsigned long, MapPoint::Ptr> landmarks_to_display_;
        bool map_updated_ = false;
        bool follow_frame_ = false;
    };
}
#endif  // LEGOSLAM_VIEWER_H
