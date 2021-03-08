#include "legoslam/viewer.h"
#include "legoslam/feature.h"
#include "legoslam/frame.h"

#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>

namespace legoslam {

    Viewer::Viewer() {
        viewer_thread_ = std::thread(std::bind(&Viewer::ThreadLoop, this));
    }

    void Viewer::Close() {
        viewer_running_ = false;
        viewer_thread_.join();
    }

    void Viewer::AddCurrentFrame(Frame::Ptr current_frame) {
        std::unique_lock<std::mutex> lock(viewer_data_mutex_);
        current_frame_ = current_frame;
    }

    void Viewer::UpdateMap() {
        std::unique_lock<std::mutex> lock(viewer_data_mutex_);
        assert(map_ != nullptr);
        // 1. just visualize the ACTIVE keyframes and landmarks
        //keyframes_to_display_ = map_->GetActiveKeyFrames();
        //landmarks_to_display_ = map_->GetActiveMapPoints();

        // 2. uncomment the next 2 lines to visualize ALL keyframes and landmarks
        keyframes_to_display_ = map_->GetAllKeyFrames();
        landmarks_to_display_ = map_->GetAllMapPoints();

        map_updated_ = true;
    }

    void Viewer::ThreadLoop() {
        pangolin::CreateWindowAndBind("LEGO-SLAM", 1024, 768);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::OpenGlRenderState vis_camera(
                pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
                pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

        // add named OpenGL viewport to window and provide 3D Handler
        pangolin::View& vis_display =
                pangolin::CreateDisplay()
                        .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                        .SetHandler(new pangolin::Handler3D(vis_camera));

        const float red[3] = {1.0, 0, 0};

        while (!pangolin::ShouldQuit() && viewer_running_) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            vis_display.Activate(vis_camera);

            // the scope of mutex thread lock
            {
                std::unique_lock<std::mutex> lock(viewer_data_mutex_);
                // draw frames
                if (current_frame_) {
                    DrawFrame(current_frame_, red);
                    // 1. follow current frame
                    //FollowCurrentFrame(vis_camera, true);
                    // 2. dont follow current frame
                    //FollowCurrentFrame(vis_camera, false);
                    // 3. follow_frame_ from the config file
                    FollowCurrentFrame(vis_camera, follow_frame_);

                    cv::Mat img = PlotFrameImage();
                    cv::imshow("Current Image", img);
                    // according to KITTI dataset file: times.txt, the average time duration between 2 images is about 0.10365 s (103.65 ms)
                    // so set delay to 104 ms as default
                    // or, for faster data input, delay is set to 33 ms
                    cv::waitKey(120);
                    // cv::waitKey() -> without a given argument, you can control the input data rate,
                    // just like you are the driver of the car!
                    // cv::waitKey();
                }
                // draw map points and frames
                if (map_) {
                    // 1. draw map points and frames with trajectory
                    DrawMapPointsFramesTraj(true);
                    // 2. draw map points and frames without trajectory
                    // DrawMapPointsAndFrames(false);
                }
            }
            pangolin::FinishFrame();
            usleep(5000);
        }

        LOG(INFO) << "Stop viewer. ";
    }

    cv::Mat Viewer::PlotFrameImage() {
        cv::Mat img_out;
        // OpenCV3 and OpenCV4
        cv::cvtColor(current_frame_->left_img_, img_out, cv::COLOR_GRAY2BGR);
        for (const auto &feat : current_frame_->features_left_) {
            if (feat->map_point_.lock())
                cv::circle(img_out, feat->position_.pt, 1, cv::Scalar(0, 0, 255), 2);
        }
        return img_out;
    }

    void Viewer::FollowCurrentFrame(pangolin::OpenGlRenderState &vis_camera, bool follow) {
        SE3 Twc = current_frame_->Pose().inverse();
        pangolin::OpenGlMatrix m(Twc.matrix());
        vis_camera.Follow(m, follow);
    }

    void Viewer::DrawFrame(const Frame::Ptr &frame, const float *color) {
        SE3 Twc = frame->Pose().inverse();
        const float sz = 1.0;
        const int line_width = 2.0;
        const float fx = 400;
        const float fy = 400;
        const float cx = 512;
        const float cy = 384;
        const float width = 1080;
        const float height = 768;

        glPushMatrix();

        Sophus::Matrix4f m = Twc.matrix().template cast<float>();
        glMultMatrixf((GLfloat *)m.data());

        if (color == nullptr)
            glColor3f(1, 0, 0);
        else
            glColor3f(color[0], color[1], color[2]);

        glLineWidth(line_width);
        glBegin(GL_LINES);
        // focal lines of 4 vertex
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
        // camera box
        // 1
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        // 2
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        // 3
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        // 4
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glEnd();
        glPopMatrix();
    }

    void Viewer::DrawMapPointsFramesTraj(bool drawTraj) {
        const float green[3] = {0.0, 1.0, 0.0};
        const float dark_green[3] = {0.0, 0.5, 0.0};
        // frames and trajectory
        for (auto &kf : keyframes_to_display_) {
            if (kf.second->is_active_keyframe_)
                DrawFrame(kf.second, green);
            else
                DrawFrame(kf.second, dark_green);
        }

        // trajectory
        if (drawTraj) {
            glColor3f(0.0, 0.0, 1.0);
            glLineWidth(2.0);
            glBegin(GL_LINE_STRIP);
            for (auto &kf : keyframes_to_display_) {
                Vec3 t = kf.second->Pose().inverse().translation();
                glVertex3d(t[0], t[1], t[2]);
            }
            glEnd();
        }

        // map points
        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &landmark : landmarks_to_display_) {
            auto pos = landmark.second->Pos();
            if (landmark.second->is_active_landmark_)
                glColor3f(1.0, 0.0, 0.0);
            else
                glColor3f(0.0, 0.0, 0.0);
            glVertex3d(pos[0], pos[1], pos[2]);
        }
        glEnd();
    }
}
