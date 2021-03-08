#include <memory>
#include <opencv2/opencv.hpp>

#include "legoslam/algorithm.h"
#include "legoslam/backend.h"
#include "legoslam/config.h"
#include "legoslam/feature.h"
#include "legoslam/frontend.h"
#include "legoslam/g2o_types.h"
#include "legoslam/map.h"
#include "legoslam/viewer.h"

namespace legoslam {

    Frontend::Frontend() {
        gftt_ = cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
        num_features_init_ = Config::Get<int>("num_features_init");
        num_features_ = Config::Get<int>("num_features");
        // limits of point depth from triangulation
        stereo_depth_superior_limit_ = Config::Get<double>("stereo_depth_superior_limit");
        if (stereo_depth_superior_limit_ <= 0) stereo_depth_superior_limit_ = 1000;
        stereo_depth_inferior_limit_ = Config::Get<double>("stereo_depth_inferior_limit");
        if (stereo_depth_inferior_limit_ < 0 || stereo_depth_inferior_limit_ >= stereo_depth_superior_limit_)
            stereo_depth_inferior_limit_ = 0;
    }

    bool Frontend::AddFrame(legoslam::Frame::Ptr frame) {
        current_frame_ = frame;

        switch (status_) {
            case FrontendStatus::INITING:
                StereoInit();
                break;
            case FrontendStatus::TRACKING_GOOD:
            case FrontendStatus::TRACKING_BAD:
                Track();
                break;
            case FrontendStatus::LOST:
                Reset();
                break;
        }

        last_frame_ = current_frame_;

        return true;
    }

    bool Frontend::Track() {
        if (last_frame_) {
            current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
        }

        // track last frame and estimate pose of current frame
        //int num_track_last = TrackLastFrame();
        TrackLastFrame();
        tracking_inliers_ = EstimateCurrentPose();

        if (tracking_inliers_ >= num_features_tracking_) {
            // tracking good
            status_ = FrontendStatus::TRACKING_GOOD;
        } else if (tracking_inliers_ >= num_features_tracking_bad_) {
            // tracking bad
            status_ = FrontendStatus::TRACKING_BAD;
        } else {
            // lost
            status_ = FrontendStatus::LOST;
        }

        InsertKeyframe();
        relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

        if (viewer_) viewer_->AddCurrentFrame(current_frame_);

        return true;
    }

    bool Frontend::InsertKeyframe() {
        if (tracking_inliers_ >= num_features_needed_for_keyframe_) {
            // still enough features, don't insert keyframe
            return false;
        }

        // current frame is a new keyframe
        current_frame_->SetKeyFrame();
        map_->InsertKeyFrame(current_frame_);

        LOG(INFO) << "Set frame: " << current_frame_->id_ << " as Keyframe: " << current_frame_->keyframe_id_;

        SetObservationForKeyFrame();
        // detect new features
        DetectFeatures();
        // track in right image
        FindFeaturesInRight();
        // triangulate map points
        TriangulateNewPoints();
        // update backend because we have a new keyframe
        backend_->UpdateMap();

        if (viewer_) viewer_->UpdateMap();

        return true;
    }

    void Frontend::SetObservationForKeyFrame() {
        for (auto &feat : current_frame_->features_left_) {
            auto mp = feat->map_point_.lock();
            if (mp) mp->AddObservation(feat);
        }
    }

    int Frontend::TriangulateNewPoints() {
        std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
        SE3 current_pose_Twc = current_frame_->Pose().inverse();
        int cnt_triangulated_pts = 0;
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
            if (current_frame_->features_left_[i]->map_point_.expired() &&
                current_frame_->features_right_[i] != nullptr) {
                // feature in left image has been not connected any map point,
                // and there is its corresponding feature in right image,
                // then triangulate it
                VecVec3 points{
                        camera_left_->pixel2camera(
                                Vec2(current_frame_->features_left_[i]->position_.pt.x,
                                     current_frame_->features_left_[i]->position_.pt.y)),
                        camera_right_->pixel2camera(
                                Vec2(current_frame_->features_right_[i]->position_.pt.x,
                                     current_frame_->features_right_[i]->position_.pt.y))};
                Vec3 pworld = Vec3::Zero();

                // triangulation of new landmarks
                // some constraints for autonomous driving environments
//                if (triangulation(poses, points, pworld) && pworld[2] > 0) {
                if (triangulation(poses, points, pworld) &&
                    pworld[1] <= 2 &&     // ground constraints: y < 2 m
                    pworld[2] > stereo_depth_inferior_limit_ && pworld[2] <= stereo_depth_superior_limit_     // depth constraints
                    ) {
                    auto new_map_point = MapPoint::CreateNewMappoint();
                    pworld = current_pose_Twc * pworld;
                    new_map_point->SetPos(pworld);
                    new_map_point->AddObservation(
                            current_frame_->features_left_[i]);
                    new_map_point->AddObservation(
                            current_frame_->features_right_[i]);

                    current_frame_->features_left_[i]->map_point_ = new_map_point;
                    current_frame_->features_right_[i]->map_point_ = new_map_point;
                    map_->InsertMapPoint(new_map_point);
                    cnt_triangulated_pts++;
                }
            }
        }
        LOG(INFO) << "New Landmarks: " << cnt_triangulated_pts;

        return cnt_triangulated_pts;
    }

    int Frontend::EstimateCurrentPose() {
        // set up g2o solver
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

        auto solver = new g2o::OptimizationAlgorithmLevenberg(
                g2o::make_unique<BlockSolverType>(
                        g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        // vertex
        auto *vertex_pose = new VertexPose();
        vertex_pose->setId(0);
        vertex_pose->setEstimate(current_frame_->Pose());
        optimizer.addVertex(vertex_pose);

        // K
        Mat33 K = camera_left_->K();
        // edges
        int index = 1;
        std::vector<EdgeProjectionPoseOnly *> edges;
        std::vector<Feature::Ptr> features;
        for (auto &feat_left : current_frame_->features_left_) {
            auto mp = feat_left->map_point_.lock();
            if (mp) {
                features.push_back(feat_left);

                auto *edge = new EdgeProjectionPoseOnly(mp->pos_, K);
                edge->setId(index);
                edge->setVertex(0, vertex_pose);
                edge->setMeasurement(toVec2(feat_left->position_.pt));
                edge->setInformation(Eigen::Matrix2d::Identity());
                edge->setRobustKernel(new g2o::RobustKernelHuber);
                edges.push_back(edge);
                optimizer.addEdge(edge);

                index++;
            }
        }

        // estimate the pose and determine the outliers: chi-squared distribution
        const double chi2_th = 5.991;
        int cnt_outlier = 0;
        for (int iteration = 0; iteration < 4; ++iteration) {
            vertex_pose->setEstimate(current_frame_->Pose());
            optimizer.initializeOptimization();
            optimizer.optimize(10);
            cnt_outlier = 0;

            // count the outliers
            for (size_t i = 0; i < edges.size(); ++i) {
                auto e = edges[i];

                if (features[i]->is_outlier_) {
                    e->computeError();
                }
                if (e->chi2() > chi2_th) {
                    features[i]->is_outlier_ = true;
                    e->setLevel(1);
                    cnt_outlier++;
                } else {
                    features[i]->is_outlier_ = false;
                    e->setLevel(0);
                }

                if (iteration == 2) {
                    e->setRobustKernel(nullptr);
                }
            }
        }

        LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/" << features.size() - cnt_outlier;

        // set pose and outliers
        current_frame_->SetPose(vertex_pose->estimate());

        LOG(INFO) << "Current Pose = \n" << current_frame_->Pose().matrix();

        for (auto &feat : features) {
            if (feat->is_outlier_) {
                feat->map_point_.reset();
                // maybe we can still use it in future
                feat->is_outlier_ = false;
            }
        }

        return features.size() - cnt_outlier;
    }

    int Frontend::TrackLastFrame() {
        // 1. OpenCV optical flow
        //int num_good_pts = TrackLastFrameLKOpticalFlowOpenCV();
        // 2. four layers LK optical flow with Gauss-Newton method
        int num_good_pts = TrackLastFrameLKOpticalFlow4LayerSelf();

        LOG(INFO) << "Find " << num_good_pts << " in the last image. ";

        return num_good_pts;
    }

    bool Frontend::StereoInit() {
        // detect features
        //int num_features_left = DetectFeatures();
        DetectFeatures();
        int num_coor_features = FindFeaturesInRight();
        if (num_coor_features < num_features_init_) {
            return false;
        }

        bool build_map_success = BuildInitMap();
        if (build_map_success) {
            status_ = FrontendStatus::TRACKING_GOOD;
            if (viewer_) {
                viewer_->AddCurrentFrame(current_frame_);
                viewer_->UpdateMap();
            }
            return true;
        }
        return false;
    }

    int Frontend::DetectFeatures() {
        cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
        for (auto &feat : current_frame_->features_left_) {
            cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                          feat->position_.pt + cv::Point2f(10, 10), 0, CV_FILLED);
        }

        std::vector<cv::KeyPoint> keypoints;
        gftt_->detect(current_frame_->left_img_, keypoints, mask);
        int cnt_detected = 0;
        for (auto &kp : keypoints) {
            current_frame_->features_left_.push_back(std::make_shared<Feature>(current_frame_, kp));
            cnt_detected++;
        }

        LOG(INFO) << "Detect: " << cnt_detected << " new features. ";

        return cnt_detected;
    }

    int Frontend::FindFeaturesInRight() {
        // 1. OpenCV
        //int num_good_pts = FindFeaturesInRightLKOpticalFlowOpenCV();
        // 2. four layers LK optical flow with Gauss-Newton method
        int num_good_pts = FindFeaturesInRightLKOpticalFlow4LayerSelf();

        LOG(INFO) << "Find: " << num_good_pts << " in the right image. ";

        return num_good_pts;
    }

    bool Frontend::BuildInitMap() {
        std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
        size_t cnt_init_landmarks = 0;
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
            if (current_frame_->features_right_[i] == nullptr) continue;
            // create map point from triangulation
            VecVec3 points{
                    camera_left_->pixel2camera(
                            Vec2(current_frame_->features_left_[i]->position_.pt.x,
                                 current_frame_->features_left_[i]->position_.pt.y)),
                    camera_right_->pixel2camera(
                            Vec2(current_frame_->features_right_[i]->position_.pt.x,
                                 current_frame_->features_right_[i]->position_.pt.y))};
            Vec3 pworld = Vec3::Zero();

            // triangulation of initial landmarks
            // some constraints for autonomous driving environments
//            if (triangulation(poses, points, pworld) && pworld[2] > 0) {
            if (triangulation(poses, points, pworld) &&
                pworld[1] <= 2 &&     // ground constraints: y < 2 m
                pworld[2] > stereo_depth_inferior_limit_ && pworld[2] <= stereo_depth_superior_limit_     // depth constraints
                ) {
                auto new_map_point = MapPoint::CreateNewMappoint();
                new_map_point->SetPos(pworld);
                new_map_point->AddObservation(current_frame_->features_left_[i]);
                new_map_point->AddObservation(current_frame_->features_right_[i]);
                current_frame_->features_left_[i]->map_point_ = new_map_point;
                current_frame_->features_right_[i]->map_point_ = new_map_point;
                cnt_init_landmarks++;
                map_->InsertMapPoint(new_map_point);
            }
        }
        current_frame_->SetKeyFrame();
        map_->InsertKeyFrame(current_frame_);
        backend_->UpdateMap();

        LOG(INFO) << "Initial map created with: " << cnt_init_landmarks << " map points. ";

        return true;
    }

    bool Frontend::Reset() {
        // reset the frontend with a new map and backend
        LOG(INFO) << "Tracking LOST... \nCaptain, we are LOST... \nTake it easy, bro. \nReseting... ";

        // something need to be reset
        // hang the backend thread
        backend_->Hang();
        // reset the map
        map_->Reset();
        // restart the backend thread
        backend_->Restart();
        status_ = FrontendStatus::INITING;

        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////
    // 1. OpenCV optical flow
    int Frontend::TrackLastFrameLKOpticalFlowOpenCV() {
        // 1. OpenCV optical flow
        // use LK optical flow to estimate points in the right image
        std::vector<cv::Point2f> kps_last, kps_current;
        for (auto &kp : last_frame_->features_left_) {
            if (kp->map_point_.lock()) {
                // use project point
                auto mp = kp->map_point_.lock();
                auto px = camera_left_->world2pixel(mp->pos_, current_frame_->Pose());
                kps_last.push_back(kp->position_.pt);
                kps_current.emplace_back(px[0], px[1]);
            } else {
                kps_last.push_back(kp->position_.pt);
                kps_current.push_back(kp->position_.pt);
            }
        }

        std::vector<uchar> status;
        Mat error;
        cv::calcOpticalFlowPyrLK(
                last_frame_->left_img_, current_frame_->left_img_, kps_last,
                kps_current, status, error, cv::Size(11, 11), 3,
                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                cv::OPTFLOW_USE_INITIAL_FLOW);

        int num_good_pts = 0;
        for (size_t i = 0; i < status.size(); ++i) {
            if (status[i]) {
                cv::KeyPoint kp(kps_current[i], 7);
                Feature::Ptr feature(new Feature(current_frame_, kp));
                feature->map_point_ = last_frame_->features_left_[i]->map_point_;
                current_frame_->features_left_.push_back(feature);
                num_good_pts++;
            }
        }

        return num_good_pts;
    }

    // 1. OpenCV optical flow
    int Frontend::FindFeaturesInRightLKOpticalFlowOpenCV() {
        // 1. OpenCV optical flow
        // use LK flow to estimate points in the right image
        std::vector<cv::Point2f> kps_left, kps_right;
        for (auto &kp : current_frame_->features_left_) {
            kps_left.push_back(kp->position_.pt);
            auto mp = kp->map_point_.lock();
            if (mp) {
                // use projected points as initial guess
                auto px = camera_right_->world2pixel(mp->pos_, current_frame_->Pose());
                kps_right.emplace_back(px[0], px[1]);
            } else {
                // use same pixel in left iamge
                kps_right.push_back(kp->position_.pt);
            }
        }

        std::vector<uchar> status;
        Mat error;
        cv::calcOpticalFlowPyrLK(
                current_frame_->left_img_, current_frame_->right_img_, kps_left,
                kps_right, status, error, cv::Size(11, 11), 3,
                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                                 0.01),
                cv::OPTFLOW_USE_INITIAL_FLOW);

        int num_good_pts = 0;
        for (size_t i = 0; i < status.size(); ++i) {
            if (status[i]) {
                cv::KeyPoint kp(kps_right[i], 7);
                Feature::Ptr feat(new Feature(current_frame_, kp));
                feat->is_on_left_image_ = false;
                current_frame_->features_right_.push_back(feat);
                num_good_pts++;
            } else {
                current_frame_->features_right_.push_back(nullptr);
            }
        }

        return num_good_pts;
    }

    // 2. four layers LK optical flow with Gauss-Newton method
    int Frontend::TrackLastFrameLKOpticalFlow4LayerSelf() {
        // 2. four layers LK optical flow with Gauss-Newton method
        // use LK optical flow to estimate points in the right image
        std::vector<cv::KeyPoint> kps_last, kps_current;
        for (auto &kp : last_frame_->features_left_) {
            if (kp->map_point_.lock()) {
                // projected point
                auto mp = kp->map_point_.lock();
                auto px = camera_left_->world2pixel(mp->pos_, current_frame_->Pose());
                kps_last.emplace_back(kp->position_.pt, 7);
                kps_current.emplace_back(cv::Point2f(px[0], px[1]), 7);
            } else {
                kps_last.emplace_back(kp->position_.pt, 7);
                kps_current.emplace_back(kp->position_.pt, 7);
            }
        }

        // LKOpticalFlow4Layer: LK optical flow 4 layer
        std::vector<bool> status;
        // a. with forward mode: inverse = false
        LKOpticalFlow4Layer(last_frame_->left_img_, current_frame_->left_img_, kps_last, kps_current, status, false, true);
        // b. with backward (inverse) mode: inverse = true -> faster
        // LKOpticalFlow4Layer(last_frame_->left_img_, current_frame_->left_img_, kps_last, kps_current, status, true, true);

        int num_good_pts = 0;
        for (size_t i = 0; i < status.size(); ++i) {
            //bool inIMG = IsPtInImg(kps_current[i], current_frame_->left_img_);
            if (status[i]) {
                cv::KeyPoint kpi(kps_current[i].pt, 7);
                Feature::Ptr feature(new Feature(current_frame_, kpi));
                //feature->is_on_left_image_ = true;
                //feature->is_on_right_image_ = false;
                feature->map_point_ = last_frame_->features_left_[i]->map_point_;
                current_frame_->features_left_.push_back(feature);
                num_good_pts++;
            }
        }

        return num_good_pts;
    }

    // 2. four layers LK optical flow with Gauss-Newton method
    int Frontend::FindFeaturesInRightLKOpticalFlow4LayerSelf() {
        // 2. four layers LK optical flow with Gauss-Newton method
        // use LK optical flow to estimate points in the right image
        std::vector<cv::KeyPoint> kps_left, kps_right;
        for (auto &kp : current_frame_->features_left_) {
            kps_left.emplace_back(kp->position_.pt, 7);
            auto mp = kp->map_point_.lock();
            if (mp) {
                // use projected points as initial guess
                auto px = camera_right_->world2pixel(mp->pos_, current_frame_->Pose());
                kps_right.emplace_back(cv::Point2f(px[0], px[1]), 7);
            } else {
                // use same pixel coordinates in left iamge
                kps_right.emplace_back(kp->position_.pt, 7);
            }
        }

        // LKOpticalFlow4Layer: LK optical flow 4 layer
        std::vector<bool> status;
        // a. with forward mode: inverse = false
        LKOpticalFlow4Layer(current_frame_->left_img_, current_frame_->right_img_, kps_left, kps_right, status, false, true);
        // b. with backward (inverse) mode: inverse = true -> faster
        // LKOpticalFlow4Layer(current_frame_->left_img_, current_frame_->right_img_, kps_left, kps_right, status, true, true);

        int num_good_pts = 0;
        for (size_t i = 0; i < status.size(); ++i) {
            //bool inIMG = IsPtInImg(kps_right[i], current_frame_->right_img_);
            if (status[i]) {
                cv::KeyPoint kpi(kps_right[i].pt, 7);
                Feature::Ptr feat(new Feature(current_frame_, kpi));
                feat->is_on_left_image_ = false;
                //feat->is_on_right_image_ = true;
                current_frame_->features_right_.push_back(feat);
                num_good_pts++;
            } else {
                current_frame_->features_right_.push_back(nullptr);
            }
        }

        return num_good_pts;
    }
}
