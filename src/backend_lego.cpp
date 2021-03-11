#include "legoslam/backend.h"

#include <memory>
#include "legoslam/algorithm.h"
#include "legoslam/feature.h"
#include "legoslam/lego_types.h"
#include "legoslam/map.h"
#include "legoslam/mappoint.h"

namespace legoslam {

    Backend::Backend() {
        backend_running_.store(true);
        backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this));
    }

    void Backend::UpdateMap() {
        std::unique_lock<std::mutex> lock(data_mutex_);
        map_update_.notify_one();
    }

    void Backend::Hang() {
        backend_running_.store(false);
        LOG(INFO) << "Backend is hanging. ";
    }

    void Backend::Restart() {
        backend_running_.store(true);
        LOG(INFO) << "Backend restart.";
    }

    void Backend::Stop() {
        backend_running_.store(false);
        map_update_.notify_one();
        backend_thread_.join();
    }

    void Backend::BackendLoop() {
        while (backend_running_.load()) {
            std::unique_lock<std::mutex> lock(data_mutex_);
            map_update_.wait(lock);

            // backend
            // 1. just optimize active keyframes and landmarks
            Map::KeyframesType kfs_to_opti = map_->GetActiveKeyFrames();
            Map::LandmarksType landmarks_to_opti = map_->GetActiveMapPoints();
            // 2. optimize all keyframes and landmarks
            //Map::KeyframesType kfs_to_opti = map_->GetAllKeyFrames();
            //Map::LandmarksType landmarks_to_opti = map_->GetAllMapPoints();

            // optimize
            Optimize(kfs_to_opti, landmarks_to_opti);
        }
    }

    void Backend::Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks) {
        // setup lego problem
        lego::Problem problem(lego::Problem::ProblemType::SLAM);
        //problem.setVerbose(false);

        // add vertex and edges
        // pose vertex with keyframe id
        std::map<unsigned long, std::shared_ptr<VertexPose>> vertices;
        unsigned long max_kf_id = 0;
        for (auto &keyframe : keyframes) {
            auto kf = keyframe.second;
            // camera vertex_pose
            std::shared_ptr<VertexPose> vertex_pose(new VertexPose());
            vertex_pose->setId(kf->keyframe_id_);
            vertex_pose->setEstimate(kf->Pose());
            problem.addVertex(vertex_pose);

            if (kf->keyframe_id_ > max_kf_id)
                max_kf_id = kf->keyframe_id_;

            vertices.insert({kf->keyframe_id_, vertex_pose});
        }

        // landmark vertex with landmark id
        std::map<unsigned long, std::shared_ptr<VertexXYZ>> vertices_landmarks;

        // K intrinsics matrix and extrinsics of left and right camera
        Mat33 K = cam_left_->K();
        SE3 left_ext = cam_left_->pose();
        SE3 right_ext = cam_right_->pose();

        // edges
        int index = 1;
        // robust kernel threshold: chi-squared distribution with 95% probability
        double chi2_th = 5.991;
        // cost function
        lego::CostFunction *costFunction = new lego::HuberCost(chi2_th);
        // edges and features
        std::map<std::shared_ptr<EdgeProjection>, Feature::Ptr> edges_and_features;
        // add landmark vertex and edges
        for (auto &landmark : landmarks) {
            if (landmark.second->is_outlier_) continue;

            unsigned long landmark_id = landmark.second->id_;
            auto observations = landmark.second->GetObs();
            for (auto &obs : observations) {
                if (obs.lock() == nullptr) continue;

                auto feature = obs.lock();
                auto frame = feature->frame_.lock();
                if (feature->is_outlier_ || frame == nullptr) continue;

                std::shared_ptr<EdgeProjection> edge;
                //if (feature->is_on_left_image_)
                if (feature->is_on_left_image_ && !feature->is_on_right_image_)
                {
                    edge = std::make_shared<EdgeProjection>(K, left_ext);
                }
                //else
                //else if (!feature->is_on_left_image_)
                else if (feature->is_on_right_image_ && !feature->is_on_left_image_)
                {
                    edge = std::make_shared<EdgeProjection>(K, right_ext);
                }
                else
                    continue;

                // add landmark vertex
                if (vertices_landmarks.find(landmark_id) == vertices_landmarks.end()) {
                    std::shared_ptr<VertexXYZ> v(new VertexXYZ());
                    v->setEstimate(landmark.second->Pos());
                    v->setId(landmark_id + max_kf_id + 1);
                    //v->setMarginalized(true);
                    vertices_landmarks.insert({landmark_id, v});
                    problem.addVertex(v);
                }

//                if (frame->is_keyframe_) {

                // add edges
                //edge->setId(index);
                std::vector<std::shared_ptr<lego::BaseVertex>> edge_vertexes;
                // pose
                edge_vertexes.push_back(vertices.at(frame->keyframe_id_));
                // landmark
                edge_vertexes.push_back(vertices_landmarks.at(landmark_id));
                // add vertexes to edge
                edge->setVertex(edge_vertexes);
                edge->setMeasurement(toVec2(feature->position_.pt));
                edge->setInformation(Mat22::Identity());
                // robust kernel
                edge->setCostFunction(costFunction);

                // add edges to the problem
                problem.addEdge(edge);
                edges_and_features.insert({edge, feature});

                index++;
//                }
            }
        }

        // solve: optimize and eliminate the outliers
        problem.solve(10);

        // outliers
        int cnt_outlier = 0, cnt_inlier = 0;
        int iteration = 0;
        while (iteration < 5) {
            cnt_outlier = 0;
            cnt_inlier = 0;
            // determine if we want to adjust the outlier threshold
            for (auto &ef : edges_and_features) {
                if (ef.first->getRobustChi2() > chi2_th) {
                    cnt_outlier++;
                } else {
                    cnt_inlier++;
                }
            }
            double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
            if (inlier_ratio > 0.5) {
                break;
            } else {
                chi2_th *= 2;
                iteration++;
            }
        }

        for (auto &ef : edges_and_features) {
            if (ef.first->getRobustChi2() > chi2_th) {
                ef.second->is_outlier_ = true;
                // remove the observation
                ef.second->map_point_.lock()->RemoveObservation(ef.second);
            } else {
                ef.second->is_outlier_ = false;
            }
        }

        LOG(INFO) << "Outlier/Inlier in optimization: " << cnt_outlier << " / " << cnt_inlier;

        // set pose and landmark position
        for (auto &v0 : vertices) {
//            // 7D
//            Vec7 v0_esti = v0.second->getEstimate();
//            SE3 T(Eigen::Quaterniond(v0_esti[6], v0_esti[3], v0_esti[4], v0_esti[5]),
//                  v0_esti.head<3>());

            // 6D
            Vec6 v0_esti = v0.second->getEstimate();
            SE3 T = SE3::exp(v0_esti);

            keyframes.at(v0.first)->SetPose(T);
        }

        for (auto &v1 : vertices_landmarks) {
            landmarks.at(v1.first)->SetPos(Vec3(v1.second->getEstimate()));
        }
    }
}
