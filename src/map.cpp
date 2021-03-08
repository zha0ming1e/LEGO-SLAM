#include "legoslam/map.h"
#include "legoslam/feature.h"

namespace legoslam {

    void Map::InsertKeyFrame(Frame::Ptr frame) {
        frame->is_keyframe_ = true;
        frame->is_active_keyframe_ = true;
        current_frame_ = frame;
        if (keyframes_.find(frame->keyframe_id_) == keyframes_.end()) {
            keyframes_.insert(std::make_pair(frame->keyframe_id_, frame));
            active_keyframes_.insert(std::make_pair(frame->keyframe_id_, frame));
        } else {
            keyframes_[frame->keyframe_id_] = frame;
            active_keyframes_[frame->keyframe_id_] = frame;
        }

        if (active_keyframes_.size() > num_active_keyframes_) {
            RemoveOldKeyframe();
        }
    }

    void Map::InsertMapPoint(MapPoint::Ptr map_point) {
        map_point->is_active_landmark_ = true;
        if (landmarks_.find(map_point->id_) == landmarks_.end()) {
            landmarks_.insert(std::make_pair(map_point->id_, map_point));
            active_landmarks_.insert(std::make_pair(map_point->id_, map_point));
        } else {
            landmarks_[map_point->id_] = map_point;
            active_landmarks_[map_point->id_] = map_point;
        }
    }

    void Map::RemoveOldKeyframe() {
        if (current_frame_ == nullptr) return;

        // search the min distance and max distance keyframes from current frame
        double max_dis = 0, min_dis = 9999;
        unsigned long max_kf_id = 0, min_kf_id = 0;
        auto Twc = current_frame_->Pose().inverse();
        for (auto &kf : active_keyframes_) {
            if (kf.second == current_frame_) continue;
            auto dis = (kf.second->Pose() * Twc).log().norm();

            if (dis > max_dis) {
                max_dis = dis;
                max_kf_id = kf.first;
            }
            if (dis < min_dis) {
                min_dis = dis;
                min_kf_id = kf.first;
            }
        }

        // threshold of the min distance
        const double min_dis_th = 0.2;
        Frame::Ptr frame_to_remove = nullptr;
        // if min_dis < min_dis_th, then remove the frame with the min dis
        // otherwise, remove the frame with the max dis
        if (min_dis < min_dis_th)
            frame_to_remove = keyframes_.at(min_kf_id);
        else
            frame_to_remove = keyframes_.at(max_kf_id);

        // log
        LOG(INFO) << "Remove keyframe: " << frame_to_remove->keyframe_id_;

        // remove the keyframe and landmark observation
        frame_to_remove->is_keyframe_ = false;
        frame_to_remove->is_active_keyframe_ = false;
        active_keyframes_.erase(frame_to_remove->keyframe_id_);
        for (const auto &feat : frame_to_remove->features_left_) {
            auto mp = feat->map_point_.lock();
            if (mp)
                mp->RemoveObservation(feat);
        }
        for (const auto &feat : frame_to_remove->features_right_) {
            if (feat == nullptr) continue;
            auto mp = feat->map_point_.lock();
            if (mp)
                mp->RemoveObservation(feat);
        }

        // clean map: remove map points whose observation_times_ == 0
        CleanMap();
    }

    void Map::CleanMap() {
        int cnt_landmark_removed = 0;
        for (auto iter = active_landmarks_.begin(); iter != active_landmarks_.end(); ) {
            if (iter->second->observed_times_ == 0) {
                iter->second->is_active_landmark_ = false;
                iter = active_landmarks_.erase(iter);
                cnt_landmark_removed++;
            } else
                ++iter;
        }
        // log
        LOG(INFO) << "Removed: " << cnt_landmark_removed << " active landmarks. ";
    }
}
