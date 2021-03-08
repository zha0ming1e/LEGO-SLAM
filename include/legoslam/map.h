#pragma once

#ifndef LEGOSLAM_MAP_H
#define LEGOSLAM_MAP_H

#include "legoslam/common_include.h"
#include "legoslam/frame.h"
#include "legoslam/mappoint.h"

namespace legoslam {

    /*
     * map
     */
    class Map {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Map> Ptr;
        typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
        //typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;
        typedef std::map<unsigned long, Frame::Ptr> KeyframesType;

        // constructor
        Map() = default;

        // insert a keyframe
        void InsertKeyFrame(Frame::Ptr frame);
        // insert a map point
        void InsertMapPoint(MapPoint::Ptr map_point);
        // get all map points
        LandmarksType GetAllMapPoints() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return landmarks_;
        }
        // get all keyframes
        KeyframesType GetAllKeyFrames() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return keyframes_;
        }
        // get active map pointd
        LandmarksType GetActiveMapPoints() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return active_landmarks_;
        }
        // get active keyframes
        KeyframesType GetActiveKeyFrames() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return active_keyframes_;
        }
        // clean points in the map whose observation_times_ is 0
        void CleanMap();
        // reset map
        void Reset() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            // clear data
            LOG(INFO) << "Map is resetting... ";
            active_landmarks_.clear();
            landmarks_.clear();
            active_keyframes_.clear();
            keyframes_.clear();
            LOG(INFO) << "Map resetting done. ";
        }

    private:
        // set old keyframe to inactive
        void RemoveOldKeyframe();

        std::mutex data_mutex_;
        // all landmarks
        LandmarksType landmarks_;
        // active landmarks
        LandmarksType active_landmarks_;
        // all keyframes
        KeyframesType keyframes_;
        // active keyframes
        KeyframesType active_keyframes_;

        // current frame
        Frame::Ptr current_frame_ = nullptr;

        // num of active keyframes
        int num_active_keyframes_ = 15;
    };
}
#endif  // LEGOSLAM_MAP_H
