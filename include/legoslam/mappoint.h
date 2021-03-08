#pragma once

#ifndef LEGOSLAM_MAPPOINT_H
#define LEGOSLAM_MAPPOINT_H

#include "legoslam/common_include.h"

namespace legoslam {

    class Frame;
    class Feature;

    /*
     * map point
     * feature will be landmark after triangulation
     */
    class MapPoint {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<MapPoint> Ptr;

        // id
        unsigned long id_ = 0;
        bool is_outlier_ = false;
        bool is_active_landmark_ = false;
        // position in world
        Vec3 pos_ = Vec3::Zero();
        std::mutex data_mutex_;
        // be observed by feature matching algorithm
        int observed_times_ = 0;
        // feature from the observation of this map point
        std::list<std::weak_ptr<Feature>> observations_;

    public:
        MapPoint() = default;

        MapPoint(long id, Vec3 position);

        Vec3 Pos() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return pos_;
        }

        void SetPos(const Vec3 &pos) {
            std::unique_lock<std::mutex> lck(data_mutex_);
            pos_ = pos;
        }

        void AddObservation(std::shared_ptr<Feature> feature) {
            std::unique_lock<std::mutex> lck(data_mutex_);
            observations_.push_back(feature);
            observed_times_++;
        }

        void RemoveObservation(std::shared_ptr<Feature> feat);

        std::list<std::weak_ptr<Feature>> GetObs() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return observations_;
        }

        // factory function
        static MapPoint::Ptr CreateNewMappoint();
    };
}
#endif  // LEGOSLAM_MAPPOINT_H
