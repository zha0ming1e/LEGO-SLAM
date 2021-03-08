#pragma once

#ifndef LEGOSLAM_VISUAL_ODOMETRY_H
#define LEGOSLAM_VISUAL_ODOMETRY_H

#include "legoslam/backend.h"
#include "legoslam/common_include.h"
#include "legoslam/dataset.h"
#include "legoslam/frontend.h"
#include "legoslam/viewer.h"

namespace legoslam {

    /*
     * visual odometry API
     */
    class VisualOdometry {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<VisualOdometry> Ptr;
        // algorithm: feature-based OR direct method
        enum class ALGO { FEATURE, DIRECT };

        // constructor with config file path
        explicit VisualOdometry(std::string &config_path);
        // with algorithm
        VisualOdometry(std::string &config_path, ALGO algo);

        // initialization
        bool Init();
        // start vo in the dataset
        void Run();
        // make a step forward in dataset
        bool Step();

        // get frontend status
        FrontendStatus GetFrontendStatus() const { return frontend_->GetStatus(); }

    private:
        ALGO algo_ = ALGO::FEATURE;
        bool inited_ = false;
        std::string config_file_path_;

        Frontend::Ptr frontend_ = nullptr;
        Backend::Ptr backend_ = nullptr;
        Map::Ptr map_ = nullptr;
        Viewer::Ptr viewer_ = nullptr;
        Dataset::Ptr dataset_ = nullptr;
    };
}
#endif  // LEGOSLAM_VISUAL_ODOMETRY_H
