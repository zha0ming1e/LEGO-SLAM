#ifndef LEGOSLAM_DATASET_H
#define LEGOSLAM_DATASET_H

#include "legoslam/camera.h"
#include "legoslam/common_include.h"
#include "legoslam/frame.h"

namespace legoslam {

    /*
     * dataset: after init, feed the camera and the next frame to the frontend
     */
    class Dataset {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Dataset> Ptr;

        Dataset(const std::string &dataset_path);

        // initialization
        bool Init();
        // create and return the next frame containing the stereo images
        Frame::Ptr NextFrame();
        // get camera by id
        Camera::Ptr GetCamera(int camera_id) const { return cameras_.at(camera_id); }

    private:
        std::string dataset_path_;
        int current_image_index_ = 0;
        std::vector<Camera::Ptr> cameras_;
    };
}
#endif  // LEGOSLAM_DATASET_H
