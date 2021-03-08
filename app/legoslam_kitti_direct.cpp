#include <iostream>
#include <gflags/gflags.h>
#include "legoslam/visual_odometry.h"

DEFINE_string(config_file, "../config/default.yaml", "config file path");

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "Usage: RUN_KITTI_STEREO [CONDIFG_FILE_YAML] \nERROR: argc is not 2. " << std::endl;
        return -1;
    }

    google::ParseCommandLineFlags(&argc, &argv, true);

    // set the path of the config file
    FLAGS_config_file.assign(argv[1]);

    // set algo_: FEATURE or DIRECT method
    // direct method
    legoslam::VisualOdometry::Ptr vo(new legoslam::VisualOdometry(FLAGS_config_file, legoslam::VisualOdometry::ALGO::DIRECT));

    // initialization
    assert(vo->Init());

    // run
    vo->Run();

    return 0;
}
