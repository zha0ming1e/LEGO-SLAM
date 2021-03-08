#pragma once

#ifndef LEGOSLAM_CONFIG_H
#define LEGOSLAM_CONFIG_H

#include "legoslam/common_include.h"

namespace legoslam {

    /*
     * config class
     */
    class Config {
    private:
        static std::shared_ptr<Config> config_;
        cv::FileStorage file_;

        // private constructor makes a singleton
        Config() = default;

    public:
        // close the file when deconstructing
        ~Config();

        // set a new config file
        static bool SetParameterFile(const std::string &filename);

        // access the parameter values
        template <typename T>
        static T Get(const std::string &key) {
            return T(Config::config_->file_[key]);
        }
    };
}
#endif  // LEGOSLAM_CONFIG_H
