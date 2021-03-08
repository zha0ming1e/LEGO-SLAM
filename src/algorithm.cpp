#include "legoslam/algorithm.h"
#include <opencv2/opencv.hpp>

namespace legoslam {

    /*
     * LK optical flow calculation:
     * class and function
     */
    // LK optical flow with 1 layer
    void LKOpticalFlow1Layer(const cv::Mat &img1, const cv::Mat &img2,
                             const std::vector<cv::KeyPoint> &kp1,
                             std::vector<cv::KeyPoint> &kp2,
                             std::vector<bool> &success,
                             bool inverse, bool has_initial) {
        // initialization
        kp2.resize(kp1.size());
        success.resize(kp1.size());
        // LKOpticalFlowTracker
        LKOpticalFlowTracker tracker(img1, img2, kp1, kp2, success, inverse, has_initial);
        // cv::parallel_for_ : parallel computing for LK optical flow tracking
        // cv::Range
        cv::Range range(0, kp1.size());
        // cv::ParallelLoopBody
        LKOpticalFlowTrackerParallelLoopBody LKloopbody(&tracker);
        // cv::parallel_for_
        cv::parallel_for_(range, LKloopbody);

        // if you dont wanna the parallel computing, step by step: serial computing
        //tracker.calcLKOpticalFlow(range);
    }

    /*
     * LKOpticalFlowTracker::calcLKOpticalFlow(const cv::Range &range)
     */
    // calculate optical flow: Gauss-Newton optimization -> (J^T * W * J) * dx = -e * W * J^T
    void LKOpticalFlowTracker::calcLKOpticalFlow(const cv::Range &range) {
        // parameters
        // window side length = (2 * half_patch_size + 1)
        int half_patch_size = 3;
        int half_grad_step = 1;
        int iterations = 10;
        for (int i = range.start; i < range.end; ++i) {
            auto kp = kp1[i];
            double dx = 0, dy = 0;
            // if has initialized the kp2
            if (has_initial) {
                dx = kp2[i].pt.x - kp.pt.x;
                dy = kp2[i].pt.y - kp.pt.y;
            }

            double cost = 0, lastCost = 0;
            bool succ = true;
            // H * dx = b
            Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
            Eigen::Vector2d b = Eigen::Vector2d::Zero();
            Eigen::Vector2d J = Eigen::Vector2d::Zero();
            for (int iter = 0; iter < iterations; ++iter) {
                if (!inverse) H = Eigen::Matrix2d::Zero();

                b = Eigen::Vector2d::Zero();
                cost = 0;
                for (int x = -half_patch_size; x <= half_patch_size; ++x) {
                    for (int y = -half_patch_size; y <= half_patch_size; ++y) {
                        double error = GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y) -
                                       GetPixelValue(img2, kp.pt.x + x + dx, kp.pt.y + y + dy);

                        if (!inverse) {
                            J = -1.0 * Eigen::Vector2d(
                                    0.5 * (GetPixelValue(img2, kp.pt.x + x + dx + half_grad_step, kp.pt.y + y + dy) -
                                           GetPixelValue(img2, kp.pt.x + x + dx - half_grad_step, kp.pt.y + y + dy)),
                                    0.5 * (GetPixelValue(img2, kp.pt.x + x + dx, kp.pt.y + y + dy + half_grad_step) -
                                           GetPixelValue(img2, kp.pt.x + x + dx, kp.pt.y + y + dy - half_grad_step)));
                        } else if (iter == 0) {
                            J = -1.0 * Eigen::Vector2d(
                                    0.5 * (GetPixelValue(img1, kp.pt.x + x + half_grad_step, kp.pt.y + y) -
                                           GetPixelValue(img1, kp.pt.x + x - half_grad_step, kp.pt.y + y)),
                                    0.5 * (GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y + half_grad_step) -
                                           GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y - half_grad_step)));
                        }

                        // H, b and cost
                        b += -error * J;
                        cost += error * error;
                        if (!inverse || iter == 0) {
                            H += J * J.transpose();
                        }
                    }
                }

                // compute update with cholesky decomposition
                Eigen::Vector2d update = Eigen::Vector2d::Zero();
                update = H.ldlt().solve(b);
                if (std::isnan(update[0]) || std::isnan(update[1]) ||
                    std::isinf(update[0]) || std::isinf(update[1])) {
                    // sometimes occurred when we have a black or white patch and H is irreversible
                    std::cout << "Update is NaN or INF. " << std::endl;
                    succ = false;
                    break;
                }

                if (iter > 0 && cost > lastCost) {
                    break;
                }

                // update
                dx += update[0];
                dy += update[1];
                lastCost = cost;
                succ = true;

                // converge
                if (update.norm() < 1e-2) {
                    break;
                }
            }
            // check
            // set success[i]
            success[i] = succ;
            // set kp2[i]
            kp2[i].pt = kp.pt + cv::Point2f(dx, dy);
            // check if kp2[i] is out of the img2
            if (!IsPtInImg(kp2[i], img2)) success[i] = false;
        }
    }

    // LK optical flow with 4 layers
    void LKOpticalFlow4Layer(const cv::Mat &img1, const cv::Mat &img2,
                             const std::vector<cv::KeyPoint> &kp1,
                             std::vector<cv::KeyPoint> &kp2,
                             std::vector<bool> &success,
                             bool inverse, bool has_initial) {
        // parameters
        // 4 layers with scale = 0.5
        int pyramids = 4;
        double pyramid_scale = 0.5;
        double scales[] = {1.0, 0.5, 0.25, 0.125};

        // create image pyramids
        std::vector<cv::Mat> pyr1, pyr2;
        for (int i = 0; i < pyramids; i++) {
            if (i == 0) {
                pyr1.push_back(img1);
                pyr2.push_back(img2);
            } else {
                cv::Mat img1_pyr, img2_pyr;
                cv::resize(pyr1[i-1], img1_pyr,
                           cv::Size(pyr1[i-1].cols * pyramid_scale, pyr1[i-1].rows * pyramid_scale));
                cv::resize(pyr2[i-1], img2_pyr,
                           cv::Size(pyr2[i-1].cols * pyramid_scale, pyr2[i-1].rows * pyramid_scale));
                pyr1.push_back(img1_pyr);
                pyr2.push_back(img2_pyr);
            }
        }

        // coarse-to-fine LK optical flow tracking in pyramids
        // key point positions in the top layer
        kp2.resize(kp1.size());
        std::vector<cv::KeyPoint> kp1_pyr, kp2_pyr;
        for (size_t i = 0; i < kp1.size(); ++i) {
            double scale_top = scales[pyramids - 1];
            auto kp1_top = kp1[i];
            auto kp2_top = kp2[i];

            kp1_top.pt *= scale_top;
            kp2_top.pt *= scale_top;
            kp1_pyr.push_back(kp1_top);
            kp2_pyr.push_back(kp2_top);
        }
//        for (auto &kp : kp1) {
//            auto kp_top = kp;
//            kp_top.pt *= scales[pyramids - 1];
//            kp1_pyr.push_back(kp_top);
//        }
//        for (auto &kp : kp2) {
//            auto kp_top = kp;
//            kp_top.pt *= scales[pyramids - 1];
//            kp2_pyr.push_back(kp_top);
//        }

        // from coarse to fine
        for (int level = pyramids-1; level >= 0; --level) {
            // success.clear();
            // the first loop: has_initial is set by users
            if (level == pyramids-1)
                LKOpticalFlow1Layer(pyr1[level], pyr2[level], kp1_pyr, kp2_pyr, success, inverse, has_initial);
            else
                // the other loop: set has_initial = true
                LKOpticalFlow1Layer(pyr1[level], pyr2[level], kp1_pyr, kp2_pyr, success, inverse, true);

            // for the first 3 layers
            if (level > 0) {
                for (size_t i = 0; i < kp1_pyr.size(); ++i) {
                    kp1_pyr[i].pt /= pyramid_scale;

                    if (success[i])
                        kp2_pyr[i].pt /= pyramid_scale;
                    else
                        kp2_pyr[i].pt = kp1_pyr[i].pt;
                }
            }
        }

        // update kp2 with kp2_pyr in the last loop
        kp2 = kp2_pyr;
    }
}
