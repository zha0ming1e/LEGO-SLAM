#ifndef LEGOSLAM_ALGORITHM_H
#define LEGOSLAM_ALGORITHM_H

#include "legoslam/common_include.h"

namespace legoslam {

    /*
     * triangulation
     */
    inline bool triangulation(const std::vector<SE3> &poses,
                              const VecVec3 &points,
                              Vec3 &pt_world,
                              double singRatioThr=1e-3) {
        MatXX A(2 * poses.size(), 4);
        VecX b(2 * poses.size());
        b.setZero();
        for (size_t i = 0; i < poses.size(); ++i) {
            Mat34 m = poses[i].matrix3x4();
            A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
            A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
        }
        auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

        if (std::isnan(pt_world[0]) || std::isnan(pt_world[1]) || std::isnan(pt_world[2]) ||
            std::isinf(pt_world[0]) || std::isinf(pt_world[1]) || std::isinf(pt_world[2]))
            return false;
        // check singular value ratio: null-space representation with singular values
        if (svd.singularValues()[3] / svd.singularValues()[2] < singRatioThr)
            return true;
        // abandon this result
        return false;
    }

    // converters
    inline Vec2 toVec2(const cv::Point2f &p) { return Vec2{p.x, p.y}; }

    // GetPixelValue (bi-linear interpolated)
    inline float GetPixelValue(const cv::Mat &img, float x, float y) {
        // boundary check
        if (x < 0) x = 0;
        if (y < 0) y = 0;
        if (x >= img.cols) x = img.cols - 1;
        if (y >= img.rows) y = img.rows - 1;

        // bi-linear interpolate
        uchar *data = &img.data[int(y) * img.step + int(x)];
        float xx = x - floor(x);
        float yy = y - floor(y);
        return float(
                (1 - xx) * (1 - yy) * data[0] +
                xx * (1 - yy) * data[1] +
                (1 - xx) * yy * data[img.step] +
                xx * yy * data[img.step + 1]
        );
    }

    // is a point in the img
    inline bool IsPtInImg(const cv::KeyPoint &kp, const cv::Mat &img) {
        double x = kp.pt.x, y = kp.pt.y;
        if (x < 0 || y < 0 || x >= img.cols || y >= img.rows)
            return false;
        else
            return true;
    }

    //////////////////////////////LK Optical Flow//////////////////////////////
    //
    // LK optical flow calculation: Gauss-Newton method
    // class and function
    //

    /*
     * class LKOpticalFlowTracker: LK optical flow tracker
     */
    class LKOpticalFlowTracker {
    public:
        LKOpticalFlowTracker(const cv::Mat &img1_, const cv::Mat &img2_,
                             const std::vector<cv::KeyPoint> &kp1_,
                             std::vector<cv::KeyPoint> &kp2_,
                             std::vector<bool> &success_,
                             bool inverse_, bool has_initial_) :
                             img1(img1_), img2(img2_),
                             kp1(kp1_), kp2(kp2_), success(success_), inverse(inverse_), has_initial(has_initial_) {}
        // calculate optical flow
        void calcLKOpticalFlow(const cv::Range &range);

    private:
        const cv::Mat &img1;
        const cv::Mat &img2;
        const std::vector<cv::KeyPoint> &kp1;
        std::vector<cv::KeyPoint> &kp2;
        std::vector<bool> &success;
        bool inverse = false;
        bool has_initial = true;
    };

    /*
     * class LKOpticalFlowTrackerParallelLoopBody : public cv::ParallelLoopBody
     */
    // using cv::parallel_for_ with parallel computing
    // cv::parallel_for_ -> class cv::ParallelLoopBody
    // parallel class: operator() -> overload
    class LKOpticalFlowTrackerParallelLoopBody : public cv::ParallelLoopBody {
    public:
        explicit LKOpticalFlowTrackerParallelLoopBody(const LKOpticalFlowTracker *tracker_ptr_) {
            tracker_ptr = const_cast<LKOpticalFlowTracker *>(tracker_ptr_);
        }

        // overloading operator()
        void operator()(const cv::Range &range) const override {
            tracker_ptr->calcLKOpticalFlow(range);
        }

    public:
        // public data member: the pointer to a object instance of LKOpticalFlowTracker
        // for the reason that operator() is a const function
        LKOpticalFlowTracker *tracker_ptr;
    };

    // LK optical flow 1 layer
    void LKOpticalFlow1Layer(const cv::Mat &img1, const cv::Mat &img2,
                             const std::vector<cv::KeyPoint> &kp1,
                             std::vector<cv::KeyPoint> &kp2,
                             std::vector<bool> &success,
                             bool inverse=false,
                             bool has_initial=true);

    // LK optical flow 4 layer
    void LKOpticalFlow4Layer(const cv::Mat &img1, const cv::Mat &img2,
                             const std::vector<cv::KeyPoint> &kp1,
                             std::vector<cv::KeyPoint> &kp2,
                             std::vector<bool> &success,
                             bool inverse=false,
                             bool has_initial=true);
}
#endif  // LEGOSLAM_ALGORITHM_H
