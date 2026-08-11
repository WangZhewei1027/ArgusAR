#pragma once

#include <memory>
#include <vector>
#include <queue>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Core>
#include "camera_calibration.hpp"
#include "feature_extractor.hpp"
#include "feature_tracker.hpp"
#include "frame.hpp"
#include "mapper.hpp"
#include "map_manager.hpp"
#include "loop_closer.hpp"
#include "plane_detector.hpp"
#include "state.hpp"
#include "utils.hpp"
#include "visual_frontend.hpp"

class System
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    System();

    ~System();

    void configure(int imageWidth, int imageHeight, double fx, double fy, double cx, double cy, double k1, double k2, double p1, double p2);

    void reset();

    int findCameraPoseWithIMU(int imageRGBADataPtr, int imuDataPtr, int posePtr);

    int findCameraPose(int imageRGBADataPtr, int posePtr);

    // like findCameraPose but with a caller-supplied timestamp (ms) — makes
    // benchmark runs reproducible (the wall clock varies run-to-run under
    // throttled/hidden-pane frame pacing, changing motion-model priors)
    int findCameraPoseAt(int imageRGBADataPtr, int posePtr, double timestampMs);

    int findPlane(int locationPtr, int numIterations);

    // Multi-plane detection: updates the persistent PlaneManager from current
    // map points and serializes all tracked planes into planesPtr (floats).
    // Layout: [count, {id, type, inliers, pose[16], extentU, extentV,
    //                  hullCount, hull xyz * hullCount} ...]
    int getPlanes(int planesPtr);

    // Ray-cast screen pixel (x, y) against tracked planes. Writes a pose
    // (plane orientation, position = hit point) into posePtr.
    // Returns the plane id, or 0 on miss.
    int hitTest(float x, float y, int posePtr);

    int getFramePoints(int pointsPtr);

    // number of successfully closed loops since start/reset
    int getLoopClosureCount();

    void setDebug(bool enabled);

    // loop-closing telemetry: writes [kfFed, skippedFewDescs, lastStatus,
    // candidates, verifyRejects, loops] as floats; returns 6
    int getLoopStats(int statsPtr);

private:
    cv::Mat processPlane(std::vector<Eigen::Vector3d> mapPoints, Sophus::SE3d Twc, int numIterations = 50);

    int processCameraPose(cv::Mat &image, double timestamp);

    std::shared_ptr<State> state_;
    std::shared_ptr<Frame> currFrame_;
    std::shared_ptr<CameraCalibration> cameraCalibration_;
    std::shared_ptr<MapManager> mapManager_;
    std::shared_ptr<Mapper> mapper_;
    std::unique_ptr<VisualFrontend> visualFrontend_;
    std::shared_ptr<FeatureExtractor> featureExtractor_;
    std::shared_ptr<FeatureTracker> featureTracker_;
    std::unique_ptr<PlaneManager> planeManager_;
    std::unique_ptr<LoopCloser> loopCloser_;
    size_t prevKeyframeCount_ = 0;

    Eigen::Vector3d currTranslation_;
    Eigen::Vector3d prevTranslation_;
};
