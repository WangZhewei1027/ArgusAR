#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "map_manager.hpp"
#include "state.hpp"

namespace ibow_lcd
{
    class LCDetector;
}

// Loop closing on top of iBoW-LCD: every new keyframe is fed to the
// incremental BoW index; on a detected loop candidate the match is verified
// geometrically (P3P against the old keyframe's map points) and the keyframe
// graph is corrected with a Ceres pose-graph optimization.
class LoopCloser
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LoopCloser(std::shared_ptr<State> state, std::shared_ptr<MapManager> mapManager);

    ~LoopCloser();

    // Feed a freshly created keyframe. Returns true when a loop was closed
    // and the keyframe/map poses were corrected.
    bool onNewKeyframe(int keyframeId);

    void reset();

    int numLoopClosures_ = 0;

    // telemetry for tuning on real devices
    int numKeyframesFed_ = 0;
    int numSkippedFewDescs_ = 0;
    int numCandidates_ = 0;
    int numVerifyRejects_ = 0;
    int lastStatus_ = -1;   // ibow_lcd::LCDetectorStatus of the last process()

private:
    bool verifyAndClose(int curKfId, int trainKfId, const std::vector<int> &queryKpIndices, const std::vector<int> &trainKpIndices);

    void poseGraphOptimize(int curKfId, int trainKfId, const Sophus::SE3d &TwcCurCorrected);

    std::shared_ptr<State> state_;
    std::shared_ptr<MapManager> mapManager_;

    std::unique_ptr<ibow_lcd::LCDetector> detector_;

    // BoW image id -> keyframe id, and per-image keypoint id order
    std::unordered_map<unsigned, int> imageToKeyframe_;
    std::unordered_map<unsigned, std::vector<int>> imageKeypointIds_;
    unsigned nextImageId_ = 0;
};
