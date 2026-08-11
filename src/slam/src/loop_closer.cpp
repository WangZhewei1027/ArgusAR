#include "loop_closer.hpp"

#include <iostream>
#include <map>

#include <ceres/ceres.h>
#include <ibow_lcd/lcdetector.h>

#include "ceres_parametrization.hpp"
#include "multi_view_geometry.hpp"

namespace
{

// residual on the relative SE3 between two keyframe poses (parameter layout
// matches PoseParametersBlock: [tx ty tz qx qy qz qw])
struct RelativePoseError
{
    explicit RelativePoseError(const Sophus::SE3d &Tij) : Tij_(Tij)
    {}

    template<typename T>
    bool operator()(const T *pi, const T *pj, T *residual) const
    {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> ti(pi);
        Eigen::Map<const Eigen::Quaternion<T>> qi(pi + 3);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> tj(pj);
        Eigen::Map<const Eigen::Quaternion<T>> qj(pj + 3);

        const Sophus::SE3<T> Ti(qi.normalized(), ti);
        const Sophus::SE3<T> Tj(qj.normalized(), tj);

        const Sophus::SE3<T> err = ( Ti.inverse() * Tj ) * Tij_.inverse().cast<T>();

        Eigen::Map<Eigen::Matrix<T, 6, 1>> r(residual);
        r = err.log();

        return true;
    }

    Sophus::SE3d Tij_;
};

} // namespace

LoopCloser::LoopCloser(std::shared_ptr<State> state, std::shared_ptr<MapManager> mapManager)
        : state_(state), mapManager_(mapManager)
{
    ibow_lcd::LCDetectorParams params;

    // the defaults target long outdoor sequences; adapt to short indoor runs
    params.p = 12;                    // only skip the last 12 keyframes
    params.min_inliers = 15;
    params.island_size = 7;
    params.nframes_after_lc = 6;

    detector_ = std::make_unique<ibow_lcd::LCDetector>(params);
}

LoopCloser::~LoopCloser() = default;

void LoopCloser::reset()
{
    ibow_lcd::LCDetectorParams params;
    params.p = 12;
    params.min_inliers = 15;
    params.island_size = 7;
    params.nframes_after_lc = 6;

    detector_ = std::make_unique<ibow_lcd::LCDetector>(params);
    imageToKeyframe_.clear();
    imageKeypointIds_.clear();
    nextImageId_ = 0;
}

bool LoopCloser::onNewKeyframe(int keyframeId)
{
    auto kfIt = mapManager_->mapKeyframes_.find(keyframeId);

    if (kfIt == mapManager_->mapKeyframes_.end() || kfIt->second == nullptr)
    {
        return false;
    }

    const auto keypoints = kfIt->second->getKeypoints();

    std::vector<cv::KeyPoint> cvKps;
    cv::Mat descs;
    std::vector<int> kpIds;

    cvKps.reserve(keypoints.size());
    kpIds.reserve(keypoints.size());

    for (const auto &kp : keypoints)
    {
        if (kp.desc_.empty())
        {
            continue;
        }

        cvKps.emplace_back(kp.px_, 1.f);
        descs.push_back(kp.desc_);
        kpIds.push_back(kp.keypointId_);
    }

    if ((int) cvKps.size() < 20)
    {
        return false;
    }

    const unsigned imageId = nextImageId_++;
    imageToKeyframe_[imageId] = keyframeId;
    imageKeypointIds_[imageId] = kpIds;

    ibow_lcd::LCDetectorResult result;
    detector_->process(imageId, cvKps, descs, &result);

    if (!result.isLoop())
    {
        return false;
    }

    const auto trainIt = imageToKeyframe_.find(result.train_id);

    if (trainIt == imageToKeyframe_.end())
    {
        return false;
    }

    if (state_->debug_)
    {
        std::cout << "- [LoopCloser]: candidate loop kf " << keyframeId << " -> kf " << trainIt->second
                  << " (" << result.inliers << " bow inliers)" << std::endl;
    }

    return verifyAndClose(keyframeId, trainIt->second, result.vquery_kpids, result.vtrain_kpids);
}

bool LoopCloser::verifyAndClose(int curKfId, int trainKfId, const std::vector<int> &queryKpIndices, const std::vector<int> &trainKpIndices)
{
    auto curIt = mapManager_->mapKeyframes_.find(curKfId);
    auto trainIt = mapManager_->mapKeyframes_.find(trainKfId);

    if (curIt == mapManager_->mapKeyframes_.end() || trainIt == mapManager_->mapKeyframes_.end())
    {
        return false;
    }

    // indices in the result refer to positions in the kps vectors we passed
    // to process(); resolve them through the stored per-image id lists.
    // (linear scan for the image ids is fine at keyframe rate)
    unsigned curImageId = 0, trainImageId = 0;

    for (const auto &pair : imageToKeyframe_)
    {
        if (pair.second == curKfId) curImageId = pair.first;
        if (pair.second == trainKfId) trainImageId = pair.first;
    }

    const auto &curIds = imageKeypointIds_.at(curImageId);
    const auto &trainIds = imageKeypointIds_.at(trainImageId);

    // gather 2D-3D correspondences: current keyframe bearing vectors vs the
    // 3D map points seen in the old keyframe
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> bvs, wpts;

    for (size_t m = 0; m < queryKpIndices.size() && m < trainKpIndices.size(); m++)
    {
        const int qi = queryKpIndices[m];
        const int ti = trainKpIndices[m];

        if (qi < 0 || ti < 0 || qi >= (int) curIds.size() || ti >= (int) trainIds.size())
        {
            continue;
        }

        const auto mpIt = mapManager_->mapMapPoints_.find(trainIds[ti]);

        if (mpIt == mapManager_->mapMapPoints_.end() || mpIt->second == nullptr || !mpIt->second->is3d_)
        {
            continue;
        }

        const auto kp = curIt->second->getKeypointById(curIds[qi]);

        if (kp.keypointId_ != curIds[qi])
        {
            continue;
        }

        bvs.push_back(kp.bv_);
        wpts.push_back(mpIt->second->getPoint());
    }

    if ((int) bvs.size() < 12)
    {
        if (state_->debug_)
        {
            std::cout << "- [LoopCloser]: too few 2D-3D pairs (" << bvs.size() << "), reject" << std::endl;
        }

        return false;
    }

    Sophus::SE3d TwcCorrected;
    std::vector<int> outliers;

    const bool ok = MultiViewGeometry::p3pRansac(
            bvs, wpts,
            state_->multiViewRansacNumIterations_,
            state_->multiViewRansacError_,
            true,
            state_->multiViewRandomEnabled_,
            curIt->second->cameraCalibration_->fx_,
            curIt->second->cameraCalibration_->fy_,
            TwcCorrected,
            outliers);

    const int numInliers = (int) bvs.size() - (int) outliers.size();

    if (!ok || numInliers < 10)
    {
        if (state_->debug_)
        {
            std::cout << "- [LoopCloser]: P3P verification failed (" << numInliers << " inliers), reject" << std::endl;
        }

        return false;
    }

    if (state_->debug_)
    {
        std::cout << "- [LoopCloser]: loop verified with " << numInliers << " inliers, running pose graph optimization" << std::endl;
    }

    poseGraphOptimize(curKfId, trainKfId, TwcCorrected);
    numLoopClosures_++;

    return true;
}

void LoopCloser::poseGraphOptimize(int curKfId, int trainKfId, const Sophus::SE3d &TwcCurCorrected)
{
    // collect keyframes ordered by id
    std::map<int, std::shared_ptr<Frame>> keyframes;

    for (const auto &pair : mapManager_->mapKeyframes_)
    {
        if (pair.second != nullptr)
        {
            keyframes.emplace(pair.first, pair.second);
        }
    }

    if (keyframes.size() < 3)
    {
        return;
    }

    // save old poses (for the map-point correction afterwards)
    std::unordered_map<int, Sophus::SE3d> oldPoses;
    std::unordered_map<int, PoseParametersBlock> blocks;

    for (auto &pair : keyframes)
    {
        oldPoses.emplace(pair.first, pair.second->getTwc());
        blocks.emplace(pair.first, PoseParametersBlock(pair.first, pair.second->getTwc()));
    }

    ceres::Problem problem;
    auto *localParam = new SE3Parameterization();

    for (auto &pair : blocks)
    {
        problem.AddParameterBlock(pair.second.values(), 7, localParam);
    }

    // odometry edges between consecutive keyframes (from current estimates)
    int prevId = -1;

    for (const auto &pair : keyframes)
    {
        if (prevId >= 0)
        {
            const Sophus::SE3d Tij = oldPoses.at(prevId).inverse() * oldPoses.at(pair.first);
            problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<RelativePoseError, 6, 7, 7>(new RelativePoseError(Tij)),
                    nullptr,
                    blocks.at(prevId).values(),
                    blocks.at(pair.first).values());
        }

        prevId = pair.first;
    }

    // loop edge: relative pose between train kf and the CORRECTED current pose
    const Sophus::SE3d TloopIJ = oldPoses.at(trainKfId).inverse() * TwcCurCorrected;
    problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<RelativePoseError, 6, 7, 7>(new RelativePoseError(TloopIJ)),
            nullptr,
            blocks.at(trainKfId).values(),
            blocks.at(curKfId).values());

    // anchor the graph at the first keyframe and at the loop target
    problem.SetParameterBlockConstant(blocks.begin()->second.values());
    problem.SetParameterBlockConstant(blocks.at(trainKfId).values());

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 25;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // write corrected poses back and move the map points with their anchors
    std::unordered_map<int, Sophus::SE3d> corrections;

    for (auto &pair : keyframes)
    {
        const Sophus::SE3d TwcNew = blocks.at(pair.first).getPose();
        corrections.emplace(pair.first, TwcNew * oldPoses.at(pair.first).inverse());
        pair.second->setTwc(TwcNew);
    }

    for (auto &pair : mapManager_->mapMapPoints_)
    {
        auto &mapPoint = pair.second;

        if (mapPoint == nullptr || !mapPoint->is3d_)
        {
            continue;
        }

        const auto corrIt = corrections.find(mapPoint->keyframeId_);

        if (corrIt != corrections.end())
        {
            mapPoint->setPoint(corrIt->second * mapPoint->getPoint());
        }
    }
}
