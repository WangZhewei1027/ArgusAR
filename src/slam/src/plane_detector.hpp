#pragma once

#include <vector>
#include <random>
#include <Eigen/Core>
#include <Eigen/Dense>

// A persistent, tracked plane in world coordinates.
// Plane equation: normal_ . x + d_ = 0, normal_ oriented towards the camera
// side at detection time.
struct DetectedPlane
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int id_ = 0;
    int type_ = 2;              // 0 = horizontal, 1 = vertical, 2 = arbitrary
    int inlierCount_ = 0;
    int hits_ = 0;              // how many updates matched this plane
    int framesSinceSeen_ = 0;

    Eigen::Vector3d normal_ = Eigen::Vector3d::UnitY();
    double d_ = 0.0;
    Eigen::Vector3d centroid_ = Eigen::Vector3d::Zero();

    // in-plane orthonormal basis (world frame); right-handed with the normal:
    // axisU_ x normal_ = axisV_
    Eigen::Vector3d axisU_ = Eigen::Vector3d::UnitX();
    Eigen::Vector3d axisV_ = Eigen::Vector3d::UnitZ();

    // convex hull of inliers in plane coords (u, v) relative to centroid_
    std::vector<Eigen::Vector2d> hull_;
    double extentU_ = 0.0;
    double extentV_ = 0.0;

    Eigen::Vector3d hullPointWorld(const Eigen::Vector2d &uv) const
    {
        return centroid_ + axisU_ * uv.x() + axisV_ * uv.y();
    }
};

// Sequential multi-plane RANSAC over the current map points, with persistent
// plane identities across updates (match -> EMA refine, no match -> new id).
class PlaneManager
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PlaneManager() = default;

    // points: world-frame map points of the current frame.
    // cameraPos: world-frame camera center (to orient plane normals).
    void update(const std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &cameraPos);

    // Intersect a world-space ray with the tracked planes (closest boundary hit
    // wins, with a small margin around the hull). Returns plane id or 0.
    int hitTest(const Eigen::Vector3d &origin, const Eigen::Vector3d &dir, Eigen::Vector3d &hitPoint) const;

    const std::vector<DetectedPlane> &planes() const { return planes_; }

    void reset();

private:
    struct FitResult
    {
        Eigen::Vector3d normal;
        double d = 0.0;
        Eigen::Vector3d centroid;
        std::vector<int> inliers;
    };

    bool ransacPlane(const std::vector<Eigen::Vector3d> &points, const std::vector<int> &candidates, double threshold, FitResult &out);

    void refinePlane(const std::vector<Eigen::Vector3d> &points, FitResult &fit, double threshold, const std::vector<int> &candidates);

    DetectedPlane *matchExisting(const FitResult &fit, double threshold);

    void applyFit(DetectedPlane &plane, const FitResult &fit, const std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &cameraPos, bool isNew);

    static void buildHull(DetectedPlane &plane, const std::vector<Eigen::Vector3d> &points, const std::vector<int> &inliers);

    static int classify(const Eigen::Vector3d &normal);

    std::vector<DetectedPlane> planes_;
    int nextId_ = 1;
    std::mt19937 rng_{42};

    static constexpr int maxPlanes_ = 8;
    static constexpr int maxPlanesPerUpdate_ = 4;
    static constexpr int ransacIterations_ = 60;
    static constexpr int minInliers_ = 25;
    static constexpr double emaAlpha_ = 0.3;
    static constexpr int maxHullPoints_ = 24;
};
