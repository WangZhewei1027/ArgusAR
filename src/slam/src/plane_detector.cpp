#include "plane_detector.hpp"

#include <algorithm>
#include <cmath>

namespace
{

// world up: this SLAM uses a y-down camera/world convention
const Eigen::Vector3d kWorldUp( 0.0, -1.0, 0.0 );

double sceneScale( const std::vector<Eigen::Vector3d> &points )
{
    Eigen::Vector3d minP = points[0];
    Eigen::Vector3d maxP = points[0];

    for( const auto &p : points )
    {
        minP = minP.cwiseMin( p );
        maxP = maxP.cwiseMax( p );
    }

    return ( maxP - minP ).norm();
}

// Andrew's monotone chain, returns hull indices into pts (counter-clockwise)
std::vector<int> convexHull2d( const std::vector<Eigen::Vector2d> &pts )
{
    const int n = (int) pts.size();

    if( n < 3 )
    {
        std::vector<int> all( n );
        for( int i = 0; i < n; i++ ) all[i] = i;
        return all;
    }

    std::vector<int> idx( n );
    for( int i = 0; i < n; i++ ) idx[i] = i;

    std::sort( idx.begin(), idx.end(), [&]( int a, int b )
    {
        return pts[a].x() < pts[b].x() || ( pts[a].x() == pts[b].x() && pts[a].y() < pts[b].y() );
    } );

    auto cross = [&]( int o, int a, int b )
    {
        const Eigen::Vector2d oa = pts[a] - pts[o];
        const Eigen::Vector2d ob = pts[b] - pts[o];
        return oa.x() * ob.y() - oa.y() * ob.x();
    };

    std::vector<int> hull( 2 * n );
    int k = 0;

    for( int i = 0; i < n; i++ )
    {
        while( k >= 2 && cross( hull[k - 2], hull[k - 1], idx[i] ) <= 0 ) k--;
        hull[k++] = idx[i];
    }

    for( int i = n - 2, t = k + 1; i >= 0; i-- )
    {
        while( k >= t && cross( hull[k - 2], hull[k - 1], idx[i] ) <= 0 ) k--;
        hull[k++] = idx[i];
    }

    hull.resize( k - 1 );
    return hull;
}

bool pointInPolygon( const std::vector<Eigen::Vector2d> &poly, const Eigen::Vector2d &p, double margin )
{
    // winding-free even-odd test on the polygon inflated by `margin` (approx:
    // test distance to edges when outside)
    const int n = (int) poly.size();

    if( n < 3 )
    {
        return false;
    }

    bool inside = false;

    for( int i = 0, j = n - 1; i < n; j = i++ )
    {
        const Eigen::Vector2d &a = poly[j];
        const Eigen::Vector2d &b = poly[i];

        if( ( ( b.y() > p.y() ) != ( a.y() > p.y() ) ) &&
            ( p.x() < ( a.x() - b.x() ) * ( p.y() - b.y() ) / ( a.y() - b.y() ) + b.x() ) )
        {
            inside = !inside;
        }
    }

    if( inside )
    {
        return true;
    }

    if( margin <= 0.0 )
    {
        return false;
    }

    // near-boundary tolerance: distance to closest edge segment
    double best = 1e30;

    for( int i = 0, j = n - 1; i < n; j = i++ )
    {
        const Eigen::Vector2d &a = poly[j];
        const Eigen::Vector2d &b = poly[i];
        const Eigen::Vector2d ab = b - a;
        const double len2 = ab.squaredNorm();
        const double t = ( len2 > 0.0 ) ? std::clamp( ( p - a ).dot( ab ) / len2, 0.0, 1.0 ) : 0.0;
        best = std::min( best, ( p - ( a + t * ab ) ).squaredNorm() );
    }

    return best <= margin * margin;
}

} // namespace

void PlaneManager::reset()
{
    planes_.clear();
    nextId_ = 1;
}

int PlaneManager::classify( const Eigen::Vector3d &normal )
{
    const double c = std::fabs( normal.dot( kWorldUp ) );

    if( c >= std::cos( 25.0 * M_PI / 180.0 ) ) return 0;   // horizontal
    if( c <= std::cos( 65.0 * M_PI / 180.0 ) ) return 1;   // vertical
    return 2;                                              // arbitrary
}

bool PlaneManager::ransacPlane( const std::vector<Eigen::Vector3d> &points, const std::vector<int> &candidates, double threshold, FitResult &out )
{
    const int n = (int) candidates.size();

    if( n < minInliers_ )
    {
        return false;
    }

    std::uniform_int_distribution<int> pick( 0, n - 1 );

    int bestCount = 0;
    Eigen::Vector3d bestN;
    double bestD = 0.0;

    for( int it = 0; it < ransacIterations_; it++ )
    {
        const Eigen::Vector3d &p0 = points[candidates[pick( rng_ )]];
        const Eigen::Vector3d &p1 = points[candidates[pick( rng_ )]];
        const Eigen::Vector3d &p2 = points[candidates[pick( rng_ )]];

        Eigen::Vector3d nrm = ( p1 - p0 ).cross( p2 - p0 );
        const double len = nrm.norm();

        if( len < 1e-12 )
        {
            continue;
        }

        nrm /= len;
        const double d = -nrm.dot( p0 );

        int count = 0;

        for( int ci : candidates )
        {
            if( std::fabs( nrm.dot( points[ci] ) + d ) < threshold )
            {
                count++;
            }
        }

        if( count > bestCount )
        {
            bestCount = count;
            bestN = nrm;
            bestD = d;
        }
    }

    if( bestCount < minInliers_ )
    {
        return false;
    }

    out.normal = bestN;
    out.d = bestD;
    refinePlane( points, out, threshold, candidates );

    return (int) out.inliers.size() >= minInliers_;
}

void PlaneManager::refinePlane( const std::vector<Eigen::Vector3d> &points, FitResult &fit, double threshold, const std::vector<int> &candidates )
{
    // two rounds: collect inliers -> PCA refit -> recollect
    for( int round = 0; round < 2; round++ )
    {
        fit.inliers.clear();

        for( int ci : candidates )
        {
            if( std::fabs( fit.normal.dot( points[ci] ) + fit.d ) < threshold )
            {
                fit.inliers.push_back( ci );
            }
        }

        if( (int) fit.inliers.size() < 3 )
        {
            return;
        }

        Eigen::Vector3d mean = Eigen::Vector3d::Zero();

        for( int ci : fit.inliers )
        {
            mean += points[ci];
        }

        mean /= (double) fit.inliers.size();

        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();

        for( int ci : fit.inliers )
        {
            const Eigen::Vector3d q = points[ci] - mean;
            cov += q * q.transpose();
        }

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es( cov );
        Eigen::Vector3d nrm = es.eigenvectors().col( 0 ); // smallest eigenvalue

        if( nrm.dot( fit.normal ) < 0.0 )
        {
            nrm = -nrm;
        }

        fit.normal = nrm.normalized();
        fit.centroid = mean;
        fit.d = -fit.normal.dot( mean );
    }
}

DetectedPlane *PlaneManager::matchExisting( const FitResult &fit, double threshold )
{
    const double cosMaxAngle = std::cos( 25.0 * M_PI / 180.0 );

    DetectedPlane *best = nullptr;
    double bestDist = 1e30;

    for( auto &plane : planes_ )
    {
        if( std::fabs( fit.normal.dot( plane.normal_ ) ) < cosMaxAngle )
        {
            continue;
        }

        // distance of the new centroid to the old plane
        const double planeDist = std::fabs( plane.normal_.dot( fit.centroid ) + plane.d_ );

        if( planeDist > 6.0 * threshold )
        {
            continue;
        }

        // lateral distance between centroids, bounded by the old extent
        const double lateral = ( fit.centroid - plane.centroid_ ).norm();
        const double reach = std::max( plane.extentU_, plane.extentV_ ) * 2.0 + 8.0 * threshold;

        if( lateral > reach )
        {
            continue;
        }

        if( planeDist < bestDist )
        {
            bestDist = planeDist;
            best = &plane;
        }
    }

    return best;
}

void PlaneManager::applyFit( DetectedPlane &plane, const FitResult &fit, const std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &cameraPos, bool isNew )
{
    Eigen::Vector3d n = fit.normal;

    // orient towards the camera side
    if( n.dot( cameraPos - fit.centroid ) < 0.0 )
    {
        n = -n;
    }

    if( isNew )
    {
        plane.normal_ = n;
        plane.centroid_ = fit.centroid;

        // pick any stable in-plane axis
        const Eigen::Vector3d ref = ( std::fabs( n.x() ) < 0.9 ) ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
        plane.axisU_ = ( ref - ref.dot( n ) * n ).normalized();
    }
    else
    {
        // EMA smoothing, keeping the axis frame continuous
        if( n.dot( plane.normal_ ) < 0.0 )
        {
            n = -n;
        }

        plane.normal_ = ( ( 1.0 - emaAlpha_ ) * plane.normal_ + emaAlpha_ * n ).normalized();
        plane.centroid_ = ( 1.0 - emaAlpha_ ) * plane.centroid_ + emaAlpha_ * fit.centroid;
        plane.axisU_ = ( plane.axisU_ - plane.axisU_.dot( plane.normal_ ) * plane.normal_ ).normalized();
    }

    plane.axisV_ = plane.axisU_.cross( plane.normal_ ).normalized();
    // keep (axisU, normal, axisV) right-handed: axisU x normal = axisV
    plane.d_ = -plane.normal_.dot( plane.centroid_ );
    plane.type_ = classify( plane.normal_ );
    plane.inlierCount_ = (int) fit.inliers.size();
    plane.framesSinceSeen_ = 0;
    plane.hits_++;

    buildHull( plane, points, fit.inliers );
}

void PlaneManager::buildHull( DetectedPlane &plane, const std::vector<Eigen::Vector3d> &points, const std::vector<int> &inliers )
{
    std::vector<Eigen::Vector2d> uv;
    uv.reserve( inliers.size() );

    for( int ci : inliers )
    {
        const Eigen::Vector3d q = points[ci] - plane.centroid_;
        uv.emplace_back( q.dot( plane.axisU_ ), q.dot( plane.axisV_ ) );
    }

    const std::vector<int> hullIdx = convexHull2d( uv );

    plane.hull_.clear();

    // decimate to the hull-point budget
    const int step = std::max( 1, (int) ( hullIdx.size() + maxHullPoints_ - 1 ) / maxHullPoints_ );

    for( size_t i = 0; i < hullIdx.size(); i += step )
    {
        plane.hull_.push_back( uv[hullIdx[i]] );
    }

    double minU = 0, maxU = 0, minV = 0, maxV = 0;

    for( const auto &p : plane.hull_ )
    {
        minU = std::min( minU, p.x() );
        maxU = std::max( maxU, p.x() );
        minV = std::min( minV, p.y() );
        maxV = std::max( maxV, p.y() );
    }

    plane.extentU_ = maxU - minU;
    plane.extentV_ = maxV - minV;
}

void PlaneManager::update( const std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &cameraPos )
{
    for( auto &plane : planes_ )
    {
        plane.framesSinceSeen_++;
    }

    // prune planes that never confirmed: seen few times and long unseen
    planes_.erase( std::remove_if( planes_.begin(), planes_.end(), []( const DetectedPlane &p )
    {
        return p.hits_ < 5 && p.framesSinceSeen_ > 120;
    } ), planes_.end() );

    if( (int) points.size() < minInliers_ * 2 )
    {
        return;
    }

    const double threshold = std::max( 1e-5, sceneScale( points ) * 0.018 );

    std::vector<int> remaining( points.size() );
    for( size_t i = 0; i < points.size(); i++ ) remaining[i] = (int) i;

    for( int k = 0; k < maxPlanesPerUpdate_; k++ )
    {
        FitResult fit;

        if( !ransacPlane( points, remaining, threshold, fit ) )
        {
            break;
        }

        DetectedPlane *existing = matchExisting( fit, threshold );

        if( existing != nullptr )
        {
            applyFit( *existing, fit, points, cameraPos, false );
        }
        else
        {
            if( (int) planes_.size() >= maxPlanes_ )
            {
                // evict the weakest never-confirmed plane, if any
                auto weakest = std::min_element( planes_.begin(), planes_.end(), []( const DetectedPlane &a, const DetectedPlane &b )
                {
                    return a.hits_ < b.hits_;
                } );

                if( weakest != planes_.end() && weakest->hits_ <= 3 )
                {
                    planes_.erase( weakest );
                }
            }

            if( (int) planes_.size() < maxPlanes_ )
            {
                DetectedPlane plane;
                plane.id_ = nextId_++;
                applyFit( plane, fit, points, cameraPos, true );
                planes_.push_back( plane );
            }
        }

        // remove consumed inliers and continue searching
        std::vector<char> used( points.size(), 0 );

        for( int ci : fit.inliers )
        {
            used[ci] = 1;
        }

        std::vector<int> next;
        next.reserve( remaining.size() );

        for( int ci : remaining )
        {
            if( !used[ci] )
            {
                next.push_back( ci );
            }
        }

        remaining.swap( next );

        if( (int) remaining.size() < minInliers_ )
        {
            break;
        }
    }
}

int PlaneManager::hitTest( const Eigen::Vector3d &origin, const Eigen::Vector3d &dir, Eigen::Vector3d &hitPoint ) const
{
    int bestId = 0;
    double bestT = 1e30;

    for( const auto &plane : planes_ )
    {
        const double denom = plane.normal_.dot( dir );

        if( std::fabs( denom ) < 1e-9 )
        {
            continue;
        }

        const double t = -( plane.normal_.dot( origin ) + plane.d_ ) / denom;

        if( t <= 0.0 || t >= bestT )
        {
            continue;
        }

        const Eigen::Vector3d p = origin + t * dir;
        const Eigen::Vector3d q = p - plane.centroid_;
        const Eigen::Vector2d uv( q.dot( plane.axisU_ ), q.dot( plane.axisV_ ) );

        const double margin = 0.15 * std::max( plane.extentU_, plane.extentV_ );

        if( !pointInPolygon( plane.hull_, uv, margin ) )
        {
            continue;
        }

        bestT = t;
        bestId = plane.id_;
        hitPoint = p;
    }

    return bestId;
}
