/**
 * This file is part of Simox.
 *
 * Simox is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * Simox is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @author     Simon Ottenhaus (simon dot ottenhaus at kit dot edu)
 * @copyright  2018 Simon Ottenhaus
 *             GNU Lesser General Public License
 */

#pragma once

//scale intern sizes to milimeters
#define HAPTIC_EXPLORATION_SCALE 40

#include <memory>
#include <stdexcept>
#include <vector>

#include <eigen3/Eigen/Core>

template <class T>
class Nullable
{
public:
    Nullable(T& value) : defined(true), value(value)
    {
    }

    Nullable() : defined(false)
    {
    }

    T
    get()
    {
        if (!defined)
        {
            throw std::runtime_error("Value is not set");
        }
        return value;
    }

    bool
    hasValue()
    {
        return defined;
    }

private:
    bool defined;
    T value;
};

namespace math
{
    typedef Nullable<Eigen::Vector3f> Vec3Opt;

    typedef std::shared_ptr<class AbstractFunctionR1R2> AbstractFunctionR1R2Ptr;
    typedef std::shared_ptr<class Line> LinePtr;
    typedef std::shared_ptr<class LineStrip> LineStripPtr;
    typedef std::shared_ptr<class AbstractFunctionR1R3> AbstractFunctionR1R3Ptr;
    typedef std::shared_ptr<class AbstractFunctionR1R6> AbstractFunctionR1R6Ptr;
    typedef std::shared_ptr<class AbstractFunctionR2R3> AbstractFunctionR2R3Ptr;
    typedef std::shared_ptr<class AbstractFunctionR3R1> AbstractFunctionR3R1Ptr;
    typedef std::shared_ptr<class Contact> ContactPtr;
    typedef std::shared_ptr<class ContactList> ContactListPtr;
    typedef std::shared_ptr<class ImplicitPlane> ImplicitPlanePtr;
    typedef std::shared_ptr<class LineR2> LineR2Ptr;
    typedef std::shared_ptr<class Plane> PlanePtr;
    typedef std::shared_ptr<class Triangle> TrianglePtr;
    typedef std::shared_ptr<class SimpleAbstractFunctionR1R3> SimpleAbstractFunctionR1R3Ptr;
    typedef std::shared_ptr<class SimpleAbstractFunctionR1R6> SimpleAbstractFunctionR1R6Ptr;
    typedef std::shared_ptr<class SimpleAbstractFunctionR2R3> SimpleAbstractFunctionR2R3Ptr;
    typedef std::shared_ptr<class SimpleAbstractFunctionR3R1> SimpleAbstractFunctionR3R1Ptr;
    typedef std::shared_ptr<class LinearInterpolatedOrientation> LinearInterpolatedOrientationPtr;
    typedef std::shared_ptr<class LinearInterpolatedPose> LinearInterpolatedPosePtr;
    typedef std::shared_ptr<class AbstractFunctionR1Ori> AbstractFunctionR1OriPtr;
    typedef std::shared_ptr<class SimpleAbstractFunctionR1Ori> SimpleAbstractFunctionR1OriPtr;
    typedef std::shared_ptr<class CompositeFunctionR1R6> CompositeFunctionR1R6Ptr;
    typedef std::shared_ptr<class AbstractFunctionR1R6> AbstractFunctionR1R6Ptr;
    typedef std::shared_ptr<class ImplicitObjectModel> ImplicitObjectModelPtr;
    typedef std::shared_ptr<class HalfSpaceObjectModel> HalfSpaceObjectModelPtr;
    typedef std::shared_ptr<class HalfSpaceImplicitSurface3D> HalfSpaceImplicitSurface3DPtr;
    typedef std::shared_ptr<class GaussianObjectModel> GaussianObjectModelPtr;
    typedef std::shared_ptr<class GaussianObjectModelNormals> GaussianObjectModelNormalsPtr;
    typedef std::shared_ptr<class GaussianImplicitSurface3D> GaussianImplicitSurface3DPtr;
    typedef std::shared_ptr<class GaussianImplicitSurface3DNormals>
        GaussianImplicitSurface3DNormalsPtr;
    typedef std::shared_ptr<class GaussianImplicitSurface3DCombined>
        GaussianImplicitSurface3DCombinedPtr;
    typedef std::shared_ptr<class DataR3R1> DataR3R1Ptr;
    typedef std::shared_ptr<class DataR3R2> DataR3R2Ptr;
    typedef std::shared_ptr<class MarchingCubes> MarchingCubesPtr;
    typedef std::shared_ptr<class Bezier> BezierPtr;
    typedef std::shared_ptr<class LinearContinuedBezier> LinearContinuedBezierPtr;
    typedef std::shared_ptr<class Primitive> PrimitivePtr;
    typedef std::shared_ptr<struct Index3> Index3Ptr;
    typedef std::shared_ptr<class AbstractContactFeature> AbstractContactFeaturePtr;
    typedef std::shared_ptr<class BinContactFeature> BinContactFeaturePtr;
    template <class T>
    class Array3D;
    typedef std::shared_ptr<Array3D<float>> Array3DFloatPtr;
    typedef std::shared_ptr<Array3D<bool>> Array3DBoolPtr;
    //typedef std::shared_ptr<Array3D<>> Array3DPtr<T>;
    typedef std::shared_ptr<class VoronoiWeights> VoronoiWeightsPtr;
    struct WeightedFloatAverage;
    class WeightedVec3Average;

    typedef std::shared_ptr<class Grid3D> Grid3DPtr;
    typedef std::shared_ptr<class GridCacheFloat3> GridCacheFloat3Ptr;
    typedef std::shared_ptr<class FeatureCluster> FeatureClusterPtr;
    typedef std::shared_ptr<class EdgeCluster> EdgeClusterPtr;
    typedef std::shared_ptr<class EdgePredictor> EdgePredictorPtr;
    typedef std::shared_ptr<class EdgeTracer> EdgeTracerPtr;
    typedef std::shared_ptr<std::vector<Eigen::Vector3f>> Vec3ListPtr;
    typedef std::shared_ptr<class Edge> EdgePtr;
    typedef std::shared_ptr<class EdgeFeature> EdgeFeaturePtr;


} // namespace math

namespace sim
{
    class Simulation;
    typedef std::shared_ptr<Simulation> SimulationPtr;
    class HapticExplorationData;
    typedef std::shared_ptr<HapticExplorationData> HapticExplorationDataPtr;

    namespace objects
    {

        class AbstractObject;
        typedef std::shared_ptr<AbstractObject> AbstractObjectPtr;
        class ImplicitObject;
        typedef std::shared_ptr<ImplicitObject> ImplicitObjectPtr;
        class InfiniteObject;
        typedef std::shared_ptr<InfiniteObject> InfiniteObjectPtr;
        class CompositeObject;
        typedef std::shared_ptr<CompositeObject> CompositeObjectPtr;
        class Sphere;
        typedef std::shared_ptr<Sphere> SpherePtr;
        class TriangleMeshObject;
        typedef std::shared_ptr<TriangleMeshObject> TriangleMeshObjectPtr;


    } // namespace objects
} // namespace sim

namespace explorationControllers
{

    class AbstractExplorationController;
    typedef std::shared_ptr<AbstractExplorationController> AbstractExplorationControllerPtr;
    class Heuristic;
    typedef std::shared_ptr<Heuristic> HeuristicPtr;
    class InitialApproach;
    typedef std::shared_ptr<InitialApproach> InitialApproachPtr;
    class LocalSearch;
    typedef std::shared_ptr<LocalSearch> LocalSearchPtr;
    class PossibleTarget;
    typedef std::shared_ptr<PossibleTarget> PossibleTargetPtr;
    class TrajectoryEdgeSearch;
    typedef std::shared_ptr<TrajectoryEdgeSearch> TrajectoryEdgeSearchPtr;
    class Target;
    typedef std::shared_ptr<Target> TargetPtr;

} // namespace explorationControllers
