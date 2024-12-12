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
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include <VirtualRobot/VirtualRobotImportExport.h>
#include <VirtualRobot/Logging.h> // convenience include for logging. Will be removed in the future.
#include <VirtualRobot/Assert.h> // convenience include for assertions. Will be removed in the future.

/*! \defgroup VirtualRobot The VirtualRobot Library
* With the VirtualRobot library you can define complex robot structures,
* perform collision detection, visualize robots and environments, do reachability analysis and generic IK solvers are provided.
*/



/** \mainpage Simox: A simulation, motion and grasp planning toolbox.

  \section Introduction Introduction

  The aim of the lightweight platform independent C++ toolbox \b Simox is to provide a set of
  algorithms for 3D simulation of robot systems, sampling based motion planning and grasp
  planning. Simox consists of three libraries (VirtualRobot, Saba and GraspStudio) and numerous
  examples showing how these libraries can be used to build complex tools in the
  context of mobile manipulation.

  Further information and documentation can be found at the wiki pages: https://git.h2t.iar.kit.edu/sw/simox/simox/-/wikis/home

  \section SimoxUtility SimoxUtility

  \ref simox "SimoxUtility"
  is a utility library providing general-purpose code tools to help working with,
  among others,
  C++ strings and containers,
  maths (e.g. pose, radians and degrees, scaling, clamping, periodic mean, ...),
  data file formats (e.g. JSON, XMl),
  colors and colormaps,
  as well as
  shapes like axis-aligned and oriented (bounding) boxes.



  \section VirtualRobot VirtualRobot

  The library \b VirtualRobot can be used to define complex
  robot systems, which may cover multiple robots with many degrees of freedom. The robot
  structure and its visualization can be easily defined via XML files and environments with
  obstacles and objects to manipulate are supported. Further, basic robot simulation components,
  as Jacobian computations and generic Inverse Kinematics (IK) solvers, are offered by
  the library. Beyond that, extended features like tools for analyzing the reachable workspace
  for robotic manipulators or contact determination for grasping are included.
  \image html VR.png

  \section Saba Motion Planning

  With \b Saba, a library for planning collision-free motions is offered, which directly incorporates
  with the data provided by VirtualRobot. The algorithms cover state-of-the-art implementations
  of sampling-based motion planning approaches (e.g. Rapidly-exploring Random Trees)
  and interfaces that allow to conveniently implement own planners. Since Saba was designed
  for planning in high-dimensional configuration spaces, complex planning problems for robots
  with a high number of degrees of freedom (DoF) can be solved efficiently.

  \image html Saba.png

  \section GraspStudio Grasp Planning

  \b GraspStudio offers possibilities to compute the grasp quality for generic end-effector definitions,
  e.g. a humanoid hand. The implemented 6D wrench-space computations can be used
  to easily (and quickly) determine the quality of an applied grasp to an object. Furthermore,
  the implemented planners are able to generate grasp maps for given objects automatically.

  \image html GraspStudio.png

  \section Wiki Installation, tutorials and documentation

  Since complex frameworks have to incorporate with several libraries in order to provide full
  functionality, several issues may arise when setting up the environment, such as dependency
  problems, incompatible library versions or even non-existing ports of needed libraries for the
  used operating systems. Hence, only a limited set of libraries are used by the Simox core in
  order to make it compile. Extended functionality (e.g. visualization) can be turned off in
  order to allow Simox compiling on most platforms. Further dependencies are encapsulated
  with interfaces, making it easy to exchange e.g. the collision engine or the visualization
  functionality. As a reference implementation Simox offers Coin3D/SoQt-based visualization
  support.

  Please have a look at the wiki pages: https://git.h2t.iar.kit.edu/sw/simox/simox/-/wikis/home
 *
 */

// include compile time defines, generated by cmake
//#include "definesVR.h"
// for now we know that PQP is used- ToDo: Change CollisionChecker implementation, use AbstractFactoryMethods
#define VR_COLLISION_DETECTION_PQP


#ifdef WIN32
// needed to have M_PI etc defined
#if !defined(_USE_MATH_DEFINES)
#define _USE_MATH_DEFINES
#endif

#endif


#include <memory>

#include <Eigen/Core>


namespace VirtualRobot
{

    class CoMIK;
    class DifferentialIK;
    class HierarchicalIK;
    class Constraint;
    class TSRConstraint;
    class BalanceConstraint;
    class PoseConstraint;
    class PositionConstraint;
    class OrientationConstraint;
    class SupportPolygon;
    class DHParameter;
    class RobotNode;
    class RobotNodeRevolute;
    class RobotNodePrismatic;
    class RobotNodeFixed;    

    class RobotNodeFactory;
    class RobotNodeSet;
    class KinematicChain;
    class Robot;
    class EndEffector;
    class EndEffectorActor;
    class CollisionChecker;
    class CollisionModel;
    class SceneObjectSet;
    class TriMeshModel;
    class SceneObject;
    class Obstacle;
    class Visualization;
    class VisualizationNode;
    class VisualizationFactory;
    class Scene;
    class RobotConfig;
    class Grasp;
    class ChainedGrasp;
    class GraspSet;
    class GraspableSensorizedObject;
    class ManipulationObject;
    class CDManager;
    class Reachability;
    class WorkspaceRepresentation;
    class WorkspaceData;
    class PoseQualityMeasurement;
    class PoseQualityManipulability;
    class Trajectory;
    class SphereApproximator;
    class BasicGraspQualityMeasure;
    class WorkspaceGrid;
    class WorkspaceDataArray;
    class ForceTorqueSensor;
    class ContactSensor;
    class Sensor;
    class LocalRobot;
    class Color;
    class BoundingBox;
    class GazeIK;

    using CoMIKPtr = std::shared_ptr<CoMIK>;
    using HierarchicalIKPtr = std::shared_ptr<HierarchicalIK>;
    using DifferentialIKPtr = std::shared_ptr<DifferentialIK>;
    using ConstraintPtr = std::shared_ptr<Constraint>;
    using TSRConstraintPtr = std::shared_ptr<TSRConstraint>;
    using BalanceConstraintPtr = std::shared_ptr<BalanceConstraint>;
    using PoseConstraintPtr = std::shared_ptr<PoseConstraint>;
    using PositionConstraintPtr = std::shared_ptr<PositionConstraint>;
    using OrientationConstraintPtr = std::shared_ptr<OrientationConstraint>;
    using RobotNodePtr = std::shared_ptr<RobotNode>;
    using SupportPolygonPtr = std::shared_ptr<SupportPolygon>;
    using RobotNodeRevolutePtr = std::shared_ptr<RobotNodeRevolute>;
    using RobotNodePrismaticPtr = std::shared_ptr<RobotNodePrismatic>;
    using RobotNodeSetPtr = std::shared_ptr<RobotNodeSet>;
    using KinematicChainPtr = std::shared_ptr<KinematicChain>;
    using RobotNodeWeakPtr = std::weak_ptr<RobotNode>;
    using RobotNodeFactoryPtr = std::shared_ptr<RobotNodeFactory>;
    using RobotPtr = std::shared_ptr<Robot>;
    using RobotWeakPtr = std::weak_ptr<Robot>;
    using EndEffectorPtr = std::shared_ptr<EndEffector>;
    using EndEffectorActorPtr = std::shared_ptr<EndEffectorActor>;
    using CollisionModelPtr = std::shared_ptr<CollisionModel>;
    using CollisionCheckerPtr = std::shared_ptr<CollisionChecker>;
    using SceneObjectSetPtr = std::shared_ptr<SceneObjectSet>;
    using TriMeshModelPtr = std::shared_ptr<TriMeshModel>;
    using SceneObjectPtr = std::shared_ptr<SceneObject>;
    using SceneObjectWeakPtr = std::weak_ptr<SceneObject>;
    using ObstaclePtr = std::shared_ptr<Obstacle>;
    using VisualizationPtr = std::shared_ptr<Visualization>;
    using VisualizationNodePtr = std::shared_ptr<VisualizationNode>;
    using VisualizationFactoryPtr = std::shared_ptr<VisualizationFactory>;
    using WorkspaceDataPtr = std::shared_ptr<WorkspaceData>;
    using WorkspaceDataArrayPtr = std::shared_ptr<WorkspaceDataArray>;
    using WorkspaceRepresentationPtr = std::shared_ptr<WorkspaceRepresentation>;
    using ReachabilityPtr = std::shared_ptr<Reachability>;
    using ScenePtr = std::shared_ptr<Scene>;
    using RobotConfigPtr = std::shared_ptr<RobotConfig>;
    using GraspPtr = std::shared_ptr<Grasp>;
    using ChainedGraspPtr = std::shared_ptr<ChainedGrasp>;
    using GraspSetPtr = std::shared_ptr<GraspSet>;
    using GraspableSensorizedObjectPtr = std::shared_ptr<GraspableSensorizedObject>;
    using GraspableSensorizedObjectWeakPtr = std::weak_ptr<GraspableSensorizedObject>;
    using ManipulationObjectPtr = std::shared_ptr<ManipulationObject>;
    using CDManagerPtr = std::shared_ptr<CDManager>;
    using PoseQualityMeasurementPtr = std::shared_ptr<PoseQualityMeasurement>;
    using PoseQualityManipulabilityPtr = std::shared_ptr<PoseQualityManipulability>;
    using TrajectoryPtr = std::shared_ptr<Trajectory>;
    using SphereApproximatorPtr = std::shared_ptr<SphereApproximator>;
    using BasicGraspQualityMeasurePtr = std::shared_ptr<BasicGraspQualityMeasure>;
    using WorkspaceGridPtr = std::shared_ptr<WorkspaceGrid>;
    using ForceTorqueSensorPtr = std::shared_ptr<ForceTorqueSensor>;
    using ContactSensorPtr = std::shared_ptr<ContactSensor>;
    using SensorPtr = std::shared_ptr<Sensor>;
    using LocalRobotPtr = std::shared_ptr<LocalRobot>;

    /*
     * Predefine for MathTools.h
     */
    namespace MathTools
    {
        struct Quaternion;
        struct SphericalCoord;
        struct Segment2D;
        struct ConvexHull2D;
        struct ConvexHull3D;
        struct ConvexHull6D;
        struct Plane;
        template<typename VectorT> struct BaseLine;
        struct Segment;
        struct OOBB;
        struct ContactPoint;
        struct TriangleFace;
        struct TriangleFace6D;

        typedef BaseLine<Eigen::Vector3f> Line;
        typedef BaseLine<Eigen::Vector2f> Line2D;
        typedef std::shared_ptr<ConvexHull2D> ConvexHull2DPtr;
        typedef std::shared_ptr<ConvexHull3D> ConvexHull3DPtr;
        typedef std::shared_ptr<ConvexHull6D> ConvexHull6DPtr;

   }


    /*!
    Initialize the runtime envionment. This method calls VisualizationFactory::init().
    */
    void VIRTUAL_ROBOT_IMPORT_EXPORT init(int &argc, char* argv[], const std::string &appName);
    void VIRTUAL_ROBOT_IMPORT_EXPORT init(const std::string &appName);

    // init method is storing appName, since the c_string is passed by refrence to QT -> we must ensure that the string stays alive
    VIRTUAL_ROBOT_IMPORT_EXPORT extern std::string globalAppName;

} // namespace
