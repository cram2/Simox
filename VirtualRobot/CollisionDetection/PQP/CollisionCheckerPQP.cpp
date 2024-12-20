
#include "CollisionCheckerPQP.h"

#include <VirtualRobot/Visualization/TriMeshModel.h>

#include "../../SceneObjectSet.h"
#include "../../VirtualRobotException.h"
#include "../CollisionChecker.h"
#include "../CollisionModel.h"
#include "CollisionModelPQP.h"
#include "PQP.h"

///
/// \brief Computes the intersection line between two triangles V and U
/// \param V0 Vertex 0 of triangle V
/// \param V1 Vertex 1 of triangle V
/// \param V2 Vertex 2 of triangle V
/// \param U0 Vertex 0 of triangle U
/// \param U1 Vertex 1 of triangle U
/// \param U2 Vertex 2 of triangle U
/// \param coplanar Output: is set to 1 if the triangles are coplanar, 0 otherwise.
/// \param isectpt1 Output: start point of intersection line
/// \param isectpt2 Output: end point of intersection line
/// \return
///
int tri_tri_intersect_with_isectline(const float V0[3],
                                     const float V1[3],
                                     const float V2[3],
                                     const float U0[3],
                                     const float U1[3],
                                     const float U2[3],
                                     int* coplanar,
                                     float isectpt1[3],
                                     float isectpt2[3]);

namespace VirtualRobot
{
    struct TriTriIntersection
    {
        Eigen::Vector3f startPoint = Eigen::Vector3f::Zero();
        Eigen::Vector3f endPoint = Eigen::Vector3f::Zero();
        bool intersect = false;
        bool coplanar = false;
    };

    static TriTriIntersection
    intersectTriangles(Eigen::Vector3f const& U0,
                       Eigen::Vector3f const& U1,
                       Eigen::Vector3f const& U2,
                       Eigen::Vector3f const& V0,
                       Eigen::Vector3f const& V1,
                       Eigen::Vector3f const& V2)
    {
        TriTriIntersection result;

        int coplanar = 0;
        int intersects = tri_tri_intersect_with_isectline(V0.data(),
                                                          V1.data(),
                                                          V2.data(),
                                                          U0.data(),
                                                          U1.data(),
                                                          U2.data(),
                                                          &coplanar,
                                                          result.startPoint.data(),
                                                          result.endPoint.data());

        result.intersect = (intersects != 0);
        result.coplanar = (coplanar != 0);

        return result;
    }

    //----------------------------------------------------------------------
    // class CollisionChecker constructor
    //----------------------------------------------------------------------
    CollisionCheckerPQP::CollisionCheckerPQP() : CollisionCheckerImplementation()
    {
        automaticSizeCheck = true;

        PQP::PQP_REAL a[3] = {0.0f, 0.0f, 0.00001f};
        PQP::PQP_REAL b[3] = {0.0f, 0.00001f, 0.00001f};
        PQP::PQP_REAL c[3] = {0.0f, 0.00001f, 0.0f};


        pointModel.reset(new PQP::PQP_Model());
        pointModel->BeginModel();
        pointModel->AddTri(a, b, c, 99999998);
        pointModel->EndModel();
        pqpChecker = new PQP::PQP_Checker();
    }

    //----------------------------------------------------------------------
    // class CollisionChecker destructor
    //----------------------------------------------------------------------
    CollisionCheckerPQP::~CollisionCheckerPQP()
    {
        delete pqpChecker;
        pqpChecker = nullptr;
    }

    float
    CollisionCheckerPQP::calculateDistance(const CollisionModelPtr& model1,
                                           const CollisionModelPtr& model2,
                                           Eigen::Vector3f& P1,
                                           Eigen::Vector3f& P2,
                                           int* trID1,
                                           int* trID2)
    {
        std::shared_ptr<PQP::PQP_Model> m1 =
            model1->getCollisionModelImplementation()->getPQPModel();
        std::shared_ptr<PQP::PQP_Model> m2 =
            model2->getCollisionModelImplementation()->getPQPModel();
        VR_ASSERT_MESSAGE(m1 && m2, "NULL data in ColChecker!");

        float res = getMinDistance(m1,
                                   m2,
                                   model1->getCollisionModelImplementation()->getGlobalPose(),
                                   model2->getCollisionModelImplementation()->getGlobalPose(),
                                   P1,
                                   P2,
                                   trID1,
                                   trID2);

        return res;
    }

#define __convEigen2Ar(sb, rot, tr)                                                                \
    rot[0][0] = sb(0, 0);                                                                          \
    rot[0][1] = sb(0, 1);                                                                          \
    rot[0][2] = sb(0, 2);                                                                          \
    tr[0] = sb(0, 3);                                                                              \
    rot[1][0] = sb(1, 0);                                                                          \
    rot[1][1] = sb(1, 1);                                                                          \
    rot[1][2] = sb(1, 2);                                                                          \
    tr[1] = sb(1, 3);                                                                              \
    rot[2][0] = sb(2, 0);                                                                          \
    rot[2][1] = sb(2, 1);                                                                          \
    rot[2][2] = sb(2, 2);                                                                          \
    tr[2] = sb(2, 3);

    bool
    CollisionCheckerPQP::checkCollision(
        const CollisionModelPtr& model1,
        const CollisionModelPtr& model2) //, Eigen::Vector3f *storeContact)
    {
        THROW_VR_EXCEPTION_IF(!model1, "model1 is null");
        THROW_VR_EXCEPTION_IF(!model2, "model2 is null");
        const auto& Impl1 = model1->getCollisionModelImplementation();
        const auto& Impl2 = model2->getCollisionModelImplementation();
        THROW_VR_EXCEPTION_IF(!Impl1, "Impl1 of model1 is null");
        THROW_VR_EXCEPTION_IF(!Impl2, "Impl2 of model2 is null");
        const std::shared_ptr<PQP::PQP_Model>& m1 = Impl1->getPQPModel();
        const std::shared_ptr<PQP::PQP_Model>& m2 = Impl2->getPQPModel();
        THROW_VR_EXCEPTION_IF(!m1, "NULL data in ColChecker in m1!");
        THROW_VR_EXCEPTION_IF(!m2, "NULL data in ColChecker in m2!");

        PQP::PQP_CollideResult result;
        PQP::PQP_REAL R1[3][3];
        PQP::PQP_REAL T1[3];
        PQP::PQP_REAL R2[3][3];
        PQP::PQP_REAL T2[3];
        const auto& P1 = Impl1->getGlobalPose();
        const auto& P2 = Impl2->getGlobalPose();
        __convEigen2Ar(P1, R1, T1);
        __convEigen2Ar(P2, R2, T2);
        pqpChecker->PQP_Collide(
            &result, R1, T1, m1.get(), R2, T2, m2.get(), PQP::PQP_FIRST_CONTACT);

        /*if (storeContact && result.Colliding()>0)
        {
        // currently the ID is used to identify the VR objects.
        // Either use ID in the way pqp is expecting it (->AddTringale with id == nr of triangle)
        // or store extra data with PQP_CollideResult->Add()
        // Now the call to result.Id1(u) will result in an error
            // take first contact
            int u = 0;
            (*storeContact)[0] = (m1.get()->tris[result.Id1(u)].p1[0] + m1.get()->tris[result.Id1(u)].p2[0] + m1.get()->tris[result.Id1(u)].p3[0]) / 3.0f;
            (*storeContact)[1] = (m1.get()->tris[result.Id1(u)].p1[1] + m1.get()->tris[result.Id1(u)].p2[1] + m1.get()->tris[result.Id1(u)].p3[1]) / 3.0f;
            (*storeContact)[2] = (m1.get()->tris[result.Id1(u)].p1[2] + m1.get()->tris[result.Id1(u)].p2[2] + m1.get()->tris[result.Id1(u)].p3[2]) / 3.0f;

            // local ? -> to Global Pose
            Eigen::Matrix4f t;
            t.setIdentity();
            t.block(0,3,3,1)=(*storeContact);
            t = model1->getCollisionModelImplementation()->getGlobalPose() * t;
            (*storeContact) = t.block(0,3,3,1);
        }*/

        return result.Colliding() != 0;
    }

    bool
    CollisionCheckerPQP::checkCollision(const CollisionModelPtr& model1,
                                        const Eigen::Vector3f& point,
                                        float tolerance)
    {
        assert(model1);
        assert(model1->getCollisionModelImplementation());
        std::shared_ptr<PQP::PQP_Model> m1 =
            model1->getCollisionModelImplementation()->getPQPModel();
        VR_ASSERT_MESSAGE(m1, "NULL data in ColChecker!");

        PQP::PQP_REAL R1[3][3];
        PQP::PQP_REAL T1[3];
        PQP::PQP_REAL R2[3][3];
        PQP::PQP_REAL T2[3];
        Eigen::Matrix4f pointPose = Eigen::Matrix4f::Identity();
        pointPose.block<3, 1>(0, 3) = point;
        __convEigen2Ar(model1->getCollisionModelImplementation()->getGlobalPose(), R1, T1);
        __convEigen2Ar(pointPose, R2, T2);
        if (tolerance == 0.0f)
        {
            PQP::PQP_CollideResult result;

            pqpChecker->PQP_Collide(
                &result, R1, T1, m1.get(), R2, T2, pointModel.get(), PQP::PQP_FIRST_CONTACT);
            return ((bool)(result.Colliding() != 0));
        }
        else
        {
            PQP::PQP_ToleranceResult result;
            pqpChecker->PQP_Tolerance(&result, R1, T1, m1.get(), R2, T2, pointModel.get(), 50);


            return ((bool)(result.CloserThanTolerance() != 0));
        }
    }

    MultiCollisionResult
    CollisionCheckerPQP::checkMultipleCollisions(const CollisionModelPtr& model1,
                                                 const CollisionModelPtr& model2)
    {
        assert(model1);
        assert(model1->getCollisionModelImplementation());
        assert(model2);
        assert(model2->getCollisionModelImplementation());
        std::shared_ptr<PQP::PQP_Model> m1 =
            model1->getCollisionModelImplementation()->getPQPModel();
        std::shared_ptr<PQP::PQP_Model> m2 =
            model2->getCollisionModelImplementation()->getPQPModel();

        VR_ASSERT_MESSAGE(m1, "NULL data in ColChecker!");
        VR_ASSERT_MESSAGE(m2, "NULL data in ColChecker!");

        PQP::PQP_REAL R1[3][3];
        PQP::PQP_REAL T1[3];
        PQP::PQP_REAL R2[3][3];
        PQP::PQP_REAL T2[3];

        __convEigen2Ar(model1->getGlobalPose(), R1, T1);
        __convEigen2Ar(model2->getGlobalPose(), R2, T2);

        PQP::PQP_CollideResult pqpResult;
        pqpChecker->PQP_Collide(
            &pqpResult, R1, T1, m1.get(), R2, T2, m2.get(), PQP::PQP_ALL_CONTACTS);

        int collisionCount = pqpResult.NumPairs();

        Eigen::Affine3f pose1(model1->getGlobalPose());
        Eigen::Affine3f pose2(model2->getGlobalPose());
        auto& faces1 = model1->getTriMeshModel()->faces;
        auto& faces2 = model2->getTriMeshModel()->faces;
        auto& vertices1 = model1->getTriMeshModel()->vertices;
        auto& vertices2 = model2->getTriMeshModel()->vertices;

        MultiCollisionResult result;
        for (int i = 0; i < collisionCount; ++i)
        {
            SingleCollisionPair pair;
            pair.id1 = pqpResult.Id1(i);
            pair.id2 = pqpResult.Id2(i);

            // Get the triangles and find the intersection line
            VirtualRobot::MathTools::TriangleFace const& faceU = faces1.at(pair.id1);
            Eigen::Vector3f U1 = pose1 * vertices1.at(faceU.id1);
            Eigen::Vector3f U2 = pose1 * vertices1.at(faceU.id2);
            Eigen::Vector3f U3 = pose1 * vertices1.at(faceU.id3);

            VirtualRobot::MathTools::TriangleFace const& faceV = faces2.at(pair.id2);
            Eigen::Vector3f V1 = pose2 * vertices2.at(faceV.id1);
            Eigen::Vector3f V2 = pose2 * vertices2.at(faceV.id2);
            Eigen::Vector3f V3 = pose2 * vertices2.at(faceV.id3);

            TriTriIntersection intersection = intersectTriangles(U1, U2, U3, V1, V2, V3);
            if (intersection.intersect)
            {
                pair.contact1 = intersection.startPoint;
                pair.contact2 = intersection.endPoint;

                result.pairs.push_back(pair);
            }
        }
        return result;
    }

    //
    // bool CollisionCheckerPQP::GetAllCollisonTriangles (SceneObjectSetPtr model1, SceneObjectSetPtr model2, std::vector<int> &storePairs)
    // {
    //  if (model1==NULL || model2==NULL)
    //  {
    //      printf ("CollisionCheckerPQP:GetAllCollisonTriangles - NULL data...\n");
    //      return false;
    //  }
    //  if (model1->GetCollisionChecker() != model2->GetCollisionChecker() || model1->GetCollisionChecker()->getCollisionCheckerImplementation()!=this)
    //  {
    //      printf ("CollisionCheckerPQP:GetAllCollisonTriangles - Could not go on, collision models are linked to different Collision Checker instances...\n");
    //      return false;
    //  }
    //  std::vector<CollisionModel*> vColModels1 = model1->getCollisionModelSetImplementation()->GetCollisionModels();
    //  std::vector<CollisionModel*> vColModels2 = model2->getCollisionModelSetImplementation()->GetCollisionModels();
    //  if (vColModels1.size()==0 || vColModels2.size()==0)
    //  {
    //      printf ("CollisionCheckerPQP:GetAllCollisonTriangles - NULL internal data...\n");
    //      return false;
    //  }
    //  std::vector<CollisionModel*>::iterator it1 = vColModels1.begin();
    //  bool bRes = false;
    //  while (it1!=vColModels1.end())
    //  {
    //      std::vector<CollisionModel*>::iterator it2 = vColModels2.begin();
    //      while (it2!=vColModels2.end())
    //      {
    //          if (!bRes)
    //              bRes = GetAllCollisonTriangles(*it1,*it2,storePairs);
    //          it2++;
    //      }
    //      it1++;
    //  }
    //  return bRes;
    // }
    //
    // bool CollisionCheckerPQP::GetAllCollisonTriangles (CollisionModelPtr model1, CollisionModelPtr model2, std::vector<int> &storePairs)
    // {
    //  if (model1==NULL || model2==NULL)
    //  {
    //      printf ("CollisionCheckerPQP:GetAllCollisonTriangles - NULL data...\n");
    //      return false;
    //  }
    //  if (model1->GetCollisionChecker() != model2->GetCollisionChecker() || model1->GetCollisionChecker()->getCollisionCheckerImplementation()!=this)
    //  {
    //      printf ("CollisionCheckerPQP:GetAllCollisonTriangles - Could not go on, collision models are linked to different Collision Checker instances...\n");
    //      return false;
    //  }
    //
    //  PQP::PQP_ModelPtr m1 = model1->getCollisionModelImplementation()->GetModel();
    //  PQP::PQP_ModelPtr m2 = model2->getCollisionModelImplementation()->GetModel();
    //  if (m1==NULL || m2==NULL)
    //  {
    //      printf ("CollisionCheckerPQP:GetAllCollisonTriangles - NULL internal data...\n");
    //      return false;
    //  }
    //
    //  // todo: this can be optimized
    //  PQP_CollideResult result;
    //  PQP::PQP_REAL R1[3][3];
    //  PQP::PQP_REAL T1[3];
    //  PQP::PQP_REAL R2[3][3];
    //  PQP::PQP_REAL T2[3];
    //  __convSb2Ar(model1->getCollisionModelImplementation()->getGlobalPose(),R1,T1);
    //  __convSb2Ar(model2->getCollisionModelImplementation()->getGlobalPose(),R2,T2);
    //  PQP_Collide(&result,
    //      R1, T1, m1,
    //      R2, T2, m2,
    //      PQP_ALL_CONTACTS);

    //
    //  //if there is a collision, then add the pair
    //  // to the collision report database.
    //  if (!result.Colliding())
    //      return false;
    //  for (int u=0; u<result.NumPairs(); u++)
    //  {
    //      storePairs.push_back(result.Id1(u));
    //      storePairs.push_back(result.Id2(u));
    //      //printf ("PQP: COLLIDING pair %d: id1:%d, id2: %d\n",u,result.Id1(u),result.Id2(u));
    //  }
    //  return true;
    // }
    //


    // returns min distance between the objects
    float
    CollisionCheckerPQP::getMinDistance(std::shared_ptr<PQP::PQP_Model> m1,
                                        std::shared_ptr<PQP::PQP_Model> m2,
                                        const Eigen::Matrix4f& mat1,
                                        const Eigen::Matrix4f& mat2)
    {
        PQP::PQP_DistanceResult result;
        CollisionCheckerPQP::GetPQPDistance(m1, m2, mat1, mat2, result);

        return (float)result.Distance();
    }

    // returns min distance between the objects
    float
    CollisionCheckerPQP::getMinDistance(std::shared_ptr<PQP::PQP_Model> m1,
                                        std::shared_ptr<PQP::PQP_Model> m2,
                                        const Eigen::Matrix4f& mat1,
                                        const Eigen::Matrix4f& mat2,
                                        Eigen::Vector3f& storeP1,
                                        Eigen::Vector3f& storeP2,
                                        int* storeID1,
                                        int* storeID2)
    {
        VR_ASSERT_MESSAGE(m1 && m2, "NULL data in ColChecker!");

        PQP::PQP_DistanceResult result;

        CollisionCheckerPQP::GetPQPDistance(m1, m2, mat1, mat2, result);

        storeP1[0] = (float)result.P1()[0];
        storeP1[1] = (float)result.P1()[1];
        storeP1[2] = (float)result.P1()[2];
        storeP2[0] = (float)result.P2()[0];
        storeP2[1] = (float)result.P2()[1];
        storeP2[2] = (float)result.P2()[2];

        // added code to store iDs to original PQP sources, if you get an error here, disable the ID query
        if (storeID1)
        {
            *storeID1 = result.P1_ID();
        }

        if (storeID2)
        {
            *storeID2 = result.P2_ID();
        }

        //////////////////////////////

        return (float)result.Distance();
    }

    void
    CollisionCheckerPQP::GetPQPDistance(const std::shared_ptr<PQP::PQP_Model>& model1,
                                        const std::shared_ptr<PQP::PQP_Model>& model2,
                                        const Eigen::Matrix4f& matrix1,
                                        const Eigen::Matrix4f& matrix2,
                                        PQP::PQP_DistanceResult& pqpResult)
    {
        VR_ASSERT_MESSAGE(pqpChecker, "NULL data in ColChecker!");


        PQP::PQP_REAL Rotation1[3][3];
        PQP::PQP_REAL Translation1[3];
        PQP::PQP_REAL Rotation2[3][3];
        PQP::PQP_REAL Translation2[3];
        __convEigen2Ar(matrix1, Rotation1, Translation1);
        __convEigen2Ar(matrix2, Rotation2, Translation2);
        pqpChecker->PQP_Distance(&pqpResult,
                                 Rotation1,
                                 Translation1,
                                 model1.get(),
                                 Rotation2,
                                 Translation2,
                                 model2.get(),
                                 0,
                                 0); // default: 0 error
    }

} // namespace VirtualRobot
