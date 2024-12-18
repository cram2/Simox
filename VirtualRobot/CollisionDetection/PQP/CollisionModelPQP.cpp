
#include "CollisionModelPQP.h"

#include "../CollisionChecker.h"
#include "CollisionCheckerPQP.h"
#include "../../Visualization/TriMeshModel.h"
#include "Logging.h"
#include "PQP.h"

namespace VirtualRobot
{

    CollisionModelPQP::CollisionModelPQP(const TriMeshModelPtr& modelData, CollisionCheckerPtr colChecker, int id)
        : CollisionModelImplementation(modelData, colChecker, id)
    {
        if (!colChecker)
        {
            colChecker = CollisionChecker::getGlobalCollisionChecker();
        }

        if (!colChecker)
        {
            VR_WARNING << "no col checker...\n";
        }
        else
        {
            colCheckerPQP = colChecker->getCollisionCheckerImplementation();
        }

        createPQPModel();
    }

    CollisionModelPQP::CollisionModelPQP(const CollisionModelPQP &orig) :
        CollisionModelImplementation(orig.modelData, nullptr, orig.id)
    {
        pqpModel = orig.pqpModel;
    }


    CollisionModelPQP::~CollisionModelPQP()
    = default;


    void CollisionModelPQP::destroyData()
    {
        pqpModel.reset();
    }

    void CollisionModelPQP::createPQPModel()
    {
        if (!modelData)
        {
            VR_WARNING << "no model data in PQP!\n";
            return;
        }

        PQP::PQP_REAL a[3];
        PQP::PQP_REAL b[3];
        PQP::PQP_REAL c[3];

        pqpModel.reset(new PQP::PQP_Model());
        pqpModel->BeginModel(modelData->faces.size());

        // If no external ID is set, we use the id as triangle face index
        bool useIndividualID = (id == 666);

        for (unsigned int i = 0; i < modelData->faces.size(); i++)
        {
            a[0] = modelData->vertices[modelData->faces[i].id1][0];
            a[1] = modelData->vertices[modelData->faces[i].id1][1];
            a[2] = modelData->vertices[modelData->faces[i].id1][2];
            b[0] = modelData->vertices[modelData->faces[i].id2][0];
            b[1] = modelData->vertices[modelData->faces[i].id2][1];
            b[2] = modelData->vertices[modelData->faces[i].id2][2];
            c[0] = modelData->vertices[modelData->faces[i].id3][0];
            c[1] = modelData->vertices[modelData->faces[i].id3][1];
            c[2] = modelData->vertices[modelData->faces[i].id3][2];
            int storeID = useIndividualID ? i : this->id;
            pqpModel->AddTri(a, b, c, storeID);
        }

        pqpModel->EndModel();
    }

    void CollisionModelPQP::print()
    {
        std::cout << "   CollisionModelPQP: ";

        if (pqpModel)
        {
            std::cout << pqpModel->num_tris << " triangles.\n";
            float mi[3];
            float ma[3];

            if (pqpModel->num_tris > 0)
            {
                for (int j = 0; j < 3; j++)
                {
                    mi[j] = ma[j] = pqpModel->tris[0].p1[j];
                }

                for (int i = 0; i < pqpModel->num_tris; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        if (pqpModel->tris[i].p1[j] < mi[j])
                        {
                            mi[j] = pqpModel->tris[i].p1[j];
                        }

                        if (pqpModel->tris[i].p2[j] < mi[j])
                        {
                            mi[j] = pqpModel->tris[i].p2[j];
                        }

                        if (pqpModel->tris[i].p3[j] < mi[j])
                        {
                            mi[j] = pqpModel->tris[i].p3[j];
                        }

                        if (pqpModel->tris[i].p1[j] > ma[j])
                        {
                            ma[j] = pqpModel->tris[i].p1[j];
                        }

                        if (pqpModel->tris[i].p2[j] > ma[j])
                        {
                            ma[j] = pqpModel->tris[i].p2[j];
                        }

                        if (pqpModel->tris[i].p3[j] > ma[j])
                        {
                            ma[j] = pqpModel->tris[i].p3[j];
                        }
                    }
                }

                std::cout << "    Min point: (" << mi[0] << "," << mi[1] << "," << mi[2] << ")\n";
                std::cout << "    Max point: (" << ma[0] << "," << ma[1] << "," << ma[2] << ")\n";
            }
        }
        else
        {
            std::cout << "no model.\n";
        }

        std::cout << std::endl;
    }

    std::shared_ptr<CollisionModelImplementation> CollisionModelPQP::clone(bool deepCopy) const
    {
        std::shared_ptr<CollisionModelPQP> p(new CollisionModelPQP(*this));
        if(deepCopy)
        {
            p->createPQPModel();
        }

        VR_ASSERT(this->pqpModel);
        return p;
    }


} // namespace
