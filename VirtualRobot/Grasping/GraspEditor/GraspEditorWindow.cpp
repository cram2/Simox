#include "GraspEditorWindow.h"

#include <cmath>
#include <cstddef>
#include <cstring>
#include <ctime>
#include <iostream>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <QFileDialog>

#include <Eigen/Geometry>

#include <SimoxUtility/algorithm/string/string_tools.h>
#include <SimoxUtility/math/convert.h>

#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include "Logging.h"
#include "VirtualRobot.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Grasping/ChainedGrasp.h"
#include "VirtualRobot/Grasping/GraspSet.h"
#include "VirtualRobot/ManipulationObject.h"
#include "VirtualRobot/Nodes/RobotNodePrismatic.h"
#include "VirtualRobot/Nodes/RobotNodeRevolute.h"
#include "VirtualRobot/RobotFactory.h"
#include "VirtualRobot/SphereApproximator.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"
#include "VirtualRobot/Visualization/TriMeshModel.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include "VirtualRobot/XML/ObjectIO.h"
#include "VirtualRobot/XML/RobotIO.h"
#include "Visualization/CoinVisualization/CoinVisualizationNode.h"
#include "ui_GraspEditor.h"
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/sensors/SoTimerSensor.h>

//#undef SPNAV_AVAILABLE

#ifdef SPNAV_AVAILABLE
#include <spnav.h>
#endif // SPNAV_AVAILABLE


using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

#ifdef SPNAV_AVAILABLE

bool spacenav_is_open = false;
spnav_event sev;

#endif // SPNAV_AVAILABLE

constexpr double MIN_ROT_SCALING = 0.00002;
constexpr double MAX_ROT_SCALING = 0.0001;
constexpr double MIN_TRANS_SCALING = 0.002;
constexpr double MAX_TRANS_SCALING = 0.01;

double rotation_scaling = 0.5 * (MIN_ROT_SCALING + MAX_ROT_SCALING);
double translation_scaling = 0.5 * (MIN_TRANS_SCALING + MAX_TRANS_SCALING);

static const constexpr char* PREPOSE_SUFFIX = "_Prepose";
static const constexpr char* GRASP_OPTIONAL_SUFFIX = "_Grasp";

namespace VirtualRobot
{

    enum class ControlStyle
    {
        HAND_COORDINATE_SYSTEM,
        GLOBAL_COORDINATE_SYSTEM,
        CAMERA_COORDINATE_SYSTEM
    };

    GraspEditorWindow::GraspEditorWindow(std::string& objFile,
                                         std::string& robotFile,
                                         bool embeddedGraspEditor) :
        QMainWindow(nullptr), UI(new Ui::MainWindowGraspEditor)
    {
        VR_INFO << " start " << std::endl;

        // Indicates whether this program is started inside another extern program
        this->embeddedGraspEditor = embeddedGraspEditor;

        objectFile = objFile;
        this->robotFile = robotFile;

        sceneSep = new SoSeparator;
        sceneSep->ref();
        robotSep = new SoSeparator;
        objectSep = new SoSeparator;
        eefVisu = new SoSeparator;
        graspSetVisu = new SoSeparator;

        //sceneSep->addChild(robotSep);

        sceneSep->addChild(eefVisu);
        sceneSep->addChild(objectSep);
        sceneSep->addChild(graspSetVisu);

        setupUI();

        if (objectFile.empty())
            objectFile = settings.value("object/path", "").toString().toStdString();
        loadObject();
        if (robotFile.empty())
            this->robotFile = settings.value("robot/path", "").toString().toStdString();
        loadRobot();

        m_pExViewer->viewAll();

        SoSensorManager* sensor_mgr = SoDB::getSensorManager();
        timer = new SoTimerSensor(timerCB, this);
        timer->setInterval(SbTime(TIMER_MS / 1000.0f));
        sensor_mgr->insertTimerSensor(timer);
    }

    GraspEditorWindow::~GraspEditorWindow()
    {
        timer->unschedule();
        delete m_pExViewer;
        delete UI;
        sceneSep->unref();
    }

    void
    GraspEditorWindow::timerCB(void* data, SoSensor* /*sensor*/)
    {
        GraspEditorWindow* ikWindow = static_cast<GraspEditorWindow*>(data);
        float x[6];
        x[0] = -(float)ikWindow->UI->horizontalSliderX->value();
        x[1] = -(float)ikWindow->UI->horizontalSliderY->value();
        x[2] = -(float)ikWindow->UI->horizontalSliderZ->value();
        x[3] = -(float)ikWindow->UI->horizontalSliderRo->value();
        x[4] = -(float)ikWindow->UI->horizontalSliderPi->value();
        x[5] = -(float)ikWindow->UI->horizontalSliderYa->value();
        x[0] /= 10.0f;
        x[1] /= 10.0f;
        x[2] /= 10.0f;
        x[3] /= 300.0f;
        x[4] /= 300.0f;
        x[5] /= 300.0f;

        if (x[0] != 0 || x[1] != 0 || x[2] != 0 || x[3] != 0 || x[4] != 0 || x[5] != 0)
        {
            ikWindow->updateEEF(x);
        }

#ifdef SPNAV_AVAILABLE

        if (!spacenav_is_open)
        {
            if (spnav_open() == -1)
            {
                cout << "Could not open the space navigator device. "
                        "Did you remember to run spacenavd (as root)?"
                     << std::endl;
                return;
            }
            else
            {
                spacenav_is_open = true;
            }
        }

        switch (spnav_poll_event(&sev))
        {
            case 0:
                // No event in queue
                break;

            case SPNAV_EVENT_MOTION:

                // An "intuitive" mapping for the hand coordinate system is [y, x, -z]
                x[0] = sev.motion.y * translation_scaling;
                x[1] = sev.motion.x * translation_scaling;
                x[2] = -sev.motion.z * translation_scaling;

                x[3] = sev.motion.ry * rotation_scaling;
                x[4] = sev.motion.rx * rotation_scaling;
                x[5] = -sev.motion.rz * rotation_scaling;

                ikWindow->updateEEF(x);

                break;

            case SPNAV_EVENT_BUTTON:

                if (sev.button.bnum < 0)
                {
                    cout << "Negative spacenav buttons not supported." << endl;
                    break;
                }

                if (!sev.button.press) // Release event
                    break;

                switch (sev.button.bnum)
                {
                    case 0:
                        ikWindow->UI->sensitivityRot->setValue(
                            ikWindow->UI->sensitivityRot->value() - 10);
                        break;
                    case 1:
                        ikWindow->UI->sensitivityRot->setValue(
                            ikWindow->UI->sensitivityRot->value() + 10);
                        break;
                    case 13:
                        ikWindow->UI->sensitivityTrans->setValue(
                            ikWindow->UI->sensitivityTrans->value() - 10);
                        break;
                    case 12:
                        ikWindow->UI->sensitivityTrans->setValue(
                            ikWindow->UI->sensitivityTrans->value() + 10);
                        break;
                    case 6:
                        ikWindow->UI->disableRotation->setChecked(true);
                        ikWindow->UI->disableTranslation->setChecked(true);
                        break;
                    case 7:
                        ikWindow->UI->disableRotation->setChecked(true);
                        break;
                    case 8:
                        ikWindow->UI->disableTranslation->setChecked(false);
                        ikWindow->UI->disableRotation->setChecked(false);
                        break;
                    case 9:
                        ikWindow->UI->disableTranslation->setChecked(true);
                        break;
                }

                break;

            default:
                cout << "Unknown message type in spacenav. This should never happen." << std::endl;
                break;
        }

        // Get rid of remaining motion events to prevent queue from growing
        spnav_remove_events(SPNAV_EVENT_MOTION);

#endif // SPNAV_AVAILABLE
    }

    void
    GraspEditorWindow::setupUI()
    {
        UI->setupUi(this);
        m_pExViewer =
            new SoQtExaminerViewer(UI->frameViewer, "", TRUE, SoQtExaminerViewer::BUILD_POPUP);

        // setup
        m_pExViewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
        m_pExViewer->setAccumulationBuffer(false);

        m_pExViewer->setAntialiasing(true, 4);

        m_pExViewer->setGLRenderAction(new SoLineHighlightRenderAction);
        m_pExViewer->setTransparencyType(SoGLRenderAction::BLEND);
        m_pExViewer->setFeedbackVisibility(true);
        m_pExViewer->setSceneGraph(sceneSep);
        m_pExViewer->viewAll();

        connect(UI->pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
        connect(UI->pushButtonLoadObject, SIGNAL(clicked()), this, SLOT(selectObject()));
        connect(UI->pushButtonSave, SIGNAL(clicked()), this, SLOT(saveObject()));
        connect(UI->pushButtonClose, SIGNAL(clicked()), this, SLOT(closeEEF()));
        connect(UI->pushButtonOpen, SIGNAL(clicked()), this, SLOT(openEEF()));
        connect(UI->pushButtonLoadRobot, SIGNAL(clicked()), this, SLOT(selectRobot()));
        connect(UI->comboBoxObject, SIGNAL(activated(int)), this, SLOT(selectRobotObject(int)));
        connect(UI->comboBoxEEF, SIGNAL(activated(int)), this, SLOT(selectEEF(int)));
        connect(UI->comboBoxGrasp, SIGNAL(activated(int)), this, SLOT(selectGrasp(int)));
        connect(UI->pushButtonAddGrasp, SIGNAL(clicked()), this, SLOT(addGrasp()));
        connect(UI->pushButtonRenameGrasp, SIGNAL(clicked()), this, SLOT(renameGrasp()));
        connect(UI->checkBoxTCP, SIGNAL(clicked()), this, SLOT(buildVisu()));

        connect(
            UI->horizontalSliderX, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectX()));
        connect(
            UI->horizontalSliderY, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectY()));
        connect(
            UI->horizontalSliderZ, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectZ()));
        connect(
            UI->horizontalSliderRo, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectA()));
        connect(
            UI->horizontalSliderPi, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectB()));
        connect(
            UI->horizontalSliderYa, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectG()));
        connect(UI->minX, SIGNAL(valueChanged(double)), this, SLOT(virtualJointValueChanged()));
        connect(UI->maxX, SIGNAL(valueChanged(double)), this, SLOT(virtualJointValueChanged()));
        connect(UI->minY, SIGNAL(valueChanged(double)), this, SLOT(virtualJointValueChanged()));
        connect(UI->maxY, SIGNAL(valueChanged(double)), this, SLOT(virtualJointValueChanged()));
        connect(UI->minZ, SIGNAL(valueChanged(double)), this, SLOT(virtualJointValueChanged()));
        connect(UI->maxZ, SIGNAL(valueChanged(double)), this, SLOT(virtualJointValueChanged()));
        connect(UI->minRoll, SIGNAL(valueChanged(double)), this, SLOT(virtualJointValueChanged()));
        connect(UI->maxRoll, SIGNAL(valueChanged(double)), this, SLOT(virtualJointValueChanged()));
        connect(UI->minPitch, SIGNAL(valueChanged(double)), this, SLOT(virtualJointValueChanged()));
        connect(UI->maxPitch, SIGNAL(valueChanged(double)), this, SLOT(virtualJointValueChanged()));
        connect(UI->minYaw, SIGNAL(valueChanged(double)), this, SLOT(virtualJointValueChanged()));
        connect(UI->maxYaw, SIGNAL(valueChanged(double)), this, SLOT(virtualJointValueChanged()));
        connect(UI->checkBoxColModel, SIGNAL(clicked()), this, SLOT(buildVisu()));
        connect(UI->checkBoxGraspSet, SIGNAL(clicked()), this, SLOT(buildVisu()));
        connect(UI->grid, SIGNAL(valueChanged(int)), this, SLOT(sampleGrasps()));

        connect(UI->sensitivityRot,
                SIGNAL(valueChanged(int)),
                this,
                SLOT(updateRotationalSensitivity()));
        connect(UI->sensitivityTrans,
                SIGNAL(valueChanged(int)),
                this,
                SLOT(updateTranslationalSensitivity()));
        connect(UI->disableRotation,
                SIGNAL(stateChanged(int)),
                this,
                SLOT(updateRotationalSensitivity()));
        connect(UI->disableTranslation,
                SIGNAL(stateChanged(int)),
                this,
                SLOT(updateTranslationalSensitivity()));

        UI->controlStyle->addItem("Hand Coodinate System",
                                  static_cast<int>(ControlStyle::HAND_COORDINATE_SYSTEM));
        UI->controlStyle->addItem("Global Coordinate System",
                                  static_cast<int>(ControlStyle::GLOBAL_COORDINATE_SYSTEM));
        //UI->controlStyle->addItem("Camera Coordinate System", static_cast<int>(ControlStyle::CAMERA_COORDINATE_SYSTEM));

#ifndef SPNAV_AVAILABLE
        UI->labelMouse->setEnabled(false);
        UI->labelRotation->setEnabled(false);
        UI->sensitivityRot->setEnabled(false);
        UI->disableRotation->setEnabled(false);
        UI->labelTranslation->setEnabled(false);
        UI->sensitivityTrans->setEnabled(false);
        UI->disableTranslation->setEnabled(false);
#endif // SPNAV_AVAILABLE

        // In case of embedded use of this program it should not be possible to load an object after the editor is started
        if (embeddedGraspEditor)
        {
            UI->pushButtonLoadObject->setVisible(false);
        }
    }

    QString
    GraspEditorWindow::formatString(const char* s, float f)
    {
        QString str1(s);

        if (f >= 0)
        {
            str1 += " ";
        }

        if (fabs(f) < 1000)
        {
            str1 += " ";
        }

        if (fabs(f) < 100)
        {
            str1 += " ";
        }

        if (fabs(f) < 10)
        {
            str1 += " ";
        }

        QString str1n;
        str1n.setNum(f, 'f', 3);
        str1 = str1 + str1n;
        return str1;
    }

    void
    GraspEditorWindow::resetSceneryAll()
    {
    }

    void
    GraspEditorWindow::closeEvent(QCloseEvent* event)
    {
        quit();
        QMainWindow::closeEvent(event);
    }

    void
    GraspEditorWindow::buildVisu()
    {
        eefVisu->removeAllChildren();

        showCoordSystem();
        SceneObject::VisualizationType colModel =
            (UI->checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;

        if (robotEEF)
        {
            std::shared_ptr<VirtualRobot::CoinVisualization> visualizationAll = robotEEF->getVisualization(colModel);
            SoNode* visualisationNode = visualizationAll->getCoinVisualization();

            if (visualisationNode)
            {
                eefVisu->addChild(visualisationNode);
                //visualizationRobot->highlight(true);
            }

            for (RobotPtr hand : hands)
            {
                auto visHand = hand->getVisualization(colModel);
                SoNode* visHandNode = visHand->getCoinVisualization();
                visHand->setTransparency(0.8);

                if (visHandNode)
                {
                    eefVisu->addChild(visHandNode);
                }
            }
        }

        if (associatedGrasp && associatedRobotEEF)
        {
            std::shared_ptr<VirtualRobot::CoinVisualization> visualizationAll = associatedRobotEEF->getVisualization(colModel);
            SoNode* visualisationNode = visualizationAll->getCoinVisualization();
            visualizationAll->setTransparency(0.8);

            if (visualisationNode)
            {
                eefVisu->addChild(visualisationNode);
            }
        }

        objectSep->removeAllChildren();
        if (object)
        {
            SoNode* visualisationNode = nullptr;
            std::shared_ptr<VirtualRobot::CoinVisualization> visualizationObject =
                object->getVisualization<CoinVisualization>(colModel);
            if (visualizationObject)
            {
                visualisationNode = visualizationObject->getCoinVisualization();
            }

            if (visualisationNode)
            {
                objectSep->addChild(visualisationNode);
            }
        }

        buildGraspSetVisu();
    }

    int
    GraspEditorWindow::main()
    {
        // initialize QCoreApp
        QCoreApplication::setOrganizationName("H2T");
        QCoreApplication::setOrganizationDomain("h2t.anthropomatik.kit.edu");
        QCoreApplication::setAttribute(Qt::AA_UseDesktopOpenGL);
        QCoreApplication::setApplicationName("GraspEditor");
        QSettings settings;

        SoQt::show(this);
        SoQt::mainLoop();
        return 0;
    }

    void
    GraspEditorWindow::quit()
    {
        std::cout << "GraspEditorWindow: Closing" << std::endl;
        this->close();
        SoQt::exitMainLoop();
    }

    void
    GraspEditorWindow::selectRobot()
    {
        QString fi = QFileDialog::getOpenFileName(
            this, tr("Open Robot File"), QString(), tr("XML Files (*.xml)"));
        if (fi.isEmpty())
        {
            return;
        }
        robotFile = std::string(fi.toLatin1());

        settings.setValue("robot/path", QString::fromStdString(robotFile));
        loadRobot();
    }

    void
    GraspEditorWindow::selectObject(std::string file)
    {
        std::string s;

        // The object must be selected manually, cannot be done in the constructor
        if (embeddedGraspEditor)
        {
            s = file;
        }
        else
        {
            QString fi;
            QFileDialog dialog(this);
            dialog.setFileMode(QFileDialog::ExistingFile);
            dialog.setAcceptMode(QFileDialog::AcceptOpen);
            QStringList nameFilters;
            nameFilters << "Manipulation Object / Robot XML Files (*.xml *.moxml)"
                        //                        << "XML Files (*.xml)"
                        << "All Files (*.*)";
            dialog.setNameFilters(nameFilters);

            if (dialog.exec())
            {
                if (dialog.selectedFiles().size() == 0)
                {
                    return;
                }

                fi = dialog.selectedFiles()[0];
            }
            else
            {
                VR_INFO << "load dialog canceled" << std::endl;
                return;
            }
            s = std::string(fi.toLatin1());
        }

        if (s != "")
        {
            objectFile = s;
            settings.setValue("object/path", QString::fromStdString(objectFile));
            loadObject();
        }
    }

    void
    GraspEditorWindow::saveObject()
    {
        if (!object)
        {
            return;
        }

        // No need to select a file where the object is saved, it is the same as the input file
        if (!embeddedGraspEditor)
        {
            QString fi;
            QFileDialog dialog(this);
            dialog.setFileMode(QFileDialog::AnyFile);
            dialog.setAcceptMode(QFileDialog::AcceptSave);
            QStringList nameFilters;
            if (!robotObject)
            {
                dialog.setDefaultSuffix("moxml");
                nameFilters << "Manipulation Object XML Files (*.moxml)";
            }
            else
            {
                dialog.setDefaultSuffix("xml");
            }
            nameFilters << "XML Files (*.xml)" << "All Files (*.*)";

            dialog.setNameFilters(nameFilters);

            if (dialog.exec())
            {
                if (dialog.selectedFiles().size() == 0)
                {
                    return;
                }

                fi = dialog.selectedFiles()[0];
            }
            else
            {
                VR_INFO << "load dialog canceled" << std::endl;
                return;
            }
            objectFile = std::string(fi.toLatin1());
        }

        bool ok = false;

        try
        {
            if (robotObject)
            {
                ok = RobotIO::saveXML(robotObject,
                                      objectFile.filename(),
                                      objectFile.parent_path(),
                                      "",
                                      true,
                                      true,
                                      true,
                                      false);
            }
            else
            {
                ok = ObjectIO::saveManipulationObject(
                    std::dynamic_pointer_cast<ManipulationObject>(object), objectFile);
            }
        }
        catch (VirtualRobotException& e)
        {
            std::cout << " ERROR while saving object" << std::endl;
            std::cout << e.what();
            return;
        }

        if (!ok)
        {
            std::cout << " ERROR while saving object" << std::endl;
            return;
        }
        else
        {
            if (embeddedGraspEditor)
            {
                std::cout << "Changes successful saved to " << objectFile << std::endl;
                QMessageBox msgBox;
                msgBox.setText(
                    QString::fromStdString("Changes successful saved to " + objectFile.string()));
                msgBox.setIcon(QMessageBox::Information);
                msgBox.setStandardButtons(QMessageBox::Ok);
                msgBox.setDefaultButton(QMessageBox::Ok);
                msgBox.exec();
            }
        }
    }

    void
    GraspEditorWindow::loadRobot()
    {
        robotSep->removeAllChildren();
        std::cout << "Loading Robot from " << robotFile << std::endl;

        try
        {
            robot = RobotIO::loadRobot(robotFile);
        }
        catch (VirtualRobotException& e)
        {
            std::cout << " ERROR while creating robot" << std::endl;
            std::cout << e.what();
            return;
        }

        if (!robot)
        {
            std::cout << " ERROR while creating robot" << std::endl;
            return;
        }

        robot->getEndEffectors(eefs);
        robot->setPropagatingJointValuesEnabled(false);
        updateEEFBox();

        if (eefs.size() == 0)
        {
            selectEEF(-1);
        }
        else
        {
            selectEEF(0);
        }

        buildVisu();
        m_pExViewer->viewAll();
    }

    void
    GraspEditorWindow::selectEEF(int n)
    {
        hands.clear();
        currentEEF.reset();
        currentGraspSet.reset();
        currentGrasp.reset();

        eefVisu->removeAllChildren();

        /*if (robotEEF && currentGrasp) {
            currentGrasp->detachChain(robotEEF);
        }*/

        robotEEF.reset();

        if (n < 0 || n >= (int)eefs.size() || !robot)
        {
            return;
        }

        currentEEF = eefs[n];


        robotEEF = currentEEF->createEefRobot(currentEEF->getName(), currentEEF->getName());
        associatedRobotEEF =
            currentEEF->createEefRobot(currentEEF->getName(), currentEEF->getName());


        robotEEF_EEF = robotEEF->getEndEffector(currentEEF->getName());
        associatedRobotEEF_EEF = associatedRobotEEF->getEndEffector(currentEEF->getName());


        //bool colModel = UI.checkBoxColModel->isChecked();
        //eefVisu->addChild(CoinVisualizationFactory::getCoinVisualization(robotEEF,colModel));

        // select grasp set
        if (object)
        {
            currentGraspSet = object->getGraspSet(currentEEF);
        }

        updateGraspBox();
        selectGrasp(0);
        if (object && currentGrasp)
        {
            currentGrasp->attachChain(robotEEF, object, true);
        }
        if (object && associatedGrasp)
        {
            associatedGrasp->attachChain(associatedRobotEEF, object, true);
        }
        sampleGrasps();
    }

    void
    GraspEditorWindow::selectRobotObject(int n)
    {
        if (!robotObject)
            return;

        object = robotObject->getRobotNode(UI->comboBoxObject->itemText(n).toStdString());

        selectEEF(0);
    }

    int
    GraspEditorWindow::getAssociatedGrasp(int n)
    {
        if (!currentGraspSet || n < 0 || n >= (int)currentGraspSet->getSize() || !robot)
        {
            return -1;
        }

        std::string name =
            std::dynamic_pointer_cast<ChainedGrasp>(currentGraspSet->getGrasp(n))->getName();

        bool isPrepose = simox::alg::ends_with(name, PREPOSE_SUFFIX);
        if (isPrepose)
        {
            std::string nameWithoutSuffix =
                name.substr(0, name.size() - std::strlen(PREPOSE_SUFFIX));
            for (std::size_t i = 0; i < currentGraspSet->getSize(); ++i)
            {
                std::string currentName =
                    std::dynamic_pointer_cast<ChainedGrasp>(currentGraspSet->getGrasp(i))
                        ->getName();
                if (currentName == nameWithoutSuffix ||
                    currentName == nameWithoutSuffix + GRASP_OPTIONAL_SUFFIX)
                {
                    return i; // Found matching grasp
                }
            }
        }
        else
        {
            if (simox::alg::ends_with(name, GRASP_OPTIONAL_SUFFIX))
            {
                name = name.substr(0, name.size() - std::strlen(GRASP_OPTIONAL_SUFFIX));
            }

            std::string nameWithSuffix = name + PREPOSE_SUFFIX;
            for (std::size_t i = 0; i < currentGraspSet->getSize(); ++i)
            {
                std::string currentName =
                    std::dynamic_pointer_cast<ChainedGrasp>(currentGraspSet->getGrasp(i))
                        ->getName();
                if (currentName == nameWithSuffix)
                {
                    return i; // Found matching prepose
                }
            }
        }
        return -1; // Found not matching grasp
    }

    void
    GraspEditorWindow::selectGrasp(int n)
    {
        currentGrasp.reset();

        if (!currentGraspSet || n < 0 || n >= (int)currentGraspSet->getSize() || !robot)
        {
            buildVisu();
            m_pExViewer->scheduleRedraw();
            return;
        }

        int associatedGraspNumber = getAssociatedGrasp(n);
        if (associatedGraspNumber > -1)
        {
            //std::cout << Found associated grasp with index: " << associatedGraspNumber << std::endl;
            associatedGrasp = std::dynamic_pointer_cast<ChainedGrasp>(
                currentGraspSet->getGrasp(associatedGraspNumber));
        }
        else
        {
            associatedGrasp = nullptr;
        }


        currentGrasp = std::dynamic_pointer_cast<ChainedGrasp>(currentGraspSet->getGrasp(n));

        if (currentGrasp && robotEEF_EEF)
        {
            Eigen::Matrix4f gp;
            gp = currentGrasp->getTransformation();
            std::string preshape = currentGrasp->getPreshapeName();

            if (!preshape.empty() && robotEEF_EEF->hasPreshape(preshape))
            {
                robotEEF_EEF->setPreshape(preshape);
            }

            UI->labelQuality->setText(QString::number(currentGrasp->getQuality()));

            setCurrentGrasp(gp);

            // Ensure correct visualization at start time
            float x[6] = {0, 0, 0, 0, 0, 0};
            updateEEF(x);
        }

        if (associatedGrasp && robotEEF_EEF)
        {
            Eigen::Matrix4f gp;
            gp = associatedGrasp->getTransformation();

            setAssociatedGrasp(gp);

            // Ensure correct visualization at start time
            float x[6] = {0, 0, 0, 0, 0, 0};
            updateEEF(x);
        }

        buildVisu();
        m_pExViewer->scheduleRedraw();
    }

    void
    GraspEditorWindow::loadObject()
    {
        std::cout << "Loading Object from " << objectFile << std::endl;

        try
        {
            object = ObjectIO::loadManipulationObject(objectFile);
            robotObject = nullptr;
        }
        catch (VirtualRobotException& e)
        {
            // TODO: not pretty!
            try
            {
                robotObject = RobotIO::loadRobot(objectFile, RobotIO::eFullVisAsCol);
                object = nullptr;
            }
            catch (VirtualRobotException& e)
            {
                std::cout << " ERROR while creating object" << std::endl;
                std::cout << e.what();

                if (embeddedGraspEditor)
                {
                    QMessageBox msgBox;
                    msgBox.setText(QString::fromStdString(" ERROR while creating object."));
                    msgBox.setInformativeText("Please select a valid manipulation file.");
                    msgBox.setIcon(QMessageBox::Information);
                    msgBox.setStandardButtons(QMessageBox::Ok);
                    msgBox.setDefaultButton(QMessageBox::Ok);
                    msgBox.exec();
                }

                return;
            }
        }

        UI->comboBoxObject->clear();

        if (robotObject)
        {
            for (auto robotNode : robotObject->getRobotNodes())
            {
                if (robotNode->getVisualization())
                {
                    if (!object)
                        object = robotNode;
                    UI->comboBoxObject->addItem(QString::fromStdString(robotNode->getName()));
                }
            }
        }

        if (!object)
        {
            std::cout << " ERROR while creating object" << std::endl;
            return;
        }

        selectEEF(0);

        buildVisu();
    }

    void
    GraspEditorWindow::updateEEFBox()
    {
        UI->comboBoxEEF->clear();

        for (auto& eef : eefs)
        {
            UI->comboBoxEEF->addItem(QString(eef->getName().c_str()));
        }
    }

    void
    GraspEditorWindow::updateGraspBox()
    {
        UI->comboBoxGrasp->clear();

        if (!currentGraspSet || currentGraspSet->getSize() == 0)
        {
            return;
        }

        for (unsigned int i = 0; i < currentGraspSet->getSize(); i++)
        {
            UI->comboBoxGrasp->addItem(QString(currentGraspSet->getGrasp(i)->getName().c_str()));
        }
    }

    void
    GraspEditorWindow::closeEEF()
    {
        if (currentGrasp && robotEEF)
        {
            auto virtual_object = currentGrasp->getObjectNode(robotEEF);
            robotEEF_EEF->closeActors(virtual_object);
            for (auto hand : hands)
            {
                hand->getEndEffector(currentEEF->getName())->closeActors(virtual_object);
            }
        }

        m_pExViewer->scheduleRedraw();
    }

    void
    GraspEditorWindow::openEEF()
    {
        if (robotEEF_EEF)
        {
            robotEEF_EEF->openActors();
            for (auto hand : hands)
            {
                hand->getEndEffector(currentEEF->getName())->openActors();
            }
        }

        m_pExViewer->scheduleRedraw();
    }

    void
    GraspEditorWindow::renameGrasp()
    {
        if (!currentGrasp)
            return;

        bool ok;
        QString text = QInputDialog::getText(this,
                                             tr("Rename Grasp"),
                                             tr("New name:"),
                                             QLineEdit::Normal,
                                             tr(currentGrasp->getName().c_str()),
                                             &ok);


        if (ok && !text.isEmpty())
        {
            std::string sText = text.toStdString();
            currentGrasp->setName(sText);

            updateGraspBox();
        }
    }

    void
    GraspEditorWindow::addGrasp()
    {
        if (!object || !robot)
        {
            return;
        }

        if (!currentGraspSet)
        {
            currentGraspSet.reset(
                new GraspSet(currentEEF->getName(), robot->getType(), currentEEF->getName()));
            object->addGraspSet(currentGraspSet);
        }

        std::stringstream ss;
        ss << "Grasp " << (currentGraspSet->getSize());
        std::string name = ss.str();
        Eigen::Matrix4f pose;

        if (currentGrasp)
        {
            pose = currentGrasp->getTransformation();
            if (robotEEF)
                currentGrasp->detachChain(robotEEF);
        }
        else
        {
            pose = Eigen::Matrix4f::Identity();
        }

        ChainedGraspPtr g(new ChainedGrasp(
            name, robot->getType(), currentEEF->getName(), pose, std::string("GraspEditor")));

        currentGraspSet->addGrasp(g);
        updateGraspBox();
        UI->comboBoxGrasp->setCurrentIndex(UI->comboBoxGrasp->count() - 1);
        selectGrasp(UI->comboBoxGrasp->count() - 1);
        buildVisu();
    }

    Eigen::Matrix4f
    makeValidRigidTransformation(const Eigen::Matrix4f& matrix)
    {
        Eigen::Matrix4f corrected = matrix;

        Eigen::Matrix3f R = corrected.block<3, 3>(0, 0);
        Eigen::Vector3f t = corrected.block<3, 1>(0, 3);

        // Fix the rotation matrix (orthonormalize and fix determinant)
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3f corrected_R = svd.matrixU() * svd.matrixV().transpose();

        if (corrected_R.determinant() < 0)
        {
            corrected_R.col(2) *= -1.0f; // Flip last column if determinant is negative
        }

        corrected.block<3, 3>(0, 0) = corrected_R;
        corrected.block<3, 1>(0, 3) = t;

        corrected.row(3) = Eigen::Vector4f(0, 0, 0, 1);

        return corrected;
    }

    void
    GraspEditorWindow::updateEEF(float x[6])
    {
        if (currentGrasp && robotEEF)
        {
            auto virtual_object = currentGrasp->getObjectNode(robotEEF);
            if (virtual_object)
            {
                Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
                MathTools::posrpy2eigen4f(x, m);

                Eigen::Matrix4f localTransformation = virtual_object->getLocalTransformation();

                Eigen::Matrix4f newLocalTransformation;
                ControlStyle currentControlStyle =
                    static_cast<ControlStyle>(UI->controlStyle->currentIndex());
                switch (currentControlStyle)
                {
                    case ControlStyle::HAND_COORDINATE_SYSTEM:
                        newLocalTransformation = m * localTransformation;
                        break;
                    case ControlStyle::GLOBAL_COORDINATE_SYSTEM:
                        newLocalTransformation = localTransformation * m;
                        break;
                    case ControlStyle::CAMERA_COORDINATE_SYSTEM:
                        // Not yet implemented
                        newLocalTransformation = localTransformation;
                        break;
                }

                virtual_object->setLocalTransformation(newLocalTransformation);
                currentGrasp->setObjectTransformation(newLocalTransformation);
                virtual_object->updatePose(false);


                const Eigen::Matrix4f global_T_object =
                    makeValidRigidTransformation(virtual_object->getGlobalPose());
                const Eigen::Matrix4f global_T_hand = robotEEF->getGlobalPose();

                const Eigen::Matrix4f hand_T_object =
                    Eigen::Isometry3f{global_T_hand}.inverse() * global_T_object;

                const Eigen::Matrix4f global_T_hand_desired =
                    Eigen::Isometry3f{hand_T_object}.inverse().matrix();


                robotEEF->setGlobalPose(global_T_hand_desired);
            }
        }

        if (associatedGrasp && associatedRobotEEF)
        {
            auto virtual_object = associatedGrasp->getObjectNode(associatedRobotEEF);
            if (virtual_object)
            {
                const Eigen::Matrix4f global_T_object =
                    makeValidRigidTransformation(virtual_object->getGlobalPose());
                const Eigen::Matrix4f global_T_hand = associatedRobotEEF->getGlobalPose();

                const Eigen::Matrix4f hand_T_object =
                    Eigen::Isometry3f{global_T_hand}.inverse() * global_T_object;

                const Eigen::Matrix4f global_T_hand_desired =
                    Eigen::Isometry3f{hand_T_object}.inverse().matrix();

                associatedRobotEEF->setGlobalPose(global_T_hand_desired);
            }
        }

        m_pExViewer->scheduleRedraw();
    }

    void
    GraspEditorWindow::sliderReleased_ObjectX()
    {
        UI->horizontalSliderX->setValue(0);
    }

    void
    GraspEditorWindow::sliderReleased_ObjectY()
    {
        UI->horizontalSliderY->setValue(0);
    }

    void
    GraspEditorWindow::sliderReleased_ObjectZ()
    {
        UI->horizontalSliderZ->setValue(0);
    }

    void
    GraspEditorWindow::sliderReleased_ObjectA()
    {
        UI->horizontalSliderRo->setValue(0);
    }

    void
    GraspEditorWindow::sliderReleased_ObjectB()
    {
        UI->horizontalSliderPi->setValue(0);
    }

    void
    GraspEditorWindow::sliderReleased_ObjectG()
    {
        UI->horizontalSliderYa->setValue(0);
    }

    void
    GraspEditorWindow::setCurrentGrasp(Eigen::Matrix4f& p)
    {
        if (currentGrasp && robotEEF)
        {
            currentGrasp->attachChain(robotEEF, object, true);
            auto virtual_object = currentGrasp->getObjectNode(robotEEF);
            if (virtual_object)
            {
                virtual_object->setLocalTransformation(p);
                virtual_object->updatePose(false);
            }
        }

        sampleGrasps();
        m_pExViewer->scheduleRedraw();
    }

    void
    GraspEditorWindow::setAssociatedGrasp(Eigen::Matrix4f& p)
    {
        if (associatedGrasp && associatedRobotEEF)
        {
            associatedGrasp->attachChain(associatedRobotEEF, object, true);
            auto virtual_object = associatedGrasp->getObjectNode(associatedRobotEEF);
            if (virtual_object)
            {
                virtual_object->setLocalTransformation(p);
                virtual_object->updatePose(false);
            }
        }

        sampleGrasps();
        m_pExViewer->scheduleRedraw();
    }

    void
    GraspEditorWindow::showCoordSystem()
    {
        if (robotEEF)
        {
            RobotNodePtr tcp = robotEEF_EEF->getTcp();

            if (!tcp)
            {
                return;
            }

            tcp->showCoordinateSystem(UI->checkBoxTCP->isChecked());

            if (currentGrasp)
                currentGrasp->visualizeRotationCoordSystem(robotEEF, UI->checkBoxTCP->isChecked());
        }
    }

    void
    GraspEditorWindow::buildGraspSetVisu()
    {
        graspSetVisu->removeAllChildren();

        if (UI->checkBoxGraspSet->isChecked() && robotEEF && robotEEF_EEF && currentGraspSet &&
            object)
        {
            GraspSetPtr gs = currentGraspSet->clone();
            gs->removeGrasp(currentGrasp);
            SoSeparator* visu = CoinVisualizationFactory::CreateGraspSetVisualization(
                gs, robotEEF_EEF, object->getGlobalPose());

            if (visu)
            {
                graspSetVisu->addChild(visu);
            }
        }
    }

    void
    GraspEditorWindow::virtualJointValueChanged()
    {
        if (currentGrasp)
        {
            currentGrasp->x.setLimitsValue(UI->minX->value(), UI->maxX->value());
            currentGrasp->y.setLimitsValue(UI->minY->value(), UI->maxY->value());
            currentGrasp->z.setLimitsValue(UI->minZ->value(), UI->maxZ->value());
            currentGrasp->roll.setLimitsValue(UI->minRoll->value(), UI->maxRoll->value());
            currentGrasp->pitch.setLimitsValue(UI->minPitch->value(), UI->maxPitch->value());
            currentGrasp->yaw.setLimitsValue(UI->minYaw->value(), UI->maxYaw->value());
            currentGrasp->updateChain(robotEEF);
            sampleGrasps();
        }
    }

    void
    GraspEditorWindow::sampleGrasps()
    {
        if (currentGrasp && robot)
        {
            int grid = UI->grid->value();
            if (grid > 1)
                hands = currentGrasp->sampleHandsUniform(robotEEF, grid);
            else
                hands.clear();
            buildVisu();
        }
    }

    void
    GraspEditorWindow::updateRotationalSensitivity()
    {
        if (UI->disableRotation->isChecked())
        {
            rotation_scaling = 0.0;
            UI->sensitivityRot->setEnabled(false);
        }
        else
        {
            rotation_scaling = MIN_ROT_SCALING + (UI->sensitivityRot->value() / 100.0) *
                                                     (MAX_ROT_SCALING - MIN_ROT_SCALING);
            UI->sensitivityRot->setEnabled(true);
        }
        //cout << "Rotational scaling: " << rotation_scaling << std::endl;
    }

    void
    GraspEditorWindow::updateTranslationalSensitivity()
    {
        if (UI->disableTranslation->isChecked())
        {
            translation_scaling = 0.0;
            UI->sensitivityTrans->setEnabled(false);
        }
        else
        {
            translation_scaling = MIN_TRANS_SCALING + (UI->sensitivityTrans->value() / 100.0) *
                                                          (MAX_TRANS_SCALING - MIN_TRANS_SCALING);
            UI->sensitivityTrans->setEnabled(true);
        }
        //cout << "Translational scaling: " << translation_scaling << std::endl;
    }

} // namespace VirtualRobot
