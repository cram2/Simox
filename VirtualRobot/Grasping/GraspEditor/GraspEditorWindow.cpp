
#include "GraspEditorWindow.h"

#include <cmath>
#include <ctime>
#include <iostream>
#include <sstream>
#include <vector>

#include <QFileDialog>

#include <Eigen/Geometry>

#include <SimoxUtility/math/convert.h>

#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include "Logging.h"
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


using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

namespace VirtualRobot
{

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
        x[0] = (float)ikWindow->UI->horizontalSliderX->value();
        x[1] = (float)ikWindow->UI->horizontalSliderY->value();
        x[2] = (float)ikWindow->UI->horizontalSliderZ->value();
        x[3] = (float)ikWindow->UI->horizontalSliderRo->value();
        x[4] = (float)ikWindow->UI->horizontalSliderPi->value();
        x[5] = (float)ikWindow->UI->horizontalSliderYa->value();
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
        if (visualizationAll)
        {
            visualizationAll->highlight(false);
        }

        eefVisu->removeAllChildren();

        showCoordSystem();
        SceneObject::VisualizationType colModel =
            (UI->checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;

        if (robotEEF)
        {
            visualizationAll = robotEEF->getVisualization(colModel);
            SoNode* visualisationNode = visualizationAll->getCoinVisualization();

            if (visualisationNode)
            {
                eefVisu->addChild(visualisationNode);
                //visualizationRobot->highlight(true);
            }

            for (auto hand : hands)
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
            nameFilters << "XML Files (*.xml)"
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

        currentEEF->print();

        robotEEF = currentEEF->createEefRobot(currentEEF->getName(), currentEEF->getName());

        //robotEEF->print();
        robotEEF_EEF = robotEEF->getEndEffector(currentEEF->getName());
        robotEEF_EEF->print();

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


        //object->print();

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

    void
    GraspEditorWindow::updateEEF(float x[6])
    {
        if (currentGrasp && robotEEF)
        {
            auto virtual_object = currentGrasp->getObjectNode(robotEEF);
            //cout << "getGlobalPose robot:" << endl << robotEEF->getGlobalPose() << std::endl;
            //cout << "getGlobalPose TCP:" << endl <<  robotEEF_EEF->getTcp()->getGlobalPose() << std::endl;
            if (virtual_object)
            {
                Eigen::Matrix4f m;
                MathTools::posrpy2eigen4f(x, m);
                Eigen::Matrix4f transformation = virtual_object->getLocalTransformation() * m;
                virtual_object->setLocalTransformation(transformation);
                currentGrasp->setObjectTransformation(transformation);
                virtual_object->updatePose(false);
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

} // namespace VirtualRobot
