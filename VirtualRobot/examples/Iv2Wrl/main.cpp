#include <filesystem>

#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/XML/ObjectIO.h>

#include <Inventor/SoDB.h>
#include <Inventor/SoInput.h>
#include <Inventor/SoInteraction.h>
#include <Inventor/SoOutput.h>
#include <Inventor/VRMLnodes/SoVRMLGroup.h>
#include <Inventor/actions/SoToVRML2Action.h>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/nodes/SoFile.h>
#include <Inventor/nodes/SoNode.h>
#include <Inventor/nodes/SoSeparator.h>


using namespace VirtualRobot;
using VirtualRobot::RuntimeEnvironment;

namespace fs = std::filesystem;

static SoGroup*
convertSoFileChildren(SoGroup* orig)
{
    if (!orig)
    {
        return new SoGroup;
    }

    SoGroup* storeResult;

    if (orig->getTypeId() == SoSeparator::getClassTypeId())
    {
        storeResult = new SoSeparator;
    }
    else
    {
        storeResult = new SoGroup;
    }

    storeResult->ref();

    if (orig->getTypeId().isDerivedFrom(SoGroup::getClassTypeId()))
    {
        // process group node
        for (int i = 0; i < orig->getNumChildren(); i++)
        {
            SoNode* n1 = orig->getChild(i);

            if (n1->getTypeId().isDerivedFrom(SoGroup::getClassTypeId()))
            {
                // convert group
                SoGroup* n2 = (SoGroup*)n1;
                SoGroup* gr1 = convertSoFileChildren(n2);
                storeResult->addChild(gr1);
            }
            else if (n1->getTypeId() == SoFile::getClassTypeId())
            {
                // really load file!!
                SoFile* fn = (SoFile*)n1;
                SoGroup* fileChildren;
                fileChildren = fn->copyChildren();
                storeResult->addChild(fileChildren);
            }
            else
            {
                // just copy child node
                storeResult->addChild(n1);
            }
        }
    }

    storeResult->unrefNoDelete();
    return storeResult;
}

bool
exportToVRML(SoNode* node, fs::path const& exportFilePath)
{
    SoOutput* so = new SoOutput();
    if (!so->openFile(exportFilePath.c_str()))
    {
        std::cerr << "Could not open file " << exportFilePath << " for writing." << std::endl;
        return false;
    }

    so->setHeaderString("#VRML V2.0 utf8");

#if 1
    SoGroup* n = new SoGroup;
    n->ref();
    n->addChild(node);
    SoGroup* newVisu = convertSoFileChildren(n);
    newVisu->ref();
#endif

    SoToVRML2Action tovrml2;

    tovrml2.writeTexCoords(true);
    tovrml2.expandTexture2Node(true);
    tovrml2.setVerbosity(true);
    std::cout << "Tex written: " << tovrml2.areTexCoordWritten() << "\n";
    std::cout << "Tex2 expanded: " << tovrml2.areTexture2NodeExpanded() << std::endl;

    tovrml2.apply(newVisu);
    SoVRMLGroup* newroot = tovrml2.getVRML2SceneGraph();
    newroot->ref();
    SoWriteAction wra(so);
    wra.apply(newroot);
    newroot->unref();

    so->closeFile();

    newVisu->unref();
    n->unref();

    return true;
}

/**
 * Loads a Simox robot and converts it to Mujoco's XML format (MJCF).
 * The converted file is stored in a directory mjcf/ placed in the directory
 * of the input file.
 */
int
main(int argc, char* argv[])
{
    SoDB::init();
    SoInteraction::init();

    RuntimeEnvironment::setCaption("Convert .iv to .wrl files");

    RuntimeEnvironment::considerKey("input", ".iv file containing the (textured) mesh");
    RuntimeEnvironment::considerKey("output", ".wrl file converted from the input .iv file");

    RuntimeEnvironment::processCommandLine(argc, argv);


    if (RuntimeEnvironment::hasHelpFlag() || !RuntimeEnvironment::hasValue("input") ||
        !RuntimeEnvironment::hasValue("output"))
    {
        RuntimeEnvironment::printOptions();
        return 0;
    }

    fs::path inputFilename;
    {
        std::string input = RuntimeEnvironment::getValue("input");

        if (RuntimeEnvironment::getDataFileAbsolute(input))
        {
            inputFilename = input;
        }
        else
        {
            std::cout << "Something is wrong with " << input;
        }
    }
    fs::path outputFilename = RuntimeEnvironment::getValue("output");

    SoInput fileInput;

    if (!fileInput.openFile(inputFilename.c_str()))
    {
        std::cerr << "Cannot open file " << inputFilename << std::endl;
        return -1;
    }

    SoNode* coinVisualization = SoDB::readAll(&fileInput);
    if (coinVisualization == nullptr)
    {
        std::cerr << "Cannot load .iv file: " << inputFilename << std::endl;
        return -2;
    }

    bool outputSuccess = exportToVRML(coinVisualization, outputFilename);
    if (!outputSuccess)
    {
        std::cerr << "Cannot output .wrl file: " << outputFilename << std::endl;
        return -3;
    }

    std::cout << "Converted " << inputFilename << " to\n" << outputFilename << std::endl;

    return 0;
}
