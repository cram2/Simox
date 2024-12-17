#include "Document.h"

#include "elements/core/exceptions.h"

namespace mjcf
{


    Document::Document() : document(new tinyxml2::XMLDocument())
    {
        // create root element
        tinyxml2::XMLElement* xml = document->NewElement(MujocoRoot::tag.c_str());
        document->InsertEndChild(xml);
        root.reset(new MujocoRoot(this, xml));
    }

    Document::Document(const Document& other)
    {
        deepCopyFrom(other);
    }

    Document::Document(Document&& other) = default;

    Document&
    Document::operator=(const Document& other)
    {
        if (&other == this)
        {
            return *this;
        }

        deepCopyFrom(other);
        return *this;
    }

    Document& Document::operator=(Document&& other) = default;

    void
    Document::loadFile(const std::string& fileName)
    {
        tinyxml2::XMLError error = document->LoadFile(fileName.c_str());
        if (error != tinyxml2::XML_SUCCESS)
        {
            throw MjcfIOError(document->ErrorStr());
        }
    }

    void
    Document::saveFile(const std::string& fileName)
    {
        tinyxml2::XMLError error = document->SaveFile(fileName.c_str());
        if (error != tinyxml2::XML_SUCCESS)
        {
            throw MjcfIOError(document->ErrorStr());
        }
    }

    void
    Document::deepCopyFrom(const Document& source)
    {
        // Copy document.
        source.document->DeepCopy(this->document.get());

        // Update root element.
        root.reset(new MujocoRoot(this, document->FirstChildElement(MujocoRoot::tag.c_str())));
    }

    void
    Document::print(std::ostream& os) const
    {
        tinyxml2::XMLPrinter printer;
        document->Print(&printer);
        os << printer.CStr();
    }

    std::string
    Document::getModelName() const
    {
        if (root->isAttributeSet("model"))
        {
            return root->getAttribute("model");
        }
        else
        {
            return "";
        }
    }

    void
    Document::setModelName(const std::string& name)
    {
        root->setAttribute("model", name);
    }

    Include
    Document::addInclude(const std::string& relativePath)
    {
        Include include = root->addChild<Include>();
        include.file = relativePath;
        return include;
    }

    void
    Document::setNewElementClass(const std::string& className, bool excludeBody)
    {
        this->newElementClass = className;
        this->newElementClassExcludeBody = excludeBody;

        if (!className.empty() && !default_().hasChild<DefaultClass>("class", className))
        {
            default_().addClass(className);
        }
    }

    float
    Document::getFloatCompPrecision() const
    {
        return floatCompPrecision;
    }

    void
    Document::setFloatCompPrecision(float value)
    {
        floatCompPrecision = value;
    }

    float
    Document::getDummyMass() const
    {
        return dummyMass;
    }

    void
    Document::setDummyMass(float value)
    {
        dummyMass = value;
    }

    std::ostream&
    operator<<(std::ostream& os, const Document& rhs)
    {
        rhs.print(os);
        return os;
    }

} // namespace mjcf
