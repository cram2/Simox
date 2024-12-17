#pragma once

#include <filesystem>
#include <memory>

#include "SimoxUtility/error/SimoxError.h"

namespace rapidxml
{
    template <class Ch>
    class xml_node;
}

namespace simox
{

    namespace error
    {
        class XMLFormatError : public SimoxError
        {
        public:
            XMLFormatError(const std::string& message = std::string()) : SimoxError(message)
            {
            }
        };
    } // namespace error

    namespace xml
    {

        namespace attribute
        {
            template <typename T,
                      typename std::enable_if<std::is_fundamental<T>::value>::type* = nullptr>
            struct XMLAttribute
            {
                XMLAttribute(const std::string& attributeName) : attributeName(attributeName)
                {
                }

                std::string attributeName;
            };
        } // namespace attribute

        class RapidXMLWrapperNode;
        typedef std::shared_ptr<RapidXMLWrapperNode> RapidXMLWrapperNodePtr;

        class RapidXMLWrapperRootNode;
        typedef std::shared_ptr<RapidXMLWrapperRootNode> RapidXMLWrapperRootNodePtr;

    } // namespace xml

} // namespace simox
