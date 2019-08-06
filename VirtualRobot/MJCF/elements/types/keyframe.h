#pragma once

#include "../core/Attribute.h"

namespace mjcf
{

struct KeyframeSection : public Element<KeyframeSection>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(KeyframeSection)
};


}
