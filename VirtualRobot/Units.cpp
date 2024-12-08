#include "Units.h"
#include <cmath>

namespace VirtualRobot
{

    Units::Units(const std::string& unitName) : unitString(unitName)
    {
    }

    bool
    Units::isRadian()
    {
        return ("radian" == unitString || "rad" == unitString);
    }

    bool
    Units::isDegree()
    {
        return ("degree" == unitString || "deg" == unitString);
    }

    bool
    Units::isAngle()
    {
        return (isRadian() || isDegree());
    }

    float
    Units::toRadian(float m)
    {
        if (isDegree())
        {
            return m * (float)M_PI / 180.0f;
        }
        else
        {
            return m;
        }
    }

    float
    Units::toDegree(float m)
    {
        if (isRadian())
        {
            return m * 180.0f / (float)M_PI;
        }
        else
        {
            return m;
        }
    }

    bool
    Units::isMillimeter()
    {
        return ("mm" == unitString || "millimeter" == unitString);
    }

    bool
    Units::isMeter()
    {
        return ("m" == unitString || "meter" == unitString);
    }

    bool
    Units::isLength()
    {
        return (isMillimeter() || isMeter());
    }

    float
    Units::toMillimeter(float m)
    {
        if (isMeter())
        {
            return m * 1000.0f;
        }
        else
        {
            return m;
        }
    }

    float
    Units::toMeter(float m)
    {
        if (isMillimeter())
        {
            return m * 0.001f;
        }
        else
        {
            return m;
        }
    }

    bool
    Units::isGram()
    {
        return ("g" == unitString || "gram" == unitString);
    }

    bool
    Units::isKilogram()
    {
        return ("kg" == unitString || "kilogram" == unitString);
    }

    bool
    Units::isTon()
    {
        return ("t" == unitString || "ton" == unitString);
    }

    bool
    Units::isWeight()
    {
        return (isGram() || isKilogram() || isTon());
    }

    float
    Units::toGram(float m)
    {
        if (isKilogram())
        {
            return m * 1000.0f;
        }
        else if (isTon())
        {
            return m * 1000000.0f;
        }
        else
        {
            return m;
        }
    }

    float
    Units::toKilogram(float m)
    {
        if (isGram())
        {
            return m * 0.001f;
        }
        else if (isTon())
        {
            return m * 1000.0f;
        }
        else
        {
            return m;
        }
    }

    float
    Units::toTon(float m)
    {
        if (isGram())
        {
            return m * 0.000001f;
        }
        else if (isKilogram())
        {
            return m * 0.001f;
        }
        else
        {
            return m;
        }
    }

    bool
    Units::isSecond()
    {
        return ("s" == unitString || "sec" == unitString || "second" == unitString);
    }

    bool
    Units::isMinute()
    {
        return ("min" == unitString || "minute" == unitString); //!< be careful m==meter!
    }

    bool
    Units::isHour()
    {
        return ("h" == unitString || "hour" == unitString);
    }

    bool
    Units::isTime()
    {
        return (isSecond() || isMinute() || isHour());
    }

    float
    Units::toSecond(float m)
    {
        if (isMinute())
        {
            return m * 60.0f;
        }
        else if (isHour())
        {
            return m * 3600.0f;
        }
        else
        {
            return m;
        }
    }

    float
    Units::toMinute(float m)
    {
        if (isSecond())
        {
            return m / 60.0f;
        }
        else if (isHour())
        {
            return m * 60.0f;
        }
        else
        {
            return m;
        }
    }

    float
    Units::toHour(float m)
    {
        if (isSecond())
        {
            return m / 3600.0f;
        }
        else if (isMinute())
        {
            return m / 60.0f;
        }
        else
        {
            return m;
        }
    }

    bool
    Units::isValid()
    {
        return (isLength() || isAngle() || isWeight() || isTime());
    }
} // namespace VirtualRobot
