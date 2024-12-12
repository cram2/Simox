#pragma once

/**
* @package    VirtualRobot
* @author     Manfred Kroehnert <mkroehnert _at_ users dot sourceforge dot net>
* @copyright  2011 Manfred Kroehnert
*/

#include "VirtualRobotImportExport.h"

#include <string>

namespace VirtualRobot
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT Units
    {
    public:

        enum UnitsType
        {
            eAngle,
            eLength,
            eWeight,
            eTime,
            eIgnore
        };

        Units(const std::string& unitName);

        bool isRadian();
        bool isDegree();
        bool isAngle();
        float toRadian(float m);
        float toDegree(float m);

        bool isMillimeter();
        bool isMeter();
        bool isLength();
        float toMillimeter(float m);
        float toMeter(float m);

        bool isGram();
        bool isKilogram();
        bool isTon();
        bool isWeight();
        float toGram(float m);
        float toKilogram(float m);
        float toTon(float m);

        bool isSecond();
        bool isMinute();
        bool isHour();
        bool isTime();
        float toSecond(float m);
        float toMinute(float m);
        float toHour(float m);

        bool isValid();

    private:
        std::string unitString;
    };

} // namespace VirtualRobot
