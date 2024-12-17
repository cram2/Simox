#include "hsv.h"

Eigen::Vector3f
simox::color::rgb_to_hsv(const Eigen::Vector3f& rgb)
{
    // source: https://stackoverflow.com/a/6930407

    Eigen::Vector3f hsv;
    float min, max, delta;

    min = rgb(0) < rgb(1) ? rgb(0) : rgb(1);
    min = min < rgb(2) ? min : rgb(2);

    max = rgb(0) > rgb(1) ? rgb(0) : rgb(1);
    max = max > rgb(2) ? max : rgb(2);

    hsv(2) = max; // v
    delta = max - min;
    if (delta < 1e-5f)
    {
        hsv(1) = 0;
        hsv(0) = 0; // undefined, maybe nan?
        return hsv;
    }
    if (max > 0.0f)
    {
        // NOTE: if Max is == 0, this divide would cause a crash
        hsv(1) = (delta / max); // s
    }
    else
    {
        // if max is 0, then r = g = b = 0
        // s = 0, h is undefined
        hsv(1) = 0.0;
        hsv(0) = std::nanf(""); // its now undefined
        return hsv;
    }
    if (rgb(0) >= max) // > is bogus, just keeps compilor happy
    {
        hsv(0) = (rgb(1) - rgb(2)) / delta; // between yellow & magenta
    }
    else
    {
        if (rgb(1) >= max)
        {
            hsv(0) = 2.f + (rgb(2) - rgb(0)) / delta; // between cyan & yellow
        }
        else
        {
            hsv(0) = 4.f + (rgb(0) - rgb(1)) / delta; // between magenta & cyan
        }
    }

    hsv(0) *= 60.0f; // degrees

    if (hsv(0) < 0.f)
    {
        hsv(0) += 360.0f;
    }

    return hsv;
}

Eigen::Vector3f
simox::color::hsv_to_rgb(const Eigen::Vector3f& hsv)
{
    // source: https://stackoverflow.com/a/6930407

    float hh, p, q, t, ff;
    long i;
    Eigen::Vector3f rgb;

    if (hsv(1) <= 0.f) // < is bogus, just shuts up warnings
    {
        rgb(0) = hsv(2);
        rgb(1) = hsv(2);
        rgb(2) = hsv(2);
        return rgb;
    }
    hh = hsv(0);
    if (hh >= 360.f)
    {
        hh = 0.0;
    }
    hh /= 60.0f;
    i = static_cast<long>(hh);
    ff = hh - i;
    p = hsv(2) * (1.f - hsv(1));
    q = hsv(2) * (1.f - (hsv(1) * ff));
    t = hsv(2) * (1.f - (hsv(1) * (1.f - ff)));

    switch (i)
    {
        case 0:
            rgb(0) = hsv(2);
            rgb(1) = t;
            rgb(2) = p;
            break;
        case 1:
            rgb(0) = q;
            rgb(1) = hsv(2);
            rgb(2) = p;
            break;
        case 2:
            rgb(0) = p;
            rgb(1) = hsv(2);
            rgb(2) = t;
            break;

        case 3:
            rgb(0) = p;
            rgb(1) = q;
            rgb(2) = hsv(2);
            break;
        case 4:
            rgb(0) = t;
            rgb(1) = p;
            rgb(2) = hsv(2);
            break;
        case 5:
        default:
            rgb(0) = hsv(2);
            rgb(1) = p;
            rgb(2) = q;
            break;
    }
    return rgb;
}
