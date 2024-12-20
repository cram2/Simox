
#include "FileIO.h"

#include "../VirtualRobotException.h"
#include "Logging.h"

std::vector<Eigen::Vector3f>
VirtualRobot::FileIO::readPts(const std::string& filename, const char separator)
{
    std::ifstream file(filename.c_str());
    THROW_VR_EXCEPTION_IF(!file.good(), "Could not open file" << filename);
    char tmp;
    float a, b, c;
    std::vector<Eigen::Vector3f> res;
    Eigen::Vector3f v;
    bool needToReadSep = true;

    if (separator == ' ' || separator == '\n')
    {
        needToReadSep = false;
    }

    while (file.good())
    {
        file >> a;

        //THROW_VR_EXCEPTION_IF(!file.good(),"Error in file " << filename << " line " << res.size());
        if (needToReadSep)
        {
            file >> tmp;
            //THROW_VR_EXCEPTION_IF(!file.good(),"Error in file " << filename << " line " << res.size());
        }

        file >> b;

        //THROW_VR_EXCEPTION_IF(!file.good(),"Error in file " << filename << " line " << res.size());
        if (needToReadSep)
        {
            file >> tmp;
            //THROW_VR_EXCEPTION_IF(!file.good(),"Error in file " << filename << " line " << res.size());
        }

        file >> c;
        v << a, b, c;
        res.push_back(v);
    }

    return res;
}

bool
VirtualRobot::FileIO::readString(std::string& res, std::ifstream& file)
{
    int length = read<int32_t>(file);

    if (length <= 0)
    {
        VR_WARNING << "Bad string length: " << length << std::endl;
        return false;
    }

    char* data = new char[length + 1];
    file.read(data, length);
    data[length] = '\0';
    res = data;
    delete[] data;
    return true;
}

bool
VirtualRobot::FileIO::readMatrix4f(Eigen::Matrix4f& res, std::ifstream& file)
{
    float m[16];

    try
    {
        readArray<float>(m, 16, file);
    }
    catch (...)
    {
        VR_WARNING << "Error while reading matrix from file" << std::endl;
        return false;
    }

    res << m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11], m[12], m[13],
        m[14], m[15];
    return true;
}

void
VirtualRobot::FileIO::writeMatrix4f(std::ofstream& file, const Eigen::Matrix4f& m)
{
    float t[16];
    int k = 0;

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
        {
            t[k] = m(i, j);
            k++;
        }

    writeArray<float>(file, t, 16);
}

void
VirtualRobot::FileIO::writeString(std::ofstream& file, const std::string& value)
{
    size_t len = value.length();
    file.write((char*)&len, sizeof(int32_t));
    file.write(value.c_str(), len);
}
