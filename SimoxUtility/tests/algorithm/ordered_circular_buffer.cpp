#include <SimoxUtility/algorithm/ordered_circular_buffer.h>

#define BOOST_TEST_MODULE SimoxUtility / algorithm / ordered_circular_buffer
#include <chrono>
#include <random>

#include <boost/test/included/unit_test.hpp>

using Clock = std::chrono::high_resolution_clock;
using Time = Clock::time_point;
using Duration = Time::duration;
using MicroSeconds = std::chrono::microseconds;

float
toMilliseconds(Duration duration)
{
    MicroSeconds microSeconds = std::chrono::duration_cast<MicroSeconds>(duration);
    return 0.001f * microSeconds.count();
}

BOOST_AUTO_TEST_CASE(test_emplace)
{
    using Map = simox::OrderedCircularBuffer<std::string, int>;
    Map buffer = Map::createWithMaxSize(3);
    BOOST_CHECK_EQUAL(buffer.getMaxSize(), 3);
    BOOST_CHECK_EQUAL(buffer.size(), 0);

    // Check emplace
    buffer.emplace("A", 10);
    BOOST_CHECK_EQUAL(buffer.size(), 1);
    BOOST_CHECK_EQUAL(buffer.at("A"), 10);

    buffer.emplace("B", 20);
    BOOST_CHECK_EQUAL(buffer.size(), 2);
    BOOST_CHECK_EQUAL(buffer.at("B"), 20);

    buffer.emplace("C", 30);
    BOOST_CHECK_EQUAL(buffer.size(), 3);
    BOOST_CHECK_EQUAL(buffer.at("C"), 30);

    {
        // Check order
        auto it = buffer.begin();
        BOOST_CHECK_EQUAL(it->first, "A");
        BOOST_CHECK_EQUAL(it->second, 10);

        ++it;
        BOOST_CHECK_EQUAL(it->first, "B");
        BOOST_CHECK_EQUAL(it->second, 20);

        ++it;
        BOOST_CHECK_EQUAL(it->first, "C");
        BOOST_CHECK_EQUAL(it->second, 30);
    }

    // Insert element beyond max size
    buffer.emplace("D", 40);
    // Still three elements, lowest element removed
    BOOST_CHECK_EQUAL(buffer.size(), 3);
    BOOST_CHECK_THROW(buffer.at("A"), std::out_of_range);
    BOOST_CHECK_EQUAL(buffer.at("B"), 20);
    BOOST_CHECK_EQUAL(buffer.at("C"), 30);
    BOOST_CHECK_EQUAL(buffer.at("D"), 40);

    {
        // Check order
        auto it = buffer.begin();
        BOOST_CHECK_EQUAL(it->first, "B");
        BOOST_CHECK_EQUAL(it->second, 20);

        ++it;
        BOOST_CHECK_EQUAL(it->first, "C");
        BOOST_CHECK_EQUAL(it->second, 30);

        ++it;
        BOOST_CHECK_EQUAL(it->first, "D");
        BOOST_CHECK_EQUAL(it->second, 40);
    }
}

template <typename ContainerT>
void
measureFindRuntime(ContainerT const& buffer,
                   int findElements,
                   std::string containerName,
                   int startI,
                   int maxElements)
{
    std::minstd_rand engine;
    // Allow for 10% of the keys to not be found in the container
    std::uniform_int_distribution<> distrib(startI, startI + maxElements + maxElements / 10);

    Time timeStart = Clock::now();

    int sum = 0;
    for (int i = 0; i < findElements; ++i)
    {
        int searchIndex = distrib(engine);
        std::string searchKey = "Map/" + std::to_string(1000000 + searchIndex);
        auto it = buffer.find(searchKey);
        if (it != buffer.end())
        {
            sum += it->second;
        }
    }

    Time timeEnd = Clock::now();
    float duration = toMilliseconds(timeEnd - timeStart);

    std::cout << containerName << " find duration: " << duration << "ms  (sum = " << sum << ")"
              << std::endl;
}

template <typename ContainerT>
void
measureForeachRuntime(ContainerT const& buffer, std::string containerName)
{
    Time timeStart = Clock::now();

    int sum = 0;
    for (auto& pair : buffer)
    {
        sum += pair.second;
    }

    Time timeEnd = Clock::now();
    float duration = toMilliseconds(timeEnd - timeStart);

    std::cout << containerName << " foreach duration: " << duration << "ms  (sum = " << sum << ")"
              << std::endl;
}

BOOST_AUTO_TEST_CASE(test_emplace_performance)
{
    int insertElements = 100000;
    int maxElements = insertElements / 2;
    int findElements = insertElements;
    int findStartI = insertElements - maxElements;
    {
        using Map = std::map<std::string, int>;
        Map buffer;

        {
            Time timeStart = Clock::now();

            for (int i = 0; i < insertElements; ++i)
            {
                std::string key = "Map/" + std::to_string(1000000 + i);
                buffer.emplace(key, i);

                if ((int)buffer.size() > maxElements)
                {
                    buffer.erase(buffer.begin());
                }
            }

            Time timeEnd = Clock::now();
            float duration = toMilliseconds(timeEnd - timeStart);

            std::cout << "std::map emplace duration: " << duration << "ms" << std::endl;
        }

        measureFindRuntime(buffer, findElements, "std::map", findStartI, maxElements);
        measureForeachRuntime(buffer, "std::map");
    }

    {
        using Map = simox::OrderedCircularBuffer<std::string, int>;
        Map buffer = Map::createWithMaxSize(maxElements);

        {
            Time timeStart = Clock::now();

            for (int i = 0; i < insertElements; ++i)
            {
                std::string key = "Map/" + std::to_string(1000000 + i);
                buffer.emplace(key, std::move(i));
            }

            Time timeEnd = Clock::now();
            float duration = toMilliseconds(timeEnd - timeStart);

            std::cout << "OrderedCircularBuffer emplace duration: " << duration << "ms"
                      << std::endl;
        }

        measureFindRuntime(buffer, findElements, "OrderedCircularBuffer", findStartI, maxElements);
        measureForeachRuntime(buffer, "OrderedCircularBuffer");
    }
}
