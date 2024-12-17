/**
* @package    VirtualRobot
* @author     Rainer Kartmann
* @copyright  2018 Rainer Kartmann
*/

#define BOOST_TEST_MODULE SimoxUtility / json / IOTest

#include <filesystem>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/json/error.h>
#include <SimoxUtility/json/io.h>


namespace fs = std::filesystem;

struct Fixture
{
    const std::string FILENAME = "JsonIOTest.json";

    simox::json::json testj;

    Fixture()
    {
        testj["s"] = "answer";
        testj["i"] = 42;

        if (fs::is_regular_file(FILENAME))
        {
            fs::remove(FILENAME);
        }
    }

    ~Fixture()
    {
        if (fs::is_regular_file(FILENAME))
        {
            fs::remove(FILENAME);
        }
    }
};


BOOST_FIXTURE_TEST_SUITE(JsonIOTest, Fixture)

BOOST_AUTO_TEST_CASE(test_read_json_existent)
{
    // Write the JSON file.
    {
        std::ofstream ofs(FILENAME);
        ofs << testj.dump();
        // Ensure file exists.
    }
    BOOST_CHECK(fs::exists(FILENAME));

    // Test reading.
    const simox::json::json j = simox::json::read(FILENAME);

    BOOST_CHECK_EQUAL(j, testj);
    BOOST_CHECK_EQUAL(j.at("s").get<std::string>(), testj.at("s").get<std::string>());
    BOOST_CHECK_EQUAL(j.at("i").get<int>(), testj.at("i").get<int>());
}

BOOST_AUTO_TEST_CASE(test_read_json_nonexistent)
{
    // Ensure file does not exist.
    BOOST_CHECK(!fs::exists(FILENAME));

    // Test reading.
    simox::json::json j;
    BOOST_CHECK_THROW(j = simox::json::read(FILENAME), simox::json::error::IOError);
}

BOOST_AUTO_TEST_CASE(test_write_json_valid)
{
    // Ensure file does not exist.
    BOOST_CHECK(!fs::exists(FILENAME));

    // Test writing.
    simox::json::write(FILENAME, testj);
    // Check that something has been written.
    BOOST_CHECK(fs::exists(FILENAME));
}

BOOST_AUTO_TEST_CASE(test_write_json_invalid)
{
    BOOST_CHECK(!fs::exists(FILENAME));

    // Make it a directory, i.e. not writable.
    fs::create_directory(FILENAME);
    BOOST_CHECK(fs::exists(FILENAME));

    // Test writing.
    BOOST_CHECK_THROW(simox::json::write(FILENAME, testj), simox::json::error::IOError);

    // Clean up.
    fs::remove(FILENAME);
    BOOST_CHECK(!fs::exists(FILENAME));
}

BOOST_AUTO_TEST_CASE(test_read_after_write_json)
{
    // Ensure file does not exist.
    BOOST_CHECK(not fs::exists(FILENAME));

    // Test writing.
    simox::json::write(FILENAME, testj);

    // Test reading.
    simox::json::json j;
    BOOST_CHECK_NO_THROW(j = simox::json::read(FILENAME));

    BOOST_CHECK_EQUAL(j, testj);
    BOOST_CHECK_EQUAL(j.at("s").get<std::string>(), testj.at("s").get<std::string>());
    BOOST_CHECK_EQUAL(j.at("i").get<int>(), testj.at("i").get<int>());
}


BOOST_AUTO_TEST_SUITE_END()
