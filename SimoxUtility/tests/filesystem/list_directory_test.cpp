/**
* @package    VirtualRobot
* @author     Rainer Kartmann
* @copyright  2019 Rainer Kartmann
*/

#define BOOST_TEST_MODULE SimoxUtility / filesystem / list_directory_test

#include <stdio.h>

#include <random>
#include <string>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/filesystem/list_directory.h>


namespace fs = simox::fs;

struct list_directory_test_Fixture
{
    const fs::path directory = "filesystem_list_directory_test.dir";
    std::vector<fs::path> filenames = {"file1", "file2.txt", "file3.txt.temp", "~file4", ".file5"};

    bool sort = true;

    list_directory_test_Fixture()
    {
        std::sort(filenames.begin(), filenames.end());

        BOOST_REQUIRE(!std::filesystem::exists(directory));
        std::filesystem::create_directories(directory);

        for (auto file : filenames)
        {
            std::ofstream ofs(directory / file);
            BOOST_REQUIRE(std::filesystem::exists(directory / file));
        }
    }

    ~list_directory_test_Fixture()
    {
        std::filesystem::remove_all(directory);
        BOOST_REQUIRE(!std::filesystem::exists(directory));
    }
};


BOOST_FIXTURE_TEST_SUITE(list_directory_test, list_directory_test_Fixture)

BOOST_AUTO_TEST_CASE(test_list_directory_local)
{
    bool local = true;
    std::vector<fs::path> entries = fs::list_directory(directory, local, sort);

    BOOST_CHECK_EQUAL_COLLECTIONS(
        entries.begin(), entries.end(), filenames.begin(), filenames.end());
}

BOOST_AUTO_TEST_CASE(test_list_directory_nonlocal)
{
    bool local = false;
    std::vector<fs::path> entries = fs::list_directory(directory, local, sort);

    std::vector<fs::path> expected;
    for (const auto& file : filenames)
    {
        expected.push_back(directory / file);
    }

    BOOST_CHECK_EQUAL_COLLECTIONS(entries.begin(), entries.end(), expected.begin(), expected.end());
}


BOOST_AUTO_TEST_SUITE_END()
