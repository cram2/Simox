/**
* @package    SimoxUtility
* @author     Andre Meixner
* @copyright  2020 Andre Meixner
*/

#define BOOST_TEST_MODULE SimoxUtility / algorithm / string_tools

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/algorithm/string.h>

BOOST_AUTO_TEST_CASE(to_lower)
{
    BOOST_CHECK_EQUAL(simox::alg::to_lower("TO LoWER tesT!"), "to lower test!");
}

BOOST_AUTO_TEST_CASE(split)
{
    std::string testString = "   test       20 50   30 ";
    std::string testString2 = "   test   ;   20,50 ; 30 ";
    std::vector<std::string> expectedResult = {"test", "20", "50", "30"};
    BOOST_CHECK(simox::alg::split(testString) == expectedResult);
    BOOST_CHECK(simox::alg::split(testString2, ";,") == expectedResult);
    BOOST_CHECK_THROW(simox::alg::split_check_size(testString, 3), simox::error::SimoxError);
}

BOOST_AUTO_TEST_CASE(to_eigen)
{
    std::string testString = " 5.0   3   -1.0   ";
    Eigen::Vector3f expectedResult(5.0, 3.0, -1.0);
    BOOST_CHECK(simox::alg::to_eigen_vec(testString) == expectedResult);
    BOOST_CHECK((simox::alg::to_eigen_vec<float, 3>(testString)) == expectedResult);
    BOOST_CHECK_THROW((simox::alg::to_eigen_vec<float, 4>(testString)), simox::error::SimoxError);
}

BOOST_AUTO_TEST_CASE(to_string)
{
    bool testBool = false;
    float testFloat = -3.156;
    double testDouble = -3.34919295196;
    Eigen::Vector3f testVector3f(5.0, 3.5, -1.0);
    Eigen::VectorXf testVectorXf(9);
    testVectorXf << 5.0, 3.5, -1.0, 15, 20, 10, 30, 100, 1;
    std::vector<float> testVector = {5.0, 3.5, -1.0};
    std::vector<int> testVectorI = {1, -2, 3};
    std::vector<std::string> testVectorStr = {"Hello", "World!"};
    BOOST_CHECK_EQUAL(simox::alg::to_string(testBool), "false");
    BOOST_CHECK_EQUAL(simox::alg::to_string(testFloat), "-3.156");
    BOOST_CHECK_EQUAL(simox::alg::to_string(testDouble), "-3.34919295196");
    BOOST_CHECK_EQUAL(simox::alg::to_string(testVector3f), "5 3.5 -1");
    BOOST_CHECK_EQUAL(simox::alg::to_string(testVectorXf), "5 3.5 -1 15 20 10 30 100 1");
    BOOST_CHECK_EQUAL(simox::alg::to_string(testVector), "5 3.5 -1");
    BOOST_CHECK_EQUAL(simox::alg::to_string(testVectorI, ";"), "1;-2;3");
    BOOST_CHECK_EQUAL(simox::alg::to_string(testVectorStr), "Hello World!");
}

BOOST_AUTO_TEST_CASE(from_string)
{
    std::string testBool = " 0";
    std::string testBool2 = "FALSE";
    std::string testFloat = " -3.156";
    std::string testVector = " 5 3.5 -1.0 ";
    std::string testVector2 = " 5;3.5;-1.0 ";
    std::string testVector3 = " 5 ,3.5 ,-1 ,15 ,20, 10,30,100,1 ";
    bool expectedBool = false;
    float expectedFloat = -3.156f;
    Eigen::Vector3f expectedVector3f(5.0, 3.5, -1.0);
    Eigen::VectorXd expectedVectorXd(9);
    expectedVectorXd << 5.0, 3.5, -1.0, 15, 20, 10, 30, 100, 1;
    Eigen::VectorXd expectedVector9d(9);
    expectedVector9d << 5.0, 3.5, -1.0, 15, 20, 10, 30, 100, 1;
    std::vector<float> expectedVector = {5.0, 3.5, -1.0};
    BOOST_CHECK_EQUAL(simox::alg::to_<bool>(testBool), expectedBool);
    BOOST_CHECK_EQUAL(simox::alg::to_<bool>(testBool2), expectedBool);
    BOOST_CHECK_EQUAL(simox::alg::to_<float>(testFloat), expectedFloat);
    BOOST_CHECK(simox::alg::to_vec<float>(testVector) == expectedVector);
    BOOST_CHECK(simox::alg::to_vec_check_size<float>(testVector2, 3, ";") == expectedVector);
    BOOST_CHECK((simox::alg::to_eigen_vec<float, 3>(testVector)) == expectedVector3f);
    BOOST_CHECK(simox::alg::to_eigen_vec(testVector2, ";") == expectedVector3f);
    BOOST_CHECK(simox::alg::to_eigen_vec<double>(testVector3, ",") == expectedVectorXd);
    BOOST_CHECK((simox::alg::to_eigen_vec_check_rows<double>(testVector3, 9, ",")) ==
                expectedVectorXd);
    BOOST_CHECK_THROW((simox::alg::to_eigen_vec_check_rows<double>(testVector3, 10, ",")),
                      simox::error::SimoxError);
    BOOST_CHECK((simox::alg::to_eigen_vec<double, 9>(testVector3, ",")) == expectedVector9d);
}

BOOST_AUTO_TEST_CASE(capitalize_words)
{
    BOOST_CHECK_EQUAL(simox::alg::capitalize_words(""), "");
    BOOST_CHECK_EQUAL(simox::alg::capitalize_words("   "), "   ");
    BOOST_CHECK_EQUAL(simox::alg::capitalize_words(" 1 2 3 4 "), " 1 2 3 4 ");

    BOOST_CHECK_EQUAL(simox::alg::capitalize_words("test"), "Test");
    BOOST_CHECK_EQUAL(simox::alg::capitalize_words("  hello world! "), "  Hello World! ");
    BOOST_CHECK_EQUAL(simox::alg::capitalize_words("cap\neach\n - line"), "Cap\nEach\n - Line");

    const std::string in =
        "\nthis is a Partly CaPiTaLiZed sTrInG _with 0some #strange +occurences.right?yes";
    const std::string ex =
        "\nThis Is A Partly CaPiTaLiZed STrInG _with 0some #strange +occurences.right?yes";
    const std::string out = simox::alg::capitalize_words(in);
    BOOST_CHECK_EQUAL(out, ex);
    BOOST_CHECK_EQUAL_COLLECTIONS(out.begin(), out.end(), ex.begin(), ex.end());
}

BOOST_AUTO_TEST_CASE(time_conversion)
{
    std::string timeString = "2021-01-12";
    std::string timeFormat = "%Y-%m-%d";
    time_t time = simox::alg::to_time_t(timeString, timeFormat);
    BOOST_CHECK_EQUAL(simox::alg::to_string(time, timeFormat), timeString);
}

BOOST_AUTO_TEST_CASE(remove_prefix_suffix)
{
    // Do something:

    BOOST_CHECK_EQUAL(simox::alg::remove_prefix("my_word", "my_"), "word");
    BOOST_CHECK_EQUAL(simox::alg::remove_suffix("word_is", "_is"), "word");

    BOOST_CHECK_EQUAL(simox::alg::remove_prefix("one-one-one", "one-"), "one-one");
    BOOST_CHECK_EQUAL(simox::alg::remove_suffix("one-one-one", "-one"), "one-one");

    BOOST_CHECK_EQUAL(simox::alg::remove_prefix("one two three", "four"), "one two three");
    BOOST_CHECK_EQUAL(simox::alg::remove_suffix("one two three", "four"), "one two three");

    BOOST_CHECK_EQUAL(simox::alg::remove_prefix("one", "one"), "");
    BOOST_CHECK_EQUAL(simox::alg::remove_suffix("one", "one"), "");


    // Change nothing:

    BOOST_CHECK_EQUAL(simox::alg::remove_prefix("inc_omplete-prefix", "incomplete"),
                      "inc_omplete-prefix");
    BOOST_CHECK_EQUAL(simox::alg::remove_suffix("suffix-inc_omplete", "incomplete"),
                      "suffix-inc_omplete");

    BOOST_CHECK_EQUAL(simox::alg::remove_prefix("one two three", "four"), "one two three");
    BOOST_CHECK_EQUAL(simox::alg::remove_suffix("one two three", "four"), "one two three");

    BOOST_CHECK_EQUAL(simox::alg::remove_prefix("", ""), "");
    BOOST_CHECK_EQUAL(simox::alg::remove_suffix("", ""), "");
}
