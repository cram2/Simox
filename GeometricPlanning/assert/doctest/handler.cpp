#ifndef DOCTEST_CONFIG_DISABLE
#include <doctest/doctest.h>

DOCTEST_MAKE_STD_HEADERS_CLEAN_FROM_WARNINGS_ON_WALL_BEGIN
#include <algorithm>
#include <cstring>
#include <iostream>
#include <sstream>
DOCTEST_MAKE_STD_HEADERS_CLEAN_FROM_WARNINGS_ON_WALL_END

#include <VirtualRobot/VirtualRobotException.h>

// std::mutex g_mut;

namespace simox::control::assert::doctest
{
    static void
    handler(const ::doctest::AssertData& ad)
    {
        // uncomment if asserts will be used in a multi-threaded context
        // std::lock_guard<std::mutex> lock(g_mut);

        // here we can choose what to do:
        // - log the failed assert
        // - throw an exception
        // - call std::abort() or std::terminate()

        std::stringstream stream;

        stream << ::doctest::Color::LightGrey << ::doctest::skipPathFromFilename(ad.m_file) << "("
               << ad.m_line << "): ";
        stream << ::doctest::Color::Red << ::doctest::failureString(ad.m_at) << ": ";

        // handling only normal (comparison and unary) asserts - exceptions-related asserts have been skipped
        if ((ad.m_at & ::doctest::assertType::is_normal) != 0)
        {
            stream << ::doctest::Color::Cyan << ::doctest::assertString(ad.m_at) << "( "
                   << ad.m_expr << " ) ";
            stream << ::doctest::Color::None
                   << (ad.m_threw ? "THREW exception: " : "is NOT correct!\n");
            if (ad.m_threw) // This should not happen. Doctest should be forced to not throw anything. We are doing it on our own.
            {
                stream << ad.m_exception;
            }
            else
            {
                stream << "  values: " << ::doctest::assertString(ad.m_at) << "( " << ad.m_decomp
                       << " )";
            }
        }
        else
        {
            stream << ::doctest::Color::None << "an assert dealing with exceptions has failed!";
        }

        throw VirtualRobot::VirtualRobotException(stream.str());

        std::cout << std::endl;
    }

    class HandlerRegistration
    {
    public:
        HandlerRegistration() : context_(0, nullptr)
        {
            // sets the context as the default one - so asserts used outside of a testing context do not crash
            context_.setAsDefaultForAssertsOutOfTestCases();

            // set a handler with a signature: void(const ::doctest::AssertData&)
            // without setting a handler we would get std::abort() called when an assert fails
            context_.setAssertHandler(handler);
        }

    private:
        // construct a context
        ::doctest::Context context_;
    };

    // a global symbol to make the context and the handler persistent
    const static auto handlerReg = HandlerRegistration();


} // namespace simox::control::assert::doctest


#endif
