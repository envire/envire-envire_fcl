#include <boost/test/unit_test.hpp>
#include <envire_fcl/Dummy.hpp>

using namespace envire_fcl;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    envire_fcl::DummyClass dummy;
    dummy.welcome();
}
