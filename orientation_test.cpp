#define BOOST_TEST_MODULE sqr_test
#include <boost/test/unit_test.hpp>

#define DEBUG
#include "robot.h"

BOOST_AUTO_TEST_CASE(Test1)
{
      BOOST_CHECK_EQUAL(0, orientationError(0, 0, 0, 0, 0));
      BOOST_CHECK_CLOSE(M_PI_2, orientationError(0, 0, 0, 0, 1), 1);
      BOOST_CHECK_CLOSE(-M_PI_2, orientationError(0, 0, 0, 0, -1), 1);
      BOOST_CHECK_CLOSE(M_PI_2, orientationError(0, 0, M_PI, 0, -1), 1);
      BOOST_CHECK_CLOSE(M_PI, orientationError(0, 0, M_PI, 1, 0), 1);
      BOOST_CHECK_CLOSE(M_PI_2, orientationError(0, 0, -M_PI, 0, -1), 1);
      BOOST_CHECK_CLOSE(M_PI, orientationError(0, 0, -M_PI+0.001, 1, 0), 1);
      BOOST_CHECK_CLOSE(-M_PI, orientationError(0, 0, -M_PI-0.001, 1, 0), 1);
      BOOST_CHECK_CLOSE(3*M_PI/4, orientationError(0, 0, 3*M_PI/4, 0, -1), 1);
      BOOST_CHECK_CLOSE(-M_PI, orientationError(0, 0, 0, -1, -0.01), 1);
      BOOST_CHECK_CLOSE(M_PI, orientationError(0, 0, 0, -1, 0.01), 1);
      BOOST_CHECK_CLOSE(-0.011, orientationError(0, 0, -M_PI+0.001, -1, 0.01), 1);

}
