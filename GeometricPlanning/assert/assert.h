#pragma once

/**
 * Depending on the configuration (whether doctest should be used or not),
 * we will either use doctest natively or mimick it's behavior by existing 
 * functionality from the VirtualRobot library + extensions.
 *
 * You can use the following:
 *
 * - REQUIRE()
 * - REQUIRE_MESSAGE()
 * - CHECK()
 * - CHECK_MESSAGE()
 *
 */

#ifdef USE_DOCTEST
  #include <GeometricPlanning/assert/doctest/assert.h>
#else
  #include <GeometricPlanning/assert/virtual_robot/assert.h>
#endif // USE_DOCTEST
