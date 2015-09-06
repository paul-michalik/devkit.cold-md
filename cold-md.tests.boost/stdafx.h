#pragma once

#define BOOST_TEST_MODULE bullet_physics_min_dist_tests
#include <boost/test/unit_test.hpp>

// TODO: reference additional headers your program requires here
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>

#include <boost/noncopyable.hpp>
#include <boost/scope_exit.hpp>
#include <boost/make_unique.hpp>

#include <limits>
#include <iostream>
#include <strstream>
#include <string>
#include <vector>
#include <array>
#include <map>
#include <hash_map>
#include <type_traits>
#include <algorithm>
#include <functional>
#include <memory>
#include <iterator>
#include <queue>
#include <stack>
#include <cstddef>
#include <cfloat>
#define _USE_MATH_DEFINES
#include <cmath>
