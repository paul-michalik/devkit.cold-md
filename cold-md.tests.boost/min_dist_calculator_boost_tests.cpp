#include "stdafx.h"
#include <cold-md/bullet/min_dist_calculator.h>

// Recommended cli options
// --log_level=test_suite --report_level=short --build_info=yes
namespace tests {

    bool initialize_global_fixture();

    // this isn´t working, crashes in bullet, some initialization issues coming on me...
    const bool c_global_fixture_initialiter = initialize_global_fixture();

    namespace bullet_physics_min_dist {

        class min_dist_world : boost::noncopyable {
            btDbvtBroadphase _col_broadphase;
            btDefaultCollisionConfiguration _col_configuration;
            btCollisionDispatcher _col_dispatcher = &_col_configuration;

            btCollisionWorld _world = {
               &_col_dispatcher,
               &_col_broadphase,
               &_col_configuration
            };
        public:
            min_dist_world()
            {
                btGImpactCollisionAlgorithm::registerAlgorithm(&_col_dispatcher);
            }

            ~min_dist_world()
            {
            }

            btCollisionWorld* get()
            {
                return &_world;
            }
        };

        const btScalar c_margin = 0.0001;

        // We will probably never get results which are more precise than that since
        // bullet always add/subtracts the given "margin" to/from all results or inputs.
        const btScalar c_precision = std::nextafter(2.5 * c_margin, (10. * c_margin));

        std::array<btScalar, 4 * 3> c_unit_square_ver = {
           -1, -1, 0,
           +1, -1, 0,
           +1, +1, 0,
           -1, +1, 0
        };

        std::array<int, 2 * 3> c_unit_square_tri = {
           0, 1, 2,
           2, 3, 0
        };

        struct min_dist_fixture {

            btTriangleIndexVertexArray c_unit_square_data = {
               (int)c_unit_square_tri.size() / 3u, &c_unit_square_tri[0], 3 * sizeof(int),
               (int)c_unit_square_ver.size() / 3u, &c_unit_square_ver[0], 3 * sizeof(btScalar)
            };

            // cant use in place stack allocated instances... something's broken in auto
            // generated move constructors of btGImpactMeshShape...
            std::vector<std::unique_ptr<btGImpactMeshShape>> shapes;
            std::vector<std::unique_ptr<btCollisionObject>> objects;
            min_dist_world world;

            // creates as many instances of unitsquares as there are transforms in
            // the initializer list. Applys each transform eaxcatly once to each created instance.
            min_dist_fixture(std::initializer_list<btTransform> const& transforms_)
            {
                for (auto const& transf : transforms_) {
                    // create shape in place...
                    shapes.push_back(boost::make_unique<btGImpactMeshShape>(&c_unit_square_data));
                    shapes.back()->setMargin(c_margin);
                    shapes.back()->updateBound();

                    // create object:
                    objects.push_back(boost::make_unique<btCollisionObject>());
                    objects.back()->setCollisionShape(shapes.back().get());
                    // apply transform:
                    objects.back()->setWorldTransform(transf);

                    // insert into world:
                    world.get()->addCollisionObject(objects.back().get());
                }
            }

            ~min_dist_fixture()
            {
            }

            bool compare_triangles(btTriangleShape const &lhs_, btTriangleShape const &rhs_)
            {
                return false;
            }
        };

        BOOST_AUTO_TEST_SUITE(min_dist_calculator_basic_tests);

        // The position of the objects does not matter; the value of distance is always (close to) zero
        // The position matters, with regard to value of the attributes m_pointInWorld and m_normalOnBInWorld.
        // The algorithm chooses one existing manifold with the smallest distance between the penetration points.

        // define position p1_0:  maybe insert a drawing or a picture...?
        // a is translated by {-1, -1, -1}
        // b is translated by {+1, +1, +1}
        // => a, b are coincident in a corner at {0, 0, 0}
        BOOST_AUTO_TEST_CASE(two_squares_touching_in_one_point_have_distance_zero_at_origin)
        {
            auto const move_diag = 1.;

            ///@ Given a collision world with colliding two mesh objects a and b...
            min_dist_fixture fixture = {
               btTransform({0., 0., 0.}, {-move_diag, -move_diag, 0.}), // a
               btTransform({0., 0., 0.}, {+move_diag, +move_diag, 0.})  // b
            };

            //@ When I perform_distance_calculation the minimimum distances in this world...
            cold::bullet::min_dist_calculator<> calc(fixture.world.get());
            auto act_result_matrix = calc.perform_distance_calculation();

            ///@ Then the number of results in the output matrix is 1
            BOOST_REQUIRE_EQUAL(1u, act_result_matrix.size());

            ///@ And the entry "act" has following properties:
            auto const& act = act_result_matrix.front();

            BOOST_TEST_MESSAGE(act);

            ///@ - result was computed and object references are set to proper objects
            BOOST_REQUIRE(act.obj_a == fixture.objects[0].get());
            BOOST_REQUIRE(act.obj_b == fixture.objects[1].get());

            ///@ A collision occured
            BOOST_CHECK(cold::bullet::output_kind::collision == act.kind);

            ///@ - distance is close to 0.
            BOOST_CHECK(act.distance < c_precision);

            ///@ we've hit expected triangles...

            ///@ the point of closest distance on obj_a is close to origin
            BOOST_CHECK(act.point_on_a.distance({0., 0., 0.}) < c_precision);

            ///@ the point of closest distance on obj_b is close to origin
            BOOST_CHECK(act.point_on_b.distance({0., 0., 0.}) < c_precision);

            ///@ the points on either object are not further away from each other than the precision
            BOOST_CHECK(act.point_on_a.distance(act.point_on_b) < c_precision);
        }

        // define position p1_1:  maybe insert a drawing or a picture...?
        // a is left untouched
        // b is rotated around the roll axis by 90 degrees
        // => a, b intersect at line which coincides with the x axis, i.e. the min dist point
        //    has coordinates {a, 0, 0} where a in <-1 - c_precision, +1 + c_precision>
        BOOST_AUTO_TEST_CASE(two_intersecting_squares_have_minimum_distance_zero_at_all_point_along_the_intersection)
        {
            ///@ Given a collision world with colliding two mesh objects a and b...
            min_dist_fixture fixture = {
               btTransform({0, 0, 0},{0, 0, 0}), // a
               btTransform({0, SIMD_HALF_PI, 0},{0, 0, 0})  // b
                // CAUTION: the notion of yaw, pitch, roll depends on the definion of
                // BT_EULER_DEFAULT_ZYX , see http://bulletphysics.org/Bullet/BulletFull/classbtQuaternion.html#a8bd5d699377ba585749d325076616ffb
            };

            //@ When I perform_distance_calculation the minimimum distances in this world...
            cold::bullet::min_dist_calculator<> calc(fixture.world.get());
            auto act_result_matrix = calc.perform_distance_calculation();

            ///@ Then the number of results in the output matrix is 1
            BOOST_REQUIRE_EQUAL(1u, act_result_matrix.size());

            ///@ And the entry <act> has following properties:
            auto const& act = act_result_matrix.front();

            BOOST_TEST_MESSAGE(act);

            ///@ - result was computed and object references are set to proper objects
            BOOST_REQUIRE(act.obj_a == fixture.objects[0].get());
            BOOST_REQUIRE(act.obj_b == fixture.objects[1].get());

            ///@ - distance is close to 0.
            BOOST_CHECK(act.distance < c_precision);

            ///@ A collision occured
            BOOST_CHECK(cold::bullet::output_kind::collision == act.kind);

            // compare triangles instead of points, they are meaningless in this constellation...

            /////@ the point of closest distance on obj_a lies on the intersection segement
            //BOOST_CHECK(
            //    std::abs(act.point_on_a.getY()) < c_precision &&
            //    std::abs(act.point_on_a.getZ()) < c_precision);
            //BOOST_CHECK(
            //    -1. - c_precision < act.point_on_a.getX() &&
            //    act.point_on_a.getX() < 1. + c_precision);

            /////@ the point of closest distance on obj_b lies on the intersection segement
            //BOOST_CHECK(
            //    std::abs(act.point_on_b.getY()) < c_precision &&
            //    std::abs(act.point_on_b.getZ()) < c_precision);
            //BOOST_CHECK(
            //    -1. - c_precision < act.point_on_b.getX() &&
            //    act.point_on_b.getX() < 1. + c_precision);

            /////@ the points on either object are not further away from each other than the precision
            //BOOST_CHECK(act.point_on_a.distance(act.point_on_b) < c_precision);
        }

        // define position p2...
        BOOST_AUTO_TEST_CASE(two_parallel_squares_have_minimum_distance_as_expected)
        {
            ///@ Given a collision world with non-colliding two mesh objects a, b in position p2...
            min_dist_fixture fixture = {
               btTransform({0., 0., 0.},{0., 0., +0.5}), // a
               btTransform({0., 0., 0.},{0., 0., -0.5})  // b
            };

            //@ When I perform_distance_calculation the minimimum distances in this world...
           cold::bullet::min_dist_calculator<> calc(fixture.world.get());
            auto act_result_matrix = calc.perform_distance_calculation();

            ///@ Then the number of results in the output matrix is 1
            BOOST_REQUIRE_EQUAL(1u, act_result_matrix.size());

            ///@ And the entry <act> has following properties:
            auto const& act = act_result_matrix.front();

            BOOST_TEST_MESSAGE(act);

            ///@ - result was computed and object references are set to proper objects
            BOOST_REQUIRE(act.obj_a == fixture.objects[0].get());
            BOOST_REQUIRE(act.obj_b == fixture.objects[1].get());
            BOOST_CHECK(cold::bullet::output_kind::free == act.kind);

            ///@ - distance is close to 1.
            BOOST_CHECK((1. - c_precision) <= act.distance && act.distance <= (1. + c_precision));
            BOOST_CHECK(
                (1. - c_precision) <= act.point_on_a.distance(act.point_on_b) &&
                act.point_on_a.distance(act.point_on_b) <= (1. + c_precision));

            ///@ the z coordinates of the point are as expected
            BOOST_CHECK(
                (0.5 - c_precision) <= act.point_on_a.getZ() &&
                act.point_on_a.getZ() < (0.5 + c_precision));

            BOOST_CHECK(
                (-0.5 - c_precision) <= act.point_on_b.getZ() &&
                act.point_on_b.getZ() < (-0.5 + c_precision));
        }

        // define position p2...
        BOOST_AUTO_TEST_CASE(three_non_penetrating_squares_in_position_p3_have_minimum_distances_as_expected)
        {
            ///@ Given a collision world with non-colliding three mesh objects a, b, c in position p3...
            min_dist_fixture fixture = {
               btTransform({0., 0., 0.}, {0., 0., 0.}),   // a
               btTransform({0., 0., 0.}, {0., -2.5, 0.}), // b
               btTransform({SIMD_HALF_PI, 0., 0.}, { 2., 0., 0.}) // c
               // CAUTION: the notion of yaw, pitch, roll depends on the definion of
               // BT_EULER_DEFAULT_ZYX , see http://bulletphysics.org/Bullet/BulletFull/classbtQuaternion.html
            };

            //@ When I perform_distance_calculation the minimimum distances in this world...
            cold::bullet::min_dist_calculator<> calc(fixture.world.get());
            auto act_result_matrix = calc.perform_distance_calculation();

            ///@ Then the number of results in the output matrix is 3
            BOOST_REQUIRE_EQUAL(3u, act_result_matrix.size());

            {
                ///@ And the entry[0] = d(a, b) has following properties:
                auto const& act = act_result_matrix[0];

                BOOST_CHECK(cold::bullet::output_kind::free == act.kind);

                BOOST_TEST_MESSAGE(act);

                ///@ - result was computed and object references are set to proper objects
                BOOST_REQUIRE(act.obj_a == fixture.objects[0].get());
                BOOST_REQUIRE(act.obj_b == fixture.objects[1].get());

                ///@ - distance is close to 0.5
                BOOST_CHECK(
                    (0.5 - c_precision) <= act.distance &&
                    act.distance <= (0.5 + c_precision));
                BOOST_CHECK(
                    (0.5 - c_precision) <= act.point_on_a.distance(act.point_on_b) &&
                    act.point_on_a.distance(act.point_on_b) <= (0.5 + c_precision));

                ///@ the y coordinates of the point are as expected
                BOOST_CHECK(
                    (-1. - c_precision) <= act.point_on_a.getY() &&
                    act.point_on_a.getY() < (-1. + c_precision));
                BOOST_CHECK(
                    (-1.5 - c_precision) <= act.point_on_b.getY() &&
                    act.point_on_b.getY() < (-1.5 + c_precision));
            }

            {
                ///@ And the entry[1] = d(a, c) has following properties:
                auto const& act = act_result_matrix[1];

                BOOST_TEST_MESSAGE(act);

                BOOST_CHECK(cold::bullet::output_kind::free == act.kind);

                ///@ - result was computed and object references are set to proper objects
                BOOST_REQUIRE(act.obj_a == fixture.objects[0].get());
                BOOST_REQUIRE(act.obj_b == fixture.objects[2].get());

                ///@ - distance is close to 1
                BOOST_CHECK(
                    (1. - c_precision) <= act.distance &&
                    act.distance <= (1. + c_precision));
                BOOST_CHECK(
                    (1. - c_precision) <= act.point_on_a.distance(act.point_on_b) &&
                    act.point_on_a.distance(act.point_on_b) <= (1. + c_precision));

                ///@ the x coordinates of the point are as expected
                BOOST_CHECK(
                    (1. - c_precision) <= act.point_on_a.getX() &&
                    act.point_on_a.getX() < (1. + c_precision));
                BOOST_CHECK(
                    (2. - c_precision) <= act.point_on_b.getX() &&
                    act.point_on_b.getX() < (2. + c_precision));
            }

            {
                ///@ And the entry[2] =  = d(b, c) has following properties:
                auto const& act = act_result_matrix[2];

                BOOST_TEST_MESSAGE(act);

                ///@ - result was computed and object references are set to proper objects
                BOOST_REQUIRE(act.obj_a == fixture.objects[1].get());
                BOOST_REQUIRE(act.obj_b == fixture.objects[2].get());

                BOOST_CHECK(cold::bullet::output_kind::free == act.kind);

                ///@ - distance is close to (1/2) * (sqrt(5))
                auto const exp_dist = 0.5 * std::sqrt(5.);
                BOOST_CHECK(
                    (exp_dist - c_precision) <= act.distance &&
                    act.distance <= (exp_dist + c_precision));
                BOOST_CHECK(
                    (exp_dist - c_precision) <= act.point_on_a.distance(act.point_on_b) &&
                    act.point_on_a.distance(act.point_on_b) <= (exp_dist + c_precision));

                ///@ the x coordinates of the point are as expected
                BOOST_CHECK(
                    (1. - c_precision) <= act.point_on_a.getX() &&
                    act.point_on_a.getX() < (1. + c_precision));
                BOOST_CHECK(
                    (2. - c_precision) <= act.point_on_b.getX() &&
                    act.point_on_b.getX() < (2. + c_precision));

                ///@ the y coordinates of the point are as expected
                BOOST_CHECK(
                    (-1.5 - c_precision) <= act.point_on_a.getY() &&
                    act.point_on_a.getY() < (-1.5 + c_precision));
                BOOST_CHECK(
                    (-1. - c_precision) <= act.point_on_b.getY() &&
                    act.point_on_b.getY() < (-1. + c_precision));
            }
        }
        // etc...

        BOOST_AUTO_TEST_SUITE_END();
    }

    // We need to run the bullet collision detection once, 
    // since Bullet's internal global caches are causing
    // false positives when monitoring memory leaks.
    bool initialize_global_fixture()
    {
        bullet_physics_min_dist::min_dist_fixture fixture{
           btTransform({0., 0., 0.}, {0., 0., 0.})
        };

        fixture.world.get()->performDiscreteCollisionDetection();

        return true;
    }
}