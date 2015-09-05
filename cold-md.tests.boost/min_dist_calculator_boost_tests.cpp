#include "stdafx.h"
#include <min_dist_calculator.h>

// Recommended cli options
// --log_level=test_suite --report_level=short --build_info=yes
namespace tests {

    bool initialize_global_fixture();

    // this isn´t working, crashes in bullet, some initialization issues coming on me...
    //const bool c_global_fixture_initialiter = initialize_global_fixture();

    namespace bullet_physics_min_dist {

        class min_dist_world : boost::noncopyable {
            btDbvtBroadphase* _col_broadphase = new btDbvtBroadphase;
            btDefaultCollisionConfiguration* _col_configuration = new btDefaultCollisionConfiguration;
            btCollisionDispatcher* _col_dispatcher = new btCollisionDispatcher{_col_configuration};

            btCollisionWorld* _world = new btCollisionWorld {
               _col_dispatcher,
               _col_broadphase,
               _col_configuration
            };
        public:
            min_dist_world()
            {
                btGImpactCollisionAlgorithm::registerAlgorithm(_col_dispatcher);
            }

            ~min_dist_world()
            {
                delete _world;
                delete _col_dispatcher;
                delete _col_configuration;
                delete _col_broadphase;
            }

            btCollisionWorld* get()
            {
                return _world;
            }
        };

        const btScalar c_margin = 0.0001;

        // We will probably never get results which are more precise than that since
        // bullet always add/subtracts the given "margin" to/from all results or inputs.
        const btScalar c_precision = std::nextafter(2. * c_margin, (3. * c_margin));

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
               c_unit_square_tri.size() / 3u, &c_unit_square_tri[0], 3 * sizeof(int),
               c_unit_square_ver.size() / 3u, &c_unit_square_ver[0], 3 * sizeof(btScalar)
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
        };

        BOOST_AUTO_TEST_SUITE(min_dist_calculator_basic_tests);

        // The position of the objects does not matter; the value of distance is always (close to) zero
        // The position matters, with regard to value of the attributes m_pointInWorld and m_normalOnBInWorld.
        // The algorithm chooses one existing manifold with the smallest distance between the penetration points.

        // define position p1_0:  maybe insert a drawing or a picture...?
        // a is translated by {-1, -1, -1}
        // b is translated by {+1, +1, +1}
        // => a, b are coincident in a corner at {0, 0, 0}
        BOOST_AUTO_TEST_CASE(two_penetrating_squares_in_position_p1_0_have_minimum_distance_zero_at_one_point)
        {
            auto const move_diag = 1.;

            ///@ Given a collision world with colliding two mesh objects a and b...
            min_dist_fixture fixture = {
               btTransform({0., 0., 0.}, {-move_diag, -move_diag, 0.}), // a
               btTransform({0., 0., 0.}, {+move_diag, +move_diag, 0.})  // b
            };

            // It is always assumed that performDiscreteCollisionDetection was performed...
            fixture.world.get()->performDiscreteCollisionDetection();

            //@ When I calculate the minimimum distances in this world...
            NSBulletPhysicsExt::min_dist_calculator calc(fixture.world.get());
            auto act_result_matrix = calc.calculate();

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
        BOOST_AUTO_TEST_CASE(two_intersecting_squares_in_position_p1_1_have_minimum_distance_zero_at_one_point)
        {
            ///@ Given a collision world with colliding two mesh objects a and b...
            min_dist_fixture fixture = {
               btTransform({0, 0, 0},{0, 0, 0}), // a
               btTransform({0, 0, SIMD_PI / 2.0},{0, 0, 0})  // b
                // CAUTION: the notion of yaw, pitch, roll depends on the definion of
                // BT_EULER_DEFAULT_ZYX , see http://bulletphysics.org/Bullet/BulletFull/classbtQuaternion.html#a8bd5d699377ba585749d325076616ffb
            };
            // It is always assumed that performDiscreteCollisionDetection was performed...
            fixture.world.get()->performDiscreteCollisionDetection();

            //@ When I calculate the minimimum distances in this world...
            NSBulletPhysicsExt::min_dist_calculator calc(fixture.world.get());
            auto act_result_matrix = calc.calculate();

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

            ///@ the point of closest distance on obj_a lies on the intersection segement
            BOOST_CHECK(
                std::abs(act.point_on_a.getY()) < c_precision &&
                std::abs(act.point_on_a.getZ()) < c_precision);
            BOOST_CHECK(
                -1. - c_precision < act.point_on_a.getX() &&
                act.point_on_a.getX() < 1. + c_precision);

            ///@ the point of closest distance on obj_b lies on the intersection segement
            BOOST_CHECK(
                std::abs(act.point_on_b.getY()) < c_precision &&
                std::abs(act.point_on_b.getZ()) < c_precision);
            BOOST_CHECK(
                -1. - c_precision < act.point_on_b.getX() &&
                act.point_on_b.getX() < 1. + c_precision);

            ///@ the points on either object are not further away from each other than the precision
            BOOST_CHECK(act.point_on_a.distance(act.point_on_b) < c_precision);
        }

        // define position p2...
        BOOST_AUTO_TEST_CASE(two_non_penetrating_squares_in_position_p2_have_minimum_distances_as_expected)
        {
            ///@ Given a collision world with non-colliding two mesh objects a, b in position p2...
            min_dist_fixture fixture = {
               btTransform({0., 0., 0.},{-1., 0., 0.}), // a
               btTransform({0., 0., 0.},{+1., 0., 0.})  // b
            };

            // this is assumed!
            fixture.world.get()->performDiscreteCollisionDetection();

            BOOST_FAIL("Not yet implemented!");
        }

        // define position p2...
        BOOST_AUTO_TEST_CASE(three_non_penetrating_squares_in_position_p3_have_minimum_distances_as_expected)
        {
            ///@ Given a collision world with non-colliding three mesh objects a, b, c in position p3...
            min_dist_fixture fixture = {
               btTransform({0., 0., 0.},{-1., 0., 0.}), // a
               btTransform({0., 0., 0.},{+1., 0., 0.}), // b
               btTransform({0., 0., 0.},{ 0.,+1., 0.})  // c
            };

            // this is assumed!
            fixture.world.get()->performDiscreteCollisionDetection();

            BOOST_FAIL("Not yet implemented!");
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