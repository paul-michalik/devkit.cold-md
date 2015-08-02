#include "stdafx.h"
#include "CppUnitTest.h"
#include <cold-md/bullet/world.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace tests {
    namespace cold_md_bullet_exploration {
        const auto c_margin = 0.001;

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

        btTriangleIndexVertexArray c_unit_square_data(
            c_unit_square_tri.size() / 3u, &c_unit_square_tri[0], 3 * sizeof(int),
            c_unit_square_ver.size() / 3u, &c_unit_square_ver[0], 3 * sizeof(btScalar));

        TEST_CLASS(test_suite)
        {
            BEGIN_TEST_CLASS_ATTRIBUTE()
                TEST_CLASS_ATTRIBUTE(L"Module", L"cold-md.bullet.exploration")
                END_TEST_CLASS_ATTRIBUTE();

            MSTEST_UTILS_TRACKED_MEM_CHECK();
        public:
            TEST_METHOD_INITIALIZE(set_up)
            {
                MSTEST_UTILS_TRACKED_MEM_CHECK_START();
            }

            TEST_METHOD_CLEANUP(tear_down)
            {
                MSTEST_UTILS_TRACKED_MEM_CHECK_FINISH();
            }

            TEST_METHOD(can_construct_broadphase)
            {
                {
                    auto t_col_broadphase =
                        std::make_unique<btDbvtBroadphase>();

                    Assert::IsTrue(!!t_col_broadphase);

                    Logger::WriteMessage(mstest_utils::wlog_message()
                        << "Destroying " << typeid(*t_col_broadphase).name()
                        << std::endl | mstest_utils::as_string);
                }
            }

            TEST_METHOD(can_construct_default_configuration)
            {
                {
                    auto t_col_configuration =
                        std::make_unique<btDefaultCollisionConfiguration>();

                    Assert::IsTrue(!!t_col_configuration);

                    Logger::WriteMessage(mstest_utils::wlog_message()
                        << "Destroying " << typeid(*t_col_configuration).name()
                        << std::endl | mstest_utils::as_string);
                }
            }

            TEST_METHOD(can_construct_and_initialize_bullet_collision_world)
            {
                auto t_col_broadphase =
                    std::make_unique<btDbvtBroadphase>();

                {
                    auto t_col_configuration =
                        std::make_unique<btDefaultCollisionConfiguration>();

                    {
                        auto t_col_dispatcher =
                            std::make_unique<btCollisionDispatcher>(t_col_configuration.get());

                        btGImpactCollisionAlgorithm::registerAlgorithm(t_col_dispatcher.get());

                        {
                            auto t_col_world =
                                std::make_unique<btCollisionWorld>(
                                t_col_dispatcher.get(),
                                t_col_broadphase.get(),
                                t_col_configuration.get());

                            Assert::IsTrue(!!t_col_world);

                            Logger::WriteMessage(mstest_utils::wlog_message()
                                << "Destroying collision world..."
                                << std::endl | mstest_utils::as_string);
                        }

                        Logger::WriteMessage(mstest_utils::wlog_message()
                            << "collision world destroyed!"
                            << std::endl | mstest_utils::as_string);
                    }
                }
            }

            class bullet_world_for_test : public cold::bullet::world {
            public:
                using base_t = cold::bullet::world;

                using base_t::get_world;
            };

            TEST_METHOD(bullet_world_detects_collisions_of_moving_squares_as_expected)
            {
                // create shapes:
                btGImpactMeshShape t_shapes[] = {
                    &c_unit_square_data,
                    &c_unit_square_data
                };

                // initialize shapes:
                for (auto &s : t_shapes) {
                    s.setMargin(0.01);
                    s.updateBound();
                }

                // create objects:
                btCollisionObject t_objects[2];
                t_objects[0].setCollisionShape(&t_shapes[0]);
                t_objects[0].setWorldTransform(btTransform(btQuaternion(0, 0, 0), btVector3(0, 0, 0)));

                // move the second shape such that initially there is no collision:
                t_objects[1].setCollisionShape(&t_shapes[1]);
                t_objects[1].setWorldTransform(btTransform(btQuaternion(0, 0, 0), btVector3(2.1, 2.1, 0)));

                { 
                    // create bullet world:
                    auto t_world = std::make_unique<bullet_world_for_test>();

                    // add bodies:
                    t_world->get_world()->addCollisionObject(&t_objects[0]);
                    t_world->get_world()->addCollisionObject(&t_objects[1]);

                    Assert::AreEqual(2, t_world->get_world()->getCollisionObjectArray().size());

                    std::tuple<btVector3, int, int> t_test_data[] = {
                        // position, expected num of contacts, actual num of contacts (inital = 0):
                        std::make_tuple(btVector3{2. + 100 * c_margin, 100 * c_margin, 0}, 0, 0),
                        std::make_tuple(btVector3{2. - 1 * c_margin, 2. - 1 * c_margin, 0}, 1, 0), 
                        std::make_tuple(btVector3{2. - 2 * c_margin, 2. - 2 * c_margin, 0}, 1, 0),
                        std::make_tuple(btVector3{2. - 4 * c_margin, 2. - 4 * c_margin, 0}, 1, 0),
                        std::make_tuple(btVector3{2. - 8 * c_margin, 2. - 8 * c_margin, 0}, 1, 0),
                        std::make_tuple(btVector3{2. - 16 * c_margin, 2. - 16 * c_margin, 0}, 1, 0),
                        std::make_tuple(btVector3{2. - 32 * c_margin, 2. - 32 * c_margin, 0}, 4, 0),
                        std::make_tuple(btVector3{2. - 100 * c_margin, 2. - 100 * c_margin, 0}, 4, 0),
                        std::make_tuple(btVector3{2. - 200 * c_margin, 2. - 200 * c_margin, 0}, 4, 0),
                        std::make_tuple(btVector3{2., 2., 0}, 1, 0),
                        std::make_tuple(btVector3{2. + 1 * c_margin, 2. + 1 * c_margin, 0}, 0, 0),
                    };

                    int t_pos_count = 0;
                    for (auto &t_test : t_test_data) {
                        t_objects[1].getWorldTransform().setOrigin(std::get<0>(t_test));

                        t_world->get_world()->performDiscreteCollisionDetection();

                        Logger::WriteMessage(mstest_utils::wlog_message()
                            << "position: " << t_pos_count 
                            << std::endl << "-------------" << std::endl 
                            | mstest_utils::as_string);

                        for (auto m = 0; m < t_world->get_world()->getDispatcher()->getNumManifolds(); ++m) {
                            auto t_manifold = t_world->get_world()->getDispatcher()->getManifoldByIndexInternal(m);
                            for (auto c = 0; c < t_manifold->getNumContacts(); ++c) {

                                Logger::WriteMessage(mstest_utils::wlog_message()
                                    << "manifold: " << m << std::endl
                                    << "contact: " << c << std::endl | mstest_utils::as_string);

                                ++std::get<2>(t_test);
                            }
                        }

                        ++t_pos_count;
                    }

                    t_pos_count = 0;
                    for (auto const &t_test : t_test_data) {
                        Logger::WriteMessage(mstest_utils::wlog_message()
                            << "position: " << t_pos_count++ << " -> exp/act "
                            << std::get<1>(t_test) << "/"
                            << std::get<2>(t_test) << std::endl | mstest_utils::as_string);
                        Assert::AreEqual(
                            std::get<1>(t_test),
                            std::get<2>(t_test));
                    }
                }
            }

            TEST_METHOD(bullet_world_detects_collisions_with_what_precision_wrt_defined_margin)
            {
                Assert::Fail(L"not implemented");
            }

            TEST_METHOD(bullet_world_reacts_on_broad_phase_callback)
            {
                Assert::Fail(L"not implemented");
            }

            TEST_METHOD(bullet_world_reacts_on_narrow_phase_callback)
            {
                Assert::Fail(L"not implemented");
            }

            TEST_METHOD(can_construct_and_initialize_cold_bullet_world)
            {
                {
                    auto t_world = std::make_unique<cold::bullet::world>();

                    Assert::IsTrue(!!t_world);
                }
            }
        };
    }
}