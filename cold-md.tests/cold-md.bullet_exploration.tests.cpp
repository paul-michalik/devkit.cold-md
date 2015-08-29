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

        std::tuple<btVector3, int, int> c_contact_test_data[] = {
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

        btTriangleIndexVertexArray c_unit_square_data(
            c_unit_square_tri.size() / 3u, &c_unit_square_tri[0], 3 * sizeof(int),
            c_unit_square_ver.size() / 3u, &c_unit_square_ver[0], 3 * sizeof(btScalar));

        class test_objects {
        public:
            std::unique_ptr<btGImpactMeshShape> shapes[2];
            std::unique_ptr<btCollisionObject> objects[2];

            test_objects(double p_margin)
            {
                shapes[0] = std::make_unique<btGImpactMeshShape>(&c_unit_square_data);
                shapes[1] = std::make_unique<btGImpactMeshShape>(&c_unit_square_data);

                // initialize shapes:
                for (auto &s : shapes) {
                    s->setMargin(p_margin);
                    s->updateBound();
                }


                objects[0] = std::make_unique<btCollisionObject>();
                objects[0]->setCollisionShape(shapes[0].get());
                objects[0]->setWorldTransform(
                    btTransform(btQuaternion(0, 0, 0), btVector3(0, 0, 0)));

                objects[1] = std::make_unique<btCollisionObject>();
                // move the second shape such that initially there is no collision:
                objects[1]->setCollisionShape(shapes[1].get());
                objects[1]->setWorldTransform(
                    btTransform(btQuaternion(0, 0, 0), btVector3(2.1, 2.1, 0)));
            }
        };

        class bullet_world_for_test : public cold::bullet::world {
        public:
            using base_t = cold::bullet::world;

            using base_t::get_world;

            test_objects& m_objs;

            bullet_world_for_test(test_objects& p_objs)
                : m_objs(p_objs)
            {
                this->get_world()->addCollisionObject(m_objs.objects[0].get());
                this->get_world()->addCollisionObject(m_objs.objects[1].get());
            }

            ~bullet_world_for_test()
            {
                // try this:
                this->get_world()->removeCollisionObject(m_objs.objects[1].get());
                this->get_world()->removeCollisionObject(m_objs.objects[0].get());
                (void)0;
            }
        };

        TEST_CLASS(test_suite)
        {
            BEGIN_TEST_CLASS_ATTRIBUTE()
                TEST_CLASS_ATTRIBUTE(L"Module", L"cold-md.bullet.exploration")
                END_TEST_CLASS_ATTRIBUTE();

            MSTEST_UTILS_TRACKED_MEM_CHECK();

            enum class mem_tracking {
                on,
                off
            };

            mem_tracking m_mem_tracking_mode;

            void set(mem_tracking p_tracking_mode)
            {
                m_mem_tracking_mode = p_tracking_mode;
            }
        public:
            test_suite(): m_mem_tracking_mode(mem_tracking::on)
            {
            }

            TEST_CLASS_INITIALIZE(set_up_class)
            {
                // run once in order to get rid of internal caches. Otherwise
                // we see memory leaks...
                test_objects t_objs(0.01);
                {
                    bullet_world_for_test t_world(t_objs);
                    t_world.get_world()->performDiscreteCollisionDetection();
                }
            }

            TEST_METHOD_INITIALIZE(set_up)
            {
                MSTEST_UTILS_TRACKED_MEM_CHECK_START();
            }

            TEST_METHOD_CLEANUP(tear_down)
            {
                if (m_mem_tracking_mode == mem_tracking::on) {
                    MSTEST_UTILS_TRACKED_MEM_CHECK_FINISH();
                }
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

            TEST_METHOD(can_construct_and_initialize_and_objects_to_bullet_collision_world)
            {
                test_objects t_objs(0.01);
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

                                t_col_world->addCollisionObject(t_objs.objects[0].get());
                                t_col_world->addCollisionObject(t_objs.objects[1].get());
                                t_col_world->performDiscreteCollisionDetection();
                            }

                            Logger::WriteMessage(mstest_utils::wlog_message()
                                << "collision world destroyed!"
                                << std::endl | mstest_utils::as_string);
                        }
                    }
                }
            }

            TEST_METHOD(can_encapsulates_anything_in_bullet_world_for_test)
            {
                test_objects t_objs(0.01);
                {
                    auto t_world = std::make_unique<bullet_world_for_test>(t_objs);
                    Assert::IsTrue(!!t_world);
                    t_world->get_world()->performDiscreteCollisionDetection();
                }
            }

            TEST_METHOD(bullet_world_detects_collisions_of_moving_squares_as_expected)
            {
                // temporarily switch of mem tracking:
                this->set(mem_tracking::off);

                test_objects t_objs(0.01);
                {
                    auto t_world = std::make_unique<bullet_world_for_test>(t_objs);
                    Assert::IsTrue(!!t_world);
                    Assert::AreEqual(2, t_world->get_world()->getCollisionObjectArray().size());


                    int t_pos_count = 0;
                    for (auto &t_test : c_contact_test_data) {
                        t_world->m_objs.objects[1]->getWorldTransform()
                            .setOrigin(std::get<0>(t_test));

                        // performDiscreteCollisionDetection leaks memory... :-(
                        t_world->get_world()->performDiscreteCollisionDetection();

                        Logger::WriteMessage(mstest_utils::wlog_message()
                            << "position: " << t_pos_count
                            << std::endl << "-------------" << endl);

                        for (auto m = 0; m < t_world
                            ->get_world()
                            ->getDispatcher()
                            ->getNumManifolds();
                        ++m) {
                            auto t_manifold = t_world->get_world()->getDispatcher()->getManifoldByIndexInternal(m);
                            for (auto c = 0; c < t_manifold->getNumContacts(); ++c) {

                                Logger::WriteMessage(mstest_utils::wlog_message()
                                    << "manifold: " << m << std::endl
                                    << "contact: " << c << endl);

                                ++std::get<2>(t_test);
                            }
                        }
                        ++t_pos_count;
                    }

                    t_pos_count = 0;
                    for (auto const &t_test : c_contact_test_data) {
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

            struct test_callback_base {
                using callback_cache_t = std::set<std::pair<void*, void*>>;
            };

            struct test_broadphase_callback : public btOverlapFilterCallback, public test_callback_base
            {
                mutable int m_num_of_calls;
                mutable callback_cache_t m_cache;

                // return true when pairs need collision
                virtual bool	needBroadphaseCollision(
                    btBroadphaseProxy* proxy0, 
                    btBroadphaseProxy* proxy1) const
                {
                    Logger::WriteMessage(mstest_utils::wlog_message() <<
                        __FUNCTION__ << std::endl <<
                        L"call nr.: " << m_num_of_calls << endl);

                    auto t_insert_res = this->m_cache.emplace(
                        std::min(proxy0->m_clientObject, proxy1->m_clientObject),
                        std::max(proxy0->m_clientObject, proxy1->m_clientObject));

                    if (t_insert_res.second) {
                        Logger::WriteMessage(mstest_utils::wlog_message() <<
                            __FUNCTION__ << std::endl <<
                            L"inserted: " <<
                            t_insert_res.first->first << ", " <<
                            t_insert_res.first->second << endl);
                    }

                    ++m_num_of_calls;

                    return true;
                }

                test_broadphase_callback()
                    : m_num_of_calls(0)
                {
                }
            };


            TEST_METHOD(bullet_world_reacts_on_broad_phase_callback)
            {
                test_objects t_objs(0.01);
                {
                    auto t_world = std::make_unique<bullet_world_for_test>(t_objs);
                    Assert::IsTrue(!!t_world);
                    Assert::AreEqual(2, t_world->get_world()->getCollisionObjectArray().size());

                    // install callback:
                    test_broadphase_callback t_callback;
                    t_world->get_world()
                        ->getPairCache()
                        ->setOverlapFilterCallback(&t_callback);

                    int t_pos_count = 0;
                    for (auto &t_test : c_contact_test_data) {
                        Logger::WriteMessage(mstest_utils::wlog_message() <<
                            "position: " << t_pos_count << endl);

                        t_world->m_objs.objects[1]->getWorldTransform()
                            .setOrigin(std::get<0>(t_test));
                        t_world->get_world()->performDiscreteCollisionDetection();

                        Assert::AreEqual(1u, t_callback.m_cache.size());
                        Assert::AreEqual(1, t_callback.m_num_of_calls);

                        // reset callback
                        test_broadphase_callback::callback_cache_t()
                            .swap(t_callback.m_cache);
                        t_callback.m_num_of_calls = 0;

                        t_pos_count++;
                    }
                }
            }

            class test_nearphase_callback : public test_callback_base {
                static test_nearphase_callback* s_this;
            public:
                callback_cache_t m_cache;
                int m_num_of_calls;

                void callback_impl(
                    btBroadphasePair& collisionPair,
                    btCollisionDispatcher& dispatcher,
                    const btDispatcherInfo& dispatchInfo)
                {
                    Logger::WriteMessage(mstest_utils::wlog_message() <<
                        __FUNCTION__ << std::endl <<
                        L"call nr.: " << m_num_of_calls << endl);

                    auto t_insert_res = this->m_cache.emplace(
                        std::min(
                        collisionPair.m_pProxy0->m_clientObject,
                        collisionPair.m_pProxy1->m_clientObject),
                        std::max(
                        collisionPair.m_pProxy0->m_clientObject,
                        collisionPair.m_pProxy1->m_clientObject));

                    if (t_insert_res.second) {
                        Logger::WriteMessage(mstest_utils::wlog_message() <<
                            __FUNCTION__ << std::endl <<
                            L"inserted: " <<
                            t_insert_res.first->first << ", " <<
                            t_insert_res.first->second << endl);
                    }

                    ++m_num_of_calls;
                }
            public:
                static void nearphase_callback(
                    btBroadphasePair& collisionPair,
                    btCollisionDispatcher& dispatcher,
                    const btDispatcherInfo& dispatchInfo)
                {
                    // Do your collision logic here
                    // Only dispatch the Bullet collision information if you want the physics to continue
                    s_this->callback_impl(collisionPair, dispatcher, dispatchInfo);

                    dispatcher.defaultNearCallback(collisionPair, dispatcher, dispatchInfo);
                }

                test_nearphase_callback()
                    : m_num_of_calls(0)
                {
                    s_this = this;
                }
            };

            TEST_METHOD(bullet_world_reacts_on_narrow_phase_callback)
            {
                test_objects t_objs(0.01);
                {
                    auto t_world = std::make_unique<bullet_world_for_test>(t_objs);
                    Assert::IsTrue(!!t_world);
                    Assert::AreEqual(2, t_world->get_world()->getCollisionObjectArray().size());

                    // install callback:
                    dynamic_cast<btCollisionDispatcher*>(
                        t_world->get_world()
                        ->getDispatcher())
                        ->setNearCallback(test_nearphase_callback::nearphase_callback);

                    test_nearphase_callback t_callback;

                    int t_pos_count = 0;
                    for (auto &t_test : c_contact_test_data) {
                        Logger::WriteMessage(mstest_utils::wlog_message() <<
                            "position: " << t_pos_count << std::endl
                            | mstest_utils::as_string);

                        t_world->m_objs.objects[1]->getWorldTransform()
                            .setOrigin(std::get<0>(t_test));
                        t_world->get_world()->performDiscreteCollisionDetection();

                        Assert::AreEqual(1u, t_callback.m_cache.size());
                        Assert::AreEqual(1, t_callback.m_num_of_calls);

                        // reset callback
                        test_broadphase_callback::callback_cache_t()
                            .swap(t_callback.m_cache);
                        t_callback.m_num_of_calls = 0;

                        t_pos_count++;
                    }
                }

            }

            TEST_METHOD(can_construct_and_initialize_cold_bullet_world)
            {
                {
                    auto t_world = std::make_unique<cold::bullet::world>();

                    Assert::IsTrue(!!t_world);
                }
            }
        };

        test_suite::test_nearphase_callback*
            test_suite::test_nearphase_callback::s_this = nullptr;
    }
}