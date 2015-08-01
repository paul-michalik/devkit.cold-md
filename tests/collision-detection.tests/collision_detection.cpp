#include "stdafx.h"
#include "CppUnitTest.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace cold {

    namespace contract {
        struct world {
            virtual world* add_object() = 0;
        protected:
            ~world() = default;
        };
    }

    std::shared_ptr<contract::world> create_world();
}

namespace tests {
    namespace collision_detection {
        TEST_CLASS(test_class)
        {
            BEGIN_TEST_CLASS_ATTRIBUTE()
                TEST_CLASS_ATTRIBUTE(L"Module", L"tests.cd.bullet_try_outs")
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

            TEST_METHOD(bullet_reacts_on_broad_phase_callback)
            {
                Assert::Fail(L"not implemented");
            }

            TEST_METHOD(bullet_reacts_on_narrow_phase_callback)
            {
                Assert::Fail(L"not implemented");
            }

            TEST_METHOD(can_construct_and_initialize_cold_world)
            {
                auto t_world = cold::create_world();

                Assert::IsTrue(!!t_world);

                Logger::WriteMessage(mstest_utils::wlog_message()
                    << "cold destroyed!"
                    << std::endl | mstest_utils::as_string);
            }
        };
    }
}

namespace cold {
    namespace bullet_impl {
        class world : public ::cold::contract::world { 
            std::unique_ptr<btCollisionWorld> m_col_world;
            std::unique_ptr<btDbvtBroadphase> m_col_broadphase;
            std::unique_ptr<btDefaultCollisionConfiguration> m_col_configuration;
            std::unique_ptr<btCollisionDispatcher> m_col_dispatcher;

            virtual ::cold::contract::world* add_object() override
            {
                return this;
            }
        public:
            world()
            {
                m_col_broadphase =
                    std::make_unique<btDbvtBroadphase>();

                {
                    m_col_configuration =
                        std::make_unique<btDefaultCollisionConfiguration>();

                    {
                        m_col_dispatcher =
                            std::make_unique<btCollisionDispatcher>(m_col_configuration.get());

                        btGImpactCollisionAlgorithm::registerAlgorithm(m_col_dispatcher.get());

                        {
                            m_col_world =
                                std::make_unique<btCollisionWorld>(
                                m_col_dispatcher.get(),
                                m_col_broadphase.get(),
                                m_col_configuration.get());
                        }
                    }
                }
            }
        };
    }

    std::shared_ptr<contract::world> create_world()
    {
        return std::make_unique<bullet_impl::world>();
    }
}