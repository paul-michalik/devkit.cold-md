#include "stdafx.h"
#include "CppUnitTest.h"
#include <cold-md.contract/cold-md.h>
#include <cold-md.bullet/world.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace tests {
    namespace cold_md_bullet_exploration {
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

            TEST_METHOD(bullet_reacts_on_broad_phase_callback)
            {
                Assert::Fail(L"not implemented");
            }

            TEST_METHOD(bullet_reacts_on_narrow_phase_callback)
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