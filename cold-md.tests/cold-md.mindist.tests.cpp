#include "stdafx.h"
#include "CppUnitTest.h"
#include <cold-md/bullet/world.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace tests {
    namespace cold_md_mindist {
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
                    btTransform(btQuaternion(0, 0, 0), btVector3(2., 2., 0)));
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

        TEST_CLASS(test_suite) {
            BEGIN_TEST_CLASS_ATTRIBUTE()
                TEST_CLASS_ATTRIBUTE(L"Module", L"cold-md.mindist")
                END_TEST_CLASS_ATTRIBUTE();

            MSTEST_UTILS_TRACKED_MEM_CHECK();
        public:

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
                MSTEST_UTILS_TRACKED_MEM_CHECK_FINISH();
            }

            TEST_METHOD(local_data_of_gimpact_mesh_shape)
            {
                auto s = std::make_unique<btGImpactMeshShape>(&c_unit_square_data);
                s->setMargin(0.001);
                s->updateBound();

                Assert::AreEqual(1, s->getMeshPartCount());
                s->
                Assert::AreEqual(true, s->hasBoxSet());
                Assert::IsTrue(nullptr != s->getBoxSet());
                Assert::AreEqual(3, s->getBoxSet()->getNodeCount());
            }

            TEST_METHOD(local_data_of_gimpact_mesh_shape_in_collision_object)
            {
                auto s = std::make_unique<btGImpactMeshShape>(&c_unit_square_data);
                s->setMargin(0.001);
                s->updateBound();

                auto o = std::make_unique<btCollisionObject>();
                o->setCollisionShape(s.get());
                o->setWorldTransform(
                    btTransform(btQuaternion(0, 0, 0), btVector3(0, 0, 0)));
            }
        };
    }
}