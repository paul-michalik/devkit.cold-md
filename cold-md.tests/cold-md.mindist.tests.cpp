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

            TEST_METHOD(access_local_data_of_gimpact_mesh_shape)
            {
                auto s = std::make_unique<btGImpactMeshShape>(&c_unit_square_data);
                s->setMargin(c_margin);
                s->updateBound();

                {
                    btTransform t(btQuaternion(0, 0, 0), btVector3(0, 0, 0));
                    btVector3 min, max;
                    s->getAabb(t, min, max);
                    Logger::WriteMessage(mstest_utils::wlog_message()
                        << "min: " << min.getX() << ", " << min.getY() << ", " << min.getZ() << ": "
                        << btVector3(-1, -1, 0).distance(min) << std::endl
                        << "max: " << max.getX() << ", " << max.getY() << ", " << max.getZ() << ": "
                        << btVector3(+1, +1, 0).distance(max) << std::endl << endl);

                    Assert::IsTrue(btVector3(-1, -1, 0).distance(min) <= 2 * c_margin);
                    Assert::IsTrue(btVector3(+1, +1, 0).distance(max) <= 2 * c_margin);
                }

                Assert::AreEqual(1, s->getMeshPartCount());
                Assert::IsTrue(nullptr != s->getMeshPart(0));
                Assert::IsTrue(nullptr != s->getMeshPart(0)->getBoxSet());

                // This is necessary, otherwise the data just isn't there.
                // I don't see a reason, maybe google saw one...
                s->getMeshPart(0)->lockChildShapes();
                Assert::IsTrue(0 < s->getMeshPart(0)->getBoxSet()->getNodeCount());
                Assert::IsTrue(nullptr != s->getMeshPart(0)->getBoxSet()->getPrimitiveManager());
                Assert::AreEqual(2, s->getMeshPart(0)->getBoxSet()->getPrimitiveManager()->get_primitive_count());
                {
                    btPrimitiveTriangle t;
                    auto pm = s->getMeshPart(0)->getBoxSet()->getPrimitiveManager();
                    pm->get_primitive_triangle(0, t);
                    Logger::WriteMessage(mstest_utils::wlog_message()
                        << "0: " << t.m_vertices[0].getX() << ", " << t.m_vertices[0].getY() << ", " << t.m_vertices[0].getZ() << std::endl
                        << "1: " << t.m_vertices[1].getX() << ", " << t.m_vertices[1].getY() << ", " << t.m_vertices[2].getZ() << std::endl
                        << "2: " << t.m_vertices[2].getX() << ", " << t.m_vertices[2].getY() << ", " << t.m_vertices[2].getZ() << endl);

                    Assert::IsTrue(btVector3(-1, -1, 0).distance(t.m_vertices[0]) < c_margin);
                    Assert::IsTrue(btVector3(+1, -1, 0).distance(t.m_vertices[1]) < c_margin);
                    Assert::IsTrue(btVector3(+1, +1, 0).distance(t.m_vertices[2]) < c_margin);
                }
                {
                    btAABB b;
                    auto pm = s->getMeshPart(0)->getBoxSet()->getPrimitiveManager();
                    pm->get_primitive_box(0, b);
                    Assert::IsTrue(btVector3(-1, -1, 0).distance(b.m_min) <= 2 * c_margin);
                    Assert::IsTrue(btVector3(+1, +1, 0).distance(b.m_max) <= 2 * c_margin);

                    pm->get_primitive_box(1, b);
                    Assert::IsTrue(btVector3(-1, -1, 0).distance(b.m_min) <= 2 * c_margin);
                    Assert::IsTrue(btVector3(+1, +1, 0).distance(b.m_max) <= 2 * c_margin);
                }

                s->getMeshPart(0)->unlockChildShapes();

                // not implemented!
                //Assert::AreEqual(1, s->getNumChildShapes());
                //Assert::IsTrue(nullptr != s->getChildShape(0));
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