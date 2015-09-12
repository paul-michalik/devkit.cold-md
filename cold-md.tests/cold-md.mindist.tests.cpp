#include "stdafx.h"
#include "CppUnitTest.h"
#include <cold-md/bullet/world.h>
#include <cold-md/bullet/min_dist_calculator.h>

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
            (int)c_unit_square_tri.size() / 3u, &c_unit_square_tri[0], 3 * sizeof(int),
            (int)c_unit_square_ver.size() / 3u, &c_unit_square_ver[0], 3 * sizeof(btScalar));

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


                // for some reason, shape'shape own box set is there but it'shape empty!
                Assert::IsTrue(nullptr != s->getBoxSet());
                Assert::IsTrue(0 == s->getBoxSet()->getNodeCount());

                // This invokes an assertion!?
                //Assert::AreEqual(0, shape->getNumChildShapes());


                Assert::AreEqual(1, s->getMeshPartCount());

                auto actual_mesh = s->getMeshPart(0);

                Assert::IsTrue(nullptr != actual_mesh);

                // This is necessary, otherwise the data just isn't there.
                // I don't see a reason, maybe google saw one...
                actual_mesh->lockChildShapes();

                Logger::WriteMessage(mstest_utils::wlog_message() <<
                    "actual_mesh->getNumChildShapes(): " << actual_mesh->getNumChildShapes() << endl);
                Assert::AreEqual(2, actual_mesh->getNumChildShapes());

                Assert::IsTrue(nullptr != actual_mesh->getBoxSet());
                Logger::WriteMessage(mstest_utils::wlog_message() <<
                    "getBoxSet()->getNodeCount(): " << actual_mesh->getBoxSet()->getNodeCount() << endl);
                Assert::IsTrue(0 < actual_mesh->getBoxSet()->getNodeCount());
                // one node at top, two subnodes for each child consisting which are leaves with triangles in this case!
                //    r
                //  /   \
                // n1   n2
                // |    |
                // t1   t2
                Assert::AreEqual(3, actual_mesh->getBoxSet()->getNodeCount());


                auto actual_mesh_pm = actual_mesh->getPrimitiveManager();
                Assert::IsTrue(nullptr != actual_mesh_pm);
                Logger::WriteMessage(mstest_utils::wlog_message() 
                    << "getPrimitiveManager()->get_primitive_count(): " 
                    << actual_mesh_pm->get_primitive_count() << endl);
                Assert::AreEqual(2, actual_mesh_pm->get_primitive_count());

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
                //Assert::AreEqual(1, shape->getNumChildShapes());
                //Assert::IsTrue(nullptr != shape->getChildShape(0));
            }

            TEST_METHOD(access_box_set_data_of_gimpact_mesh)
            {
                auto shape = std::make_unique<btGImpactMeshShape>(&c_unit_square_data);
                shape->setMargin(c_margin);
                shape->updateBound();

                // mesh consists of mesh parts, this shape has exactly one part
                // which contains all the data...
                Assert::AreEqual(1, shape->getMeshPartCount());
                auto mesh_part = shape->getMeshPart(0);
                Assert::IsTrue(nullptr != mesh_part);

                // This is necessary, otherwise the data just isn't there...
                // probably relevant when storing data on gpu device...

                mesh_part->lockChildShapes();

                BOOST_SCOPE_EXIT(mesh_part)
                {
                    mesh_part->unlockChildShapes();
                } BOOST_SCOPE_EXIT_END;

                /* one node at top, two subnodes for each child which 
                 are leaves with triangles in this case!
                    r   <- mesh_part, box_set = mesh_part->getBoxSet()
                  /   \
                 n1   n2 <- box_set->getNode(0), box_set->getNode(1)
                 |    |  <- box_set->isLeafNode(0), box_set->isLeafNode(1) = true
                 t1   t2 
                */

                Logger::WriteMessage(mstest_utils::wlog_message() <<
                    "mesh_part->getNumChildShapes(): " << mesh_part->getNumChildShapes() << endl);
                Assert::AreEqual(2, mesh_part->getNumChildShapes());
                // nope, this asserts immediately, seems to implemented only for compund shapes!
                //Assert::IsTrue(nullptr != mesh_part->getChildShape(0));
                //Logger::WriteMessage(mstest_utils::wlog_message() <<
                //    "mesh_part->getChildShape(0)->getShapeType(): " << mesh_part->getChildShape(0)->getShapeType() << endl);
                //Assert::IsTrue(BroadphaseNativeTypes::GIMPACT_SHAPE_PROXYTYPE != mesh_part->getChildShape(0)->getShapeType());

                auto box_set = mesh_part->getBoxSet();
                Assert::IsTrue(nullptr != box_set);
                Logger::WriteMessage(mstest_utils::wlog_message() <<
                    "getBoxSet()->getNodeCount(): " << box_set->getNodeCount() << endl);
                Assert::IsTrue(0 < box_set->getNodeCount());
                Assert::AreEqual(3, box_set->getNodeCount());

                // node with index 0 is root
                Assert::IsFalse(box_set->isLeafNode(0));
                // left and right node are leaves in this case
                Assert::IsTrue(box_set->isLeafNode(box_set->getLeftNode(0)));
                Assert::IsTrue(box_set->isLeafNode(box_set->getRightNode(0)));

                {
                    btPrimitiveTriangle t;
                    box_set->getNodeTriangle(box_set->getLeftNode(0), t);
                    Logger::WriteMessage(mstest_utils::wlog_message() <<
                        "triangle(0): " << std::endl <<
                        "0: " << t.m_vertices[0].getX() << ", " << t.m_vertices[0].getY() << ", " << t.m_vertices[0].getZ() << std::endl <<
                        "1: " << t.m_vertices[1].getX() << ", " << t.m_vertices[1].getY() << ", " << t.m_vertices[1].getZ() << std::endl <<
                        "2: " << t.m_vertices[2].getX() << ", " << t.m_vertices[2].getY() << ", " << t.m_vertices[2].getZ() << std::endl << endl);
                    Assert::IsTrue(btVector3(-1, -1, 0).distance(t.m_vertices[0]) < c_margin);
                    Assert::IsTrue(btVector3(+1, -1, 0).distance(t.m_vertices[1]) < c_margin);
                    Assert::IsTrue(btVector3(+1, +1, 0).distance(t.m_vertices[2]) < c_margin);
                }
                {
                    btPrimitiveTriangle t;
                    box_set->getNodeTriangle(box_set->getRightNode(0), t);
                    Logger::WriteMessage(mstest_utils::wlog_message() <<
                        "triangle(1): " << std::endl <<
                        "0: " << t.m_vertices[0].getX() << ", " << t.m_vertices[0].getY() << ", " << t.m_vertices[0].getZ() << std::endl <<
                        "1: " << t.m_vertices[1].getX() << ", " << t.m_vertices[1].getY() << ", " << t.m_vertices[1].getZ() << std::endl <<
                        "2: " << t.m_vertices[2].getX() << ", " << t.m_vertices[2].getY() << ", " << t.m_vertices[2].getZ() << std::endl << endl);
                    Assert::IsTrue(btVector3(+1, +1, 0).distance(t.m_vertices[0]) < c_margin);
                    Assert::IsTrue(btVector3(-1, +1, 0).distance(t.m_vertices[1]) < c_margin);
                    Assert::IsTrue(btVector3(-1, -1, 0).distance(t.m_vertices[2]) < c_margin);
                }
                {
                    btAABB b;
                    box_set->getNodeBound(0, b);
                    Logger::WriteMessage(mstest_utils::wlog_message() <<
                        "getBoxSet()->getNodeBound(0, b): " <<
                        b.m_min.getX() << ", " << b.m_min.getY() << ", " << b.m_min.getZ() << std::endl <<
                        b.m_max.getX() << ", " << b.m_max.getY() << ", " << b.m_max.getZ() << endl);
                    Assert::IsTrue(btVector3(-1, -1, 0).distance(b.m_min) <= 2 * c_margin);
                    Assert::IsTrue(btVector3(+1, +1, 0).distance(b.m_max) <= 2 * c_margin);
                }
                {
                    btAABB b;
                    box_set->getNodeBound(1, b);
                    Logger::WriteMessage(mstest_utils::wlog_message() <<
                        "getBoxSet()->getNodeBound(1, b): " <<
                        b.m_min.getX() << ", " << b.m_min.getY() << ", " << b.m_min.getZ() << std::endl <<
                        b.m_max.getX() << ", " << b.m_max.getY() << ", " << b.m_max.getZ() << endl);
                    Assert::IsTrue(btVector3(-1, -1, 0).distance(b.m_min) <= 2 * c_margin);
                    Assert::IsTrue(btVector3(+1, +1, 0).distance(b.m_max) <= 2 * c_margin);
                }
            }

            TEST_METHOD(access_primitive_data_of_gimpact_mesh)
            {
                auto shape = std::make_unique<btGImpactMeshShape>(&c_unit_square_data);
                shape->setMargin(c_margin);
                shape->updateBound();

                Assert::AreEqual(1, shape->getMeshPartCount());
                auto mesh_part = shape->getMeshPart(0);
                Assert::IsTrue(nullptr != mesh_part);

                // This is necessary, otherwise the data just isn't there...
                // probably relevant when storing data on gpu device...
                mesh_part->lockChildShapes();

                BOOST_SCOPE_EXIT(mesh_part)
                {
                    mesh_part->unlockChildShapes();
                } BOOST_SCOPE_EXIT_END;

                auto prim_manager = mesh_part->getPrimitiveManager();
                Assert::IsTrue(nullptr != prim_manager);
                Logger::WriteMessage(mstest_utils::wlog_message()
                    << "getPrimitiveManager()->get_primitive_count(): "
                    << prim_manager->get_primitive_count() << endl);
                Assert::AreEqual(2, prim_manager->get_primitive_count());

                {
                    btPrimitiveTriangle t;
                    prim_manager->get_primitive_triangle(0, t);
                    Logger::WriteMessage(mstest_utils::wlog_message()
                        << "triangle[0]: " << std::endl
                        << "0: " << t.m_vertices[0].getX() << ", " << t.m_vertices[0].getY() << ", " << t.m_vertices[0].getZ() << std::endl
                        << "1: " << t.m_vertices[1].getX() << ", " << t.m_vertices[1].getY() << ", " << t.m_vertices[2].getZ() << std::endl
                        << "2: " << t.m_vertices[2].getX() << ", " << t.m_vertices[2].getY() << ", " << t.m_vertices[2].getZ() << endl);

                    Assert::IsTrue(btVector3(-1, -1, 0).distance(t.m_vertices[0]) < c_margin);
                    Assert::IsTrue(btVector3(+1, -1, 0).distance(t.m_vertices[1]) < c_margin);
                    Assert::IsTrue(btVector3(+1, +1, 0).distance(t.m_vertices[2]) < c_margin);
                }
                {
                    btAABB b;
                    prim_manager->get_primitive_box(0, b);
                    Assert::IsTrue(btVector3(-1, -1, 0).distance(b.m_min) <= 2 * c_margin);
                    Assert::IsTrue(btVector3(+1, +1, 0).distance(b.m_max) <= 2 * c_margin);

                    prim_manager->get_primitive_box(1, b);
                    Assert::IsTrue(btVector3(-1, -1, 0).distance(b.m_min) <= 2 * c_margin);
                    Assert::IsTrue(btVector3(+1, +1, 0).distance(b.m_max) <= 2 * c_margin);
                }
            }

            void traverse_box_tree_in_order(btGImpactBoxSet const* box_set_)
            {
                btAABB b;
                std::queue<int> queue;

                queue.push(0);

                while (!queue.empty()) {
                    auto i = queue.front();
                    queue.pop();
                    
                    if (!box_set_->isLeafNode(i)) {
                        queue.push(box_set_->getLeftNode(i));
                        queue.push(box_set_->getRightNode(i));
                    }

                    box_set_->getNodeBound(i, b);

                    Logger::WriteMessage(mstest_utils::wlog_message() <<
                        "box[" << i << "]" << std::endl << 
                        b.m_min.getX() << ", " << b.m_min.getY() << ", " << b.m_min.getZ() << std::endl <<
                        b.m_max.getX() << ", " << b.m_max.getY() << ", " << b.m_max.getZ() << endl);
                }
            }

            void traverse(btGImpactMeshShape* mesh_shape_)
            {
                for (auto mp_index = 0; mp_index < mesh_shape_->getMeshPartCount(); ++mp_index) {
                    auto mesh_part = mesh_shape_->getMeshPart(mp_index);

                    mesh_part->lockChildShapes();

                    BOOST_SCOPE_EXIT(mesh_part)
                    {
                        mesh_part->unlockChildShapes();
                    } BOOST_SCOPE_EXIT_END;

                    Assert::IsTrue(nullptr != mesh_part->getBoxSet());

                    if (auto box_set = mesh_part->getBoxSet()) {
                        traverse_box_tree_in_order(box_set);
                    }
                }
            }

            TEST_METHOD(can_traverse_gimpact_mesh_shape)
            {
                auto shape = std::make_unique<btGImpactMeshShape>(&c_unit_square_data);
                shape->setMargin(c_margin);
                shape->updateBound();

                // mesh consists of mesh parts, this shape has exactly one part
                // which contains all the data...
                Assert::AreEqual(1, shape->getMeshPartCount());
                auto mesh_part = shape->getMeshPart(0);
                Assert::IsTrue(nullptr != mesh_part);

                // This is necessary, otherwise the data just isn't there...
                // probably relevant when storing data on gpu device...

                /* one node at top, two subnodes for each child which
                are leaves with triangles in this case!
                r   <- mesh_part, box_set = mesh_part->getBoxSet()
                /   \
                n1   n2 <- box_set->getNode(0), box_set->getNode(1)
                |    |  <- box_set->isLeafNode(0), box_set->isLeafNode(1) = true
                t1   t2
                */

                traverse(shape.get());
            }

            struct min_dist_calculator_fixture_a_b_ {
                cold::bullet::world world;
                btGImpactMeshShape shape_a = &c_unit_square_data;
                btGImpactMeshShape shape_b = &c_unit_square_data;
                btCollisionObject obj_a, obj_b;

                min_dist_calculator_fixture_a_b_(
                    btTransform const & transf_a_, 
                    btTransform const & transf_b_)
                {
                    shape_a.setMargin(c_margin);
                    shape_a.updateBound();
                    obj_a.setCollisionShape(&shape_a);
                    obj_a.setWorldTransform(transf_a_);

                    shape_b.setMargin(c_margin);
                    shape_b.updateBound();
                    obj_b.setCollisionShape(&shape_b);
                    obj_b.setWorldTransform(transf_b_);
                }
            };

            TEST_METHOD(min_dist_calculator_from_collision_a_b)
            {
                // arrange (collision):
                min_dist_calculator_fixture_a_b_ fixture(
                    btTransform(btQuaternion(0., 0., 0.), btVector3(-0.5, 0, 0)),
                    btTransform(btQuaternion(0., 0., 0.), btVector3(+0.5, 0, 0)));
               
                // act
                NSBulletPhysicsExt::min_dist_calculator calc(fixture.world.get_world());

                // assert:
                auto result = calc.calculate();
                Assert::AreEqual(2u, result.size());
            }

            TEST_METHOD(min_dist_calculator_calculate_a_b)
            {
            }

            TEST_METHOD(min_dist_calculator_calculate_pair_a_b)
            {
            }

        };
    }
}