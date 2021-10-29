#include <gtest/gtest.h>

#include <Eigen>
#include "octree.hpp"

class OctreeTest: public ::testing::Test {
    protected:
        void SetUp() override {
            root1 = new OctreeNode(0, 1, 0, 1, 0, 1);
            root2 = new OctreeNode(-1, 1, -1, 1, -1, 1);
        }

        void TearDown() override {
            delete root1;
            delete root2;
        }

        OctreeNode* root1;
        OctreeNode* root2;
};

TEST_F(OctreeTest, AddObjectTest) {
    Eigen::RowVectorXd m1 {{1, 2, 1}};
    Eigen::Matrix3Xd pos1 {
        {0.1, 0.4, 0.1},
        {0.4, 0.4, 0.1},
        {0.0, 0.0, 0.0}
    };

    root1->addObject(m1(0), pos1.col(0));
    root1->addObject(m1(1), pos1.col(1));
    root1->addObject(m1(2), pos1.col(2));

    EXPECT_EQ(root1->isEmpty, false);
    EXPECT_EQ(root1->isExternal, false);
    EXPECT_NEAR(root1->totalMass, 4, 1e-6);
    EXPECT_NEAR(root1->centerOfMass(0), 0.25, 1e-6);
    EXPECT_NEAR(root1->centerOfMass(1), 0.325, 1e-6);
    EXPECT_NEAR(root1->centerOfMass(2), 0.0, 1e-6);

    OctreeNode* mid  = root1->children[0][0][0];
    EXPECT_EQ(mid->isEmpty, false);
    EXPECT_EQ(mid->isExternal, false);
    EXPECT_NEAR(mid->totalMass, 4, 1e-6);
    EXPECT_NEAR(mid->centerOfMass(0), 0.25, 1e-6);
    EXPECT_NEAR(mid->centerOfMass(1), 0.325, 1e-6);
    EXPECT_NEAR(mid->centerOfMass(2), 0.0, 1e-6);

    OctreeNode* ext0 = mid->children[0][1][0];
    EXPECT_EQ(ext0->isEmpty, false);
    EXPECT_EQ(ext0->isExternal, true);
    EXPECT_NEAR(ext0->totalMass, 1, 1e-6);
    EXPECT_NEAR(ext0->centerOfMass(0), 0.1, 1e-6);
    EXPECT_NEAR(ext0->centerOfMass(1), 0.4, 1e-6);
    EXPECT_NEAR(ext0->centerOfMass(2), 0.0, 1e-6);

    OctreeNode* ext1 = mid->children[0][1][1];
    EXPECT_EQ(ext1->isEmpty, false);
    EXPECT_EQ(ext1->isExternal, true);
    EXPECT_NEAR(ext1->totalMass, 2, 1e-6);
    EXPECT_NEAR(ext1->centerOfMass(0), 0.4, 1e-6);
    EXPECT_NEAR(ext1->centerOfMass(1), 0.4, 1e-6);
    EXPECT_NEAR(ext1->centerOfMass(2), 0.0, 1e-6);

    OctreeNode* ext2 = mid->children[0][0][0];
    EXPECT_EQ(ext1->isEmpty, false);
    EXPECT_EQ(ext1->isExternal, true);
    EXPECT_NEAR(ext2->totalMass, 1, 1e-6);
    EXPECT_NEAR(ext2->centerOfMass(0), 0.1, 1e-6);
    EXPECT_NEAR(ext2->centerOfMass(1), 0.1, 1e-6);
    EXPECT_NEAR(ext2->centerOfMass(2), 0.0, 1e-6);

    OctreeNode* empty = root1->children[1][0][0];
    EXPECT_EQ(empty->isEmpty, true);
    EXPECT_EQ(empty->isExternal, true);
}
