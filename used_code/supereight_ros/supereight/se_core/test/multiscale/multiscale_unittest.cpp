/*

Copyright 2016 Emanuele Vespa, Imperial College London

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#include "octree.hpp"
#include "octant_ops.hpp"
#include "node_iterator.hpp"
#include "utils/math_utils.h"
#include "gtest/gtest.h"
#include <random>

struct TestVoxelT {
  typedef float VoxelData;
  static inline VoxelData empty(){ return 0.f; }
  static inline VoxelData initValue(){ return 1.f; }
};

class MultiscaleTest : public ::testing::Test {
  protected:
    virtual void SetUp() {
      oct_.init(512, 5);

    }

  typedef se::Octree<TestVoxelT> OctreeF;
  OctreeF oct_;
};

TEST_F(MultiscaleTest, Init) {
  EXPECT_EQ(oct_.get(137, 138, 130), TestVoxelT::initValue());
}

TEST_F(MultiscaleTest, PlainAlloc) {
  const Eigen::Vector3i blocks[2] = {{56, 12, 254}, {87, 32, 423}};
  se::key_t alloc_list[2];
  for(int i = 0; i < 2; ++i) {
    alloc_list[i] = oct_.hash(blocks[i](0), blocks[i](1), blocks[i](2));
  }
  oct_.allocate(alloc_list, 2);

  oct_.set(56, 12, 254, 3.f);

  EXPECT_EQ(oct_.get(56, 12, 254), 3.f);
  EXPECT_EQ(oct_.get(106, 12, 254), TestVoxelT::initValue());
  EXPECT_NE(oct_.get(106, 12, 254), 3.f);
}

TEST_F(MultiscaleTest, ScaledAlloc) {
  const Eigen::Vector3i blocks[2] = {{200, 12, 25}, {87, 32, 423}};
  se::key_t alloc_list[2];
  for(int i = 0; i < 2; ++i) {
    alloc_list[i] = oct_.hash(blocks[i](0), blocks[i](1), blocks[i](2), 5);
  }

  oct_.allocate(alloc_list, 2);
  se::Node<TestVoxelT>* n = oct_.fetch_octant(87, 32, 420, 5);
  ASSERT_TRUE(n != NULL);
  n->value_[0] = 10.f;
  EXPECT_EQ(oct_.get(87, 32, 420), 10.f);
}

TEST_F(MultiscaleTest, Iterator) {
  const Eigen::Vector3i blocks[1] = {{56, 12, 254}};
  se::key_t alloc_list[1];
  alloc_list[0] = oct_.hash(blocks[0](0), blocks[0](1), blocks[0](2));

  oct_.allocate(alloc_list, 1);
  se::node_iterator<TestVoxelT> it(oct_);
  se::Node<TestVoxelT> * node = it.next();
  for(int i = 512; node != nullptr; node = it.next(), i /= 2){
    const Eigen::Vector3i coords = se::keyops::decode(node->code_);
    const int side = node->side_;
    const se::Octree<TestVoxelT>::VoxelData val = node->value_[0];
    EXPECT_EQ(side, i);
  }
}

TEST_F(MultiscaleTest, ChildrenMaskTest) {
  const Eigen::Vector3i blocks[10] = {{56, 12, 254}, {87, 32, 423}, {128, 128, 128},
    {136, 128, 128}, {128, 136, 128}, {136, 136, 128},
    {128, 128, 136}, {136, 128, 136}, {128, 136, 136}, {136, 136, 136}};
  se::key_t alloc_list[10];
  for(int i = 0; i < 10; ++i) {
    alloc_list[i] = oct_.hash(blocks[i](0), blocks[i](1), blocks[i](2), 5);
  }

  oct_.allocate(alloc_list, 10);
  const se::MemoryPool<se::Node<TestVoxelT> >& nodes = oct_.getNodesBuffer();
  const size_t num_nodes = nodes.size();
  for(size_t i = 0; i < num_nodes; ++i) {
    se::Node<TestVoxelT>* n = nodes[i];
    for(int c = 0; c < 8; ++c) {
      if(n->child(c)) {
        ASSERT_TRUE(n->children_mask_ & (1 << c));
      }
    }
  }
}

TEST_F(MultiscaleTest, OctantAlloc) {
  const Eigen::Vector3i blocks[10] = {{56, 12, 254}, {87, 32, 423}, {128, 128, 128},
    {136, 128, 128}, {128, 136, 128}, {136, 136, 128},
    {128, 128, 136}, {136, 128, 136}, {128, 136, 136}, {136, 136, 136}};
  se::key_t alloc_list[10];
  for(int i = 0; i < 10; ++i) {
    alloc_list[i] = oct_.hash(blocks[i](0), blocks[i](1), blocks[i](2));
  }

  alloc_list[2] = alloc_list[2] | 3;
  alloc_list[9] = alloc_list[2] | 5;
  oct_.allocate(alloc_list, 10);
  se::Node<TestVoxelT> * octant = oct_.fetch_octant(blocks[4](0), blocks[4](1),
      blocks[4](2), 3);
  ASSERT_TRUE(octant != NULL);
  octant = oct_.fetch_octant(blocks[9](0), blocks[9](1),
      blocks[9](2), 6);
  ASSERT_TRUE(octant == NULL);
}

TEST_F(MultiscaleTest, SingleInsert) {
  Eigen::Vector3i vox(32, 208, 44);
  const int side = se::VoxelBlock<TestVoxelT>::side;
  se::VoxelBlock<TestVoxelT> * n = oct_.insert(vox(0), vox(1), vox(2));
  Eigen::Vector3i coords = n->coordinates();
  Eigen::Vector3i rounded = side * (vox/side);
  ASSERT_TRUE(coords == rounded);
}

TEST_F(MultiscaleTest, MultipleInsert) {
  OctreeF tree;
  tree.init(1024, 10);
  const int side = se::VoxelBlock<TestVoxelT>::side;
  const int max_depth = log2(tree.size());
  const int leaves_level = max_depth - log2(side);
  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen(1); //Standard mersenne_twister_engine seeded with rd()
  std::uniform_int_distribution<> dis(0, 1023);

  int num_tested = 0;
  for(int i = 1, edge = tree.size()/2; i <= leaves_level; ++i, edge = edge/2) {
    for(int j = 0; j < 20; ++j) {
      Eigen::Vector3i vox(dis(gen), dis(gen), dis(gen));
      se::Node<TestVoxelT> * n = tree.insert(vox(0), vox(1), vox(2), i);
      se::Node<TestVoxelT> * n1 = tree.fetch_octant(vox(0), vox(1), vox(2), i);
      Eigen::Vector3i rounded = edge * (vox/edge);
      Eigen::Vector3i coords = se::keyops::decode(n1->code_);

      // Check expected coordinates
      ASSERT_TRUE(coords == rounded);
      // Should not have any children up to this level
      ASSERT_TRUE(n1->children_mask_ == 0);
      ++num_tested;
    }
  }
}

struct TestVoxel2T {
  typedef Eigen::Vector3i VoxelData;
  static inline VoxelData empty(){ return Eigen::Vector3i::Zero(); }
  static inline VoxelData initValue(){ return Eigen::Vector3i::Zero(); }
};

TEST(MultiscaleBlock, ReadWrite) {
  se::Octree<TestVoxel2T> tree;
  tree.init(1024, 10);
  const int side = se::VoxelBlock<TestVoxel2T>::side;
  const int max_depth = log2(tree.size());
  const int leaves_level = max_depth - log2(side);
  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen(1); //Standard mersenne_twister_engine seeded with rd()
  std::uniform_int_distribution<> dis(0, 1023);
  std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> voxels;

  const int n = 50;
  for(int j = 0; j < n; ++j) {
    voxels.push_back(Eigen::Vector3i(dis(gen), dis(gen), dis(gen)));
    const Eigen::Vector3i& curr = voxels.back();
    auto * n = tree.insert(curr.x(), curr.y(), curr.z());
    const Eigen::Vector3i& base = n->coordinates();
    for(int z = 0; z < side; ++z) {
      for(int y = 0; y < side; ++y) {
        for(int x = 0; x < side; ++x) {
          const Eigen::Vector3i pos    = base + Eigen::Vector3i(x, y, z);
          const Eigen::Vector3i pos_up = (pos/2) * 2;
          const Eigen::Vector3i pos_up2 = (pos/4) * 4;
          n->data(pos, pos);
          n->data(pos, 1, pos_up);
          n->data(pos, 2, pos_up2);
        }
      }
    }
  }

  for(int i = 0; i < voxels.size(); ++i) {
    const Eigen::Vector3i& curr = voxels[i];
    auto * n = tree.fetch(curr.x(), curr.y(), curr.z());
    const Eigen::Vector3i& base = n->coordinates();
    for(int z = 0; z < side; ++z) {
      for(int y = 0; y < side; ++y) {
        for(int x = 0; x < side; ++x) {
          const Eigen::Vector3i pos    = base + Eigen::Vector3i(x, y, z);
          const Eigen::Vector3i pos_up = (pos/2) * 2;
          const Eigen::Vector3i pos_up2 = (pos/4) * 4;
          ASSERT_TRUE(n->data(pos).cwiseEqual(pos).all());
          ASSERT_TRUE(n->data(pos, 1).cwiseEqual(pos_up).all());
          ASSERT_TRUE(n->data(pos, 2).cwiseEqual(pos_up2).all());
        }
      }
    }
  }
}
