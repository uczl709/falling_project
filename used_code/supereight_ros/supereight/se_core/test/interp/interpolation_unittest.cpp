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
#include <cmath>
#include "octree.hpp"
#include "utils/math_utils.h"
#include "gtest/gtest.h"
#include "functors/axis_aligned_functor.hpp"
#include "io/ply_io.hpp"
#include "io/vtk-io.h"
#include "algorithms/balancing.hpp"
#include "interpolation/idw_interpolation.hpp"

struct TestVoxelT {
  typedef float VoxelData;
  static inline VoxelData empty(){ return 0.f; }
  static inline VoxelData initValue(){ return 1.f; }
};

float test_fun(float x, float y, float z) {
  return se::math::sq(z) + std::sin(2*x + y);
}

float sphere_dist(const Eigen::Vector3f& p, const Eigen::Vector3f& C,
    const float radius) {
  const Eigen::Vector3f dir = (C - p).normalized();
  const Eigen::Vector3f vox_o = p - C;

  const float a = dir.dot(dir);
  const float b = 2 * dir.dot(vox_o);
  const float c = vox_o.dot(vox_o) - radius*radius;
  const float delta = b*b - 4*a*c;
  float dist = std::numeric_limits<int>::max();
  if(delta > 0) {
    dist = std::min(-b + sqrtf(delta), -b - sqrtf(delta));
    dist /= 2*a;
  }
  return dist;
}

class InterpolationTest : public ::testing::Test {
  protected:
    virtual void SetUp() {
      unsigned size = 256;
      float dim = 5.f;
      oct_.init(size, dim); // 5 meters

      const unsigned center = size >> 1;
      const unsigned radius = size >> 2;
      const Eigen::Vector3f C(center, center, center);

      for(int z = center - radius; z < (center + radius); ++z) {
        for(int y = center - radius; y < (center + radius); ++y) {
          for(int x = center - radius; x < (center + radius); ++x) {
            const Eigen::Vector3i vox(x, y, z);
            const float dist = fabs(sphere_dist(vox.cast<float>(), C, radius));
            if(dist > 20.f && dist < 25.f) {
              alloc_list.push_back(oct_.hash(vox(0), vox(1), vox(2)));
            }
          }
        }
      }
      oct_.allocate(alloc_list.data(), alloc_list.size());

      auto circle_dist = [C, radius](auto& handler, const Eigen::Vector3i& v) {
        float data = sphere_dist(v.cast<float>(), C, radius);
        handler.set(data);
      };
      se::functor::axis_aligned_map(oct_, circle_dist);

      se::print_octree("./test-sphere.ply", oct_);
      {
        std::stringstream f;
        f << "./sphere-interp.vtk";
        save3DSlice(oct_, Eigen::Vector3i(0, oct_.size()/2, 0),
            Eigen::Vector3i(oct_.size(), oct_.size()/2 + 1, oct_.size()),
            [](const float& val) { return val; }, oct_.block_depth, f.str().c_str());
      }

      // balance and print.
      se::balance(oct_);
      se::functor::axis_aligned_map(oct_, circle_dist);
      se::print_octree("./test-sphere-balanced.ply", oct_);
      {
        std::stringstream f;
        f << "./sphere-interp-balanced.vtk";
        save3DSlice(oct_, Eigen::Vector3i(0, oct_.size()/2, 0),
            Eigen::Vector3i(oct_.size(), oct_.size()/2 + 1, oct_.size()),
            [](const float& val) { return val; }, oct_.block_depth, f.str().c_str());
      }

    }

  typedef se::Octree<TestVoxelT> OctreeF;
  OctreeF oct_;
  std::vector<se::key_t> alloc_list;
};

TEST_F(InterpolationTest, IDWInterp) {
  Eigen::Vector3f pos(128.4f, 129.1, 127.5);
  auto select =  [](const TestVoxelT::VoxelData& val){ return val; };
  se::internal::idw_interp<TestVoxelT::VoxelData>(oct_, pos, select);

}

// TEST_F(InterpolationTest, InterpAtPoints) {
//
//   auto test = [this](auto& handler, const Eigen::Vector3i& v) {
//     auto data = handler.get();
//     TestVoxelT::VoxelData interpolated = oct_.interp(make_float3(v(0), v(1), v(2)), [](const auto& val){ return val(0); });
//     ASSERT_EQ(data(0), interpolated);
//   };
//
//   se::functor::axis_aligned<TestVoxelT, Octree, decltype(test)>
//     funct_test(oct_, test);
//   funct_test.apply();
// }
