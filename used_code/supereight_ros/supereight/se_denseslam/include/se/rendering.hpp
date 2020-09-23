/*

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.


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

#ifndef __RENDERING_HPP
#define __RENDERING_HPP

#include <cstdint>

#include <sophus/se3.hpp>

#include <se/utils/math_utils.h>
#include <se/commons.h>
#include <lodepng.h>
#include <se/timings.h>
#include <se/continuous/volume_template.hpp>
#include <se/image/image.hpp>
#include <se/voxel_block_ray_iterator.hpp>



template <typename T>
using Volume = VolumeTemplate<T, se::Octree>;



namespace se {
  namespace internal {
    static se::Image<int> scale_image(640, 480);
    static std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>
      color_map =
      {
        {102, 194, 165},
        {252, 141, 98},
        {141, 160, 203},
        {231, 138, 195},
        {166, 216, 84},
        {255, 217, 47},
        {229, 196, 148},
        {179, 179, 179},
      };
  }
}



template<typename T>
void raycastKernel(const Volume<T>&            volume,
                   se::Image<Eigen::Vector3f>& vertex,
                   se::Image<Eigen::Vector3f>& normal,
                   const Eigen::Matrix4f&      view,
                   const float                 near_plane,
                   const float                 far_plane,
                   const float                 mu,
                   const float                 step,
                   const float                 large_step) {

  TICK();
#pragma omp parallel for
  for (int y = 0; y < vertex.height(); y++) {
#pragma omp simd
    for (int x = 0; x < vertex.width(); x++) {
      const Eigen::Vector2i pos(x, y);
      const Eigen::Vector3f dir =
          (view.topLeftCorner<3, 3>() * Eigen::Vector3f(x, y, 1.f)).normalized();
      const Eigen::Vector3f transl = view.topRightCorner<3, 1>();
      se::VoxelBlockRayIterator<typename T::VoxelType> ray(*volume._map_index, transl, dir, near_plane, far_plane);
      ray.next();
      const float t_min = ray.tcmin(); /* Get distance to the first intersected block */
      const Eigen::Vector4f hit = t_min > 0.f
          ? T::raycast(volume, transl, dir, t_min, ray.tmax(), mu, step, large_step)
          : Eigen::Vector4f::Zero();
      if (hit.w() >= 0.f) {
        vertex[x + y * vertex.width()] = hit.head<3>();
        Eigen::Vector3f surface_normal = volume.grad(hit.head<3>(),
            int(hit.w() + 0.5f),
            [](const auto& val){ return val.x; });
        se::internal::scale_image(x, y) = static_cast<int>(hit.w());
        if (surface_normal.norm() == 0.f) {
          normal[pos.x() + pos.y() * normal.width()] = Eigen::Vector3f(INVALID, 0.f, 0.f);
        } else {
          // Invert normals for TSDF representations.
          normal[pos.x() + pos.y() * normal.width()] = T::invert_normals
              ? (-1.f * surface_normal).normalized()
              : surface_normal.normalized();
        }
      } else {
        vertex[pos.x() + pos.y() * vertex.width()] = Eigen::Vector3f::Zero();
        normal[pos.x() + pos.y() * normal.width()] = Eigen::Vector3f(INVALID, 0.f, 0.f);
      }
    }
  }
  TOCK("raycastKernel", vertex.width() * vertex.height());
}



void renderRGBAKernel(uint8_t*                   output_RGBA,
                      const Eigen::Vector2i&     output_size,
                      const se::Image<uint32_t>& input_RGBA);



void renderDepthKernel(unsigned char*         out,
                       float*                 depth,
                       const Eigen::Vector2i& depth_size,
                       const float            near_plane,
                       const float            far_plane);



void renderTrackKernel(unsigned char*         out,
                       const TrackData*       data,
                       const Eigen::Vector2i& out_size);



template <typename T>
void renderVolumeKernel(const Volume<T>&                  volume,
                        unsigned char*                    out, // RGBW packed
                        const Eigen::Vector2i&            depth_size,
                        const Eigen::Matrix4f&            view,
                        const float                       near_plane,
                        const float                       far_plane,
                        const float                       mu,
                        const float                       step,
                        const float                       large_step,
                        const Eigen::Vector3f&            light,
                        const Eigen::Vector3f&            ambient,
                        bool                              raycast_normals,
                        const se::Image<Eigen::Vector3f>& vertex,
                        const se::Image<Eigen::Vector3f>& normal) {

  TICK();
#pragma omp parallel for
  for (int y = 0; y < depth_size.y(); y++) {
    for (int x = 0; x < depth_size.x(); x++) {
      Eigen::Vector4f hit;
      Eigen::Vector3f test, surface_normal;
      const int idx = (x + depth_size.x()*y) * 4;

      if (raycast_normals) {
        const Eigen::Vector3f dir =
            (view.topLeftCorner<3, 3>() * Eigen::Vector3f(x, y, 1.f)).normalized();
        const Eigen::Vector3f transl = view.topRightCorner<3, 1>();
        se::VoxelBlockRayIterator<typename T::VoxelType> ray(*volume._map_index, transl, dir, near_plane, far_plane);
        ray.next();
        const float t_min = ray.tmin(); /* Get distance to the first intersected block */
        hit = t_min > 0.f
            ? T::raycast(volume, transl, dir, t_min, ray.tmax(), mu, step, large_step)
            : Eigen::Vector4f::Zero();
        if (hit.w() >= 0.f) {
          test = hit.head<3>();
          surface_normal = volume.grad(test, [](const auto& val){ return val.x; });

          // Invert normals for TSDF representations.
          surface_normal = T::invert_normals ? -1.f * surface_normal : surface_normal;
        } else {
          surface_normal = Eigen::Vector3f(INVALID, 0.f, 0.f);
        }
      } else {
        test = vertex[x + depth_size.x() * y];
        surface_normal = normal[x + depth_size.x() * y];
      }

      if (surface_normal.x() != INVALID && surface_normal.norm() > 0.f) {
        const Eigen::Vector3f diff = (test - light).normalized();
        const Eigen::Vector3f dir
            = Eigen::Vector3f::Constant(fmaxf(surface_normal.normalized().dot(diff), 0.f));
        Eigen::Vector3f col = dir + ambient;
        se::math::clamp(col, Eigen::Vector3f::Zero(), Eigen::Vector3f::Ones());
        col = col.cwiseProduct(se::internal::color_map[se::internal::scale_image(x, y)]);
        out[idx + 0] = col.x();
        out[idx + 1] = col.y();
        out[idx + 2] = col.z();
        out[idx + 3] = 255;
      } else {
        out[idx + 0] = 0;
        out[idx + 1] = 0;
        out[idx + 2] = 0;
        out[idx + 3] = 255;
      }
    }
  }
  TOCK("renderVolumeKernel", depth_size.x() * depth_size.y());
}



inline void printNormals(const se::Image<Eigen::Vector3f>& in,
                         const unsigned int                x_dim,
                         const unsigned int                y_dim,
                         const char*                       filename);



// Find ALL the intersection along a ray till the far_plane.
template <typename T>
void raycast_full(
    const Volume<T>&                                                         volume,
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>& points,
    const Eigen::Vector3f&                                                   origin,
    const Eigen::Vector3f&                                                   direction,
    const float                                                              far_plane,
    const float                                                              step,
    const float                                                              large_step) {

  float t = 0;
  float step_size = large_step;
  float f_t = volume.interp(origin + direction * t, [](const auto& val){ return val.x;}).first;
  t += step;
  float f_tt = 1.f;

  for (; t < far_plane; t += step_size) {
    f_tt = volume.interp(origin + direction * t, [](const auto& val){ return val.x;}).first;
    if (f_tt < 0.f && f_t > 0.f && std::abs(f_tt - f_t) < 0.5f) {     // got it, jump out of inner loop
      const auto data_t  = volume.get(origin + direction * (t - step_size));
      const auto data_tt = volume.get(origin + direction * t);
      if (f_t == 1.0 || f_tt == 1.0 || data_t.y == 0 || data_tt.y == 0 ) {
        f_t = f_tt;
        continue;
      }
      t = t + step_size * f_tt / (f_t - f_tt);
      points.push_back((origin + direction * t).homogeneous());
    }
    if (f_tt < std::abs(0.8f)) {
      // coming closer, reduce step_size
      step_size = step;
    }
    f_t = f_tt;
  }
}

#endif

