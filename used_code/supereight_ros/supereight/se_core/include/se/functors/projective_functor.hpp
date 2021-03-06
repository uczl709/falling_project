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

#ifndef PROJECTIVE_FUNCTOR_HPP
#define PROJECTIVE_FUNCTOR_HPP
#include <functional>
#include <vector>

#include <sophus/se3.hpp>
#include "../utils/math_utils.h"
#include "../algorithms/filter.hpp"
#include "../node.hpp"
#include "../functors/data_handler.hpp"

namespace se {
namespace functor {
  template <typename FieldType, template <typename FieldT> class MapT,
            typename UpdateF>
  class projective_functor {

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      projective_functor(MapT<FieldType>& map,
                         UpdateF f,
                         const Sophus::SE3f& Tcw,
                         const Eigen::Matrix4f& K,
                         const Eigen::Vector3f& offset,
                         const Eigen::Vector2i& framesize) :
        _map(map), _function(f), _Tcw(Tcw), _K(K), _offset(offset),
        _frame_size(framesize) {
      }

      /*! \brief Get all the blocks that are active or inside the camera
       * frustum. The blocks are stored in projective_functor::_active_list.
       */
      void build_active_list() {
        using namespace std::placeholders;
        /* Retrieve the active list */
        const se::MemoryPool<se::VoxelBlock<FieldType> >& block_array =
          _map.getBlockBuffer();

        /* Predicates definition */
        const Eigen::Matrix4f Tcw = _Tcw.matrix();
        const float voxel_size = _map.dim()/_map.size();

        auto in_frustum_predicate =
          std::bind(algorithms::in_frustum<se::VoxelBlock<FieldType>>,
              std::placeholders::_1, voxel_size, _K*Tcw, _frame_size);

        auto is_active_predicate = [](const se::VoxelBlock<FieldType>* b) {
          return b->active();
        };

        /* Get all the blocks that are active or inside the camera frustum. */
        algorithms::filter(_active_list, block_array, is_active_predicate,
            in_frustum_predicate);
      }

      void update_block(se::VoxelBlock<FieldType> * block,
                        const float voxel_size) {
        /* Is this the VoxelBlock center? */
        const Eigen::Vector3i blockCoord = block->coordinates();
        const Eigen::Vector3f delta = _Tcw.rotationMatrix() * Eigen::Vector3f(voxel_size, 0, 0);
        const Eigen::Vector3f cameraDelta = _K.topLeftCorner<3,3>() * delta;
        bool is_visible = false;

        unsigned int y, z, blockSide;
        blockSide = se::VoxelBlock<FieldType>::side;
        unsigned int ylast = blockCoord(1) + blockSide;
        unsigned int zlast = blockCoord(2) + blockSide;
        block->current_scale(0);

        /* Iterate over each voxel in the VoxelBlock. */
        for(z = blockCoord(2); z < zlast; ++z)
          for (y = blockCoord(1); y < ylast; ++y){
            Eigen::Vector3i pix = Eigen::Vector3i(blockCoord(0), y, z);
            Eigen::Vector3f start = _Tcw * (voxel_size * (pix.cast<float>() + _offset));
            Eigen::Vector3f camerastart = _K.topLeftCorner<3,3>() * start;
#pragma omp simd
            for (unsigned int x = 0; x < blockSide; ++x){
              pix(0) = x + blockCoord(0);
              const Eigen::Vector3f camera_voxel = camerastart + (x*cameraDelta);
              const Eigen::Vector3f pos = start + (x*delta);
              if (pos(2) < 0.0001f) continue;

              const float inverse_depth = 1.f / camera_voxel(2);
              const Eigen::Vector2f pixel = Eigen::Vector2f(
                  camera_voxel(0) * inverse_depth + 0.5f,
                  camera_voxel(1) * inverse_depth + 0.5f);
              /* Skip voxels that are not visible from the camera. */
              if (pixel(0) < 0.5f || pixel(0) > _frame_size(0) - 1.5f ||
                  pixel(1) < 0.5f || pixel(1) > _frame_size(1) - 1.5f) continue;
              is_visible = true;

              /* Update the voxel. */
              VoxelBlockHandler<FieldType> handler = {block, pix};
              _function(handler, pix, pos, pixel);
            }
          }
        block->active(is_visible);
      }

      void update_node(se::Node<FieldType> * node, const float voxel_size) {
        const Eigen::Vector3i voxel = Eigen::Vector3i(unpack_morton(node->code_));
        const Eigen::Vector3f delta = _Tcw.rotationMatrix() *
          Eigen::Vector3f::Constant(0.5f * voxel_size * node->side_);
        const Eigen::Vector3f delta_c = _K.topLeftCorner<3,3>() * delta;
        Eigen::Vector3f base_cam = _Tcw *
          (voxel_size * (voxel.cast<float> () + _offset*node->side_));
        Eigen::Vector3f basepix_hom = _K.topLeftCorner<3,3>() * base_cam;

        /* Iterate over the Node children. */
#pragma omp simd
        for(int i = 0; i < 8; ++i) {
          const Eigen::Vector3i dir =  Eigen::Vector3i((i & 1) > 0, (i & 2) > 0, (i & 4) > 0);
          const Eigen::Vector3f vox_cam = base_cam + dir.cast<float>().cwiseProduct(delta);
          const Eigen::Vector3f pix_hom = basepix_hom + dir.cast<float>().cwiseProduct(delta_c);

          if (vox_cam(2) < 0.0001f) continue;
          const float inverse_depth = 1.f / pix_hom(2);
          const Eigen::Vector2f pixel = Eigen::Vector2f(
              pix_hom(0) * inverse_depth + 0.5f,
              pix_hom(1) * inverse_depth + 0.5f);
          if (pixel(0) < 0.5f || pixel(0) > _frame_size(0) - 1.5f ||
              pixel(1) < 0.5f || pixel(1) > _frame_size(1) - 1.5f) continue;

          /* Update the child Node. */
          NodeHandler<FieldType> handler = {node, i};
          _function(handler, voxel + dir, vox_cam, pixel);
        }
      }

      void apply() {

        const float voxel_size = _map.dim() / _map.size();

        /* Update the leaf Octree nodes (VoxelBlock). */
        // 获得在视锥中的点
        build_active_list();

#pragma omp parallel for
        for (unsigned int i = 0; i < _active_list.size(); ++i) {
          update_block(_active_list[i], voxel_size);
        }
        _active_list.clear();

        /* Update the intermediate Octree nodes (Node). */
        auto& nodes_list = _map.getNodesBuffer();
#pragma omp parallel for
          for (unsigned int i = 0; i < nodes_list.size(); ++i) {
            update_node(nodes_list[i], voxel_size);
         }
      }

    private:
      MapT<FieldType>& _map;
      UpdateF _function;
      Sophus::SE3f _Tcw;
      Eigen::Matrix4f _K;
      Eigen::Vector3f _offset;
      Eigen::Vector2i _frame_size;
      std::vector<se::VoxelBlock<FieldType>*> _active_list;
  };

  /*! \brief Create a projective_functor and call projective_functor::apply.
   */
  template <typename FieldType, template <typename FieldT> class MapT,
            typename UpdateF>
  void projective_map(MapT<FieldType>& map, const Eigen::Vector3f& offset,
      const Sophus::SE3f& Tcw, const Eigen::Matrix4f& K,
      const Eigen::Vector2i& framesize, UpdateF funct) {

    projective_functor<FieldType, MapT, UpdateF>
      it(map, funct, Tcw, K, offset, framesize);
    it.apply();
  }

  /*! \brief Create a projective_functor and call projective_functor::apply.
   */
  template <typename FieldType, template <typename FieldT> class MapT,
            typename UpdateF>
  void projective_map(MapT<FieldType>& map, const Sophus::SE3f& Tcw,
          const Eigen::Matrix4f& K, const Eigen::Vector2i& framesize,
          UpdateF funct) {

    projective_functor<FieldType, MapT, UpdateF>
      it(map, funct, Tcw, K, Eigen::Vector3f::Constant(0.f), framesize);
    it.apply();
  }
}
}
#endif
