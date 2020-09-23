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
#ifndef VOLUME_TEMPLATE_H
#define VOLUME_TEMPLATE_H

#include <iostream>
#include <memory>
#include <se/utils/memory_pool.hpp>
#include <se/octree.hpp>
#include <type_traits>
#include <cstring>
#include <Eigen/Dense>



/**
 * Continuous volume abstraction
 * Sparse, dynamically allocated storage accessed through the
 * appropriate indexer (octree/hash table).
 * */
template <typename VoxelImpl, template<typename> class DiscreteMapT>
class VolumeTemplate {

  public:
    typedef typename VoxelImpl::VoxelType VoxelType;
    typedef typename VoxelImpl::VoxelType::VoxelData VoxelData;

    VolumeTemplate(){};
    VolumeTemplate(unsigned int r, float d, DiscreteMapT<VoxelType>* m) :
      _map_index(m) {
        _size = r;
        _extent = d;
      };

    inline Eigen::Vector3f pos(const Eigen::Vector3i& p) const {
      static const float voxelSize = _extent/_size;
      return p.cast<float>() * voxelSize;
    }

    void set(const  Eigen::Vector3f& , const VoxelData& ) {}

    VoxelData operator[](const Eigen::Vector3f& p) const {
      const float inverseVoxelSize = _size/_extent;
      const Eigen::Vector3i scaled_pos = (p * inverseVoxelSize).cast<int>();
      return _map_index->get(scaled_pos.x(), scaled_pos.y(), scaled_pos.z());
    }

    VoxelData get(const Eigen::Vector3f& p, const int scale = 0) const {
      const float inverseVoxelSize = _size/_extent;
      const Eigen::Vector4i scaled_pos = (inverseVoxelSize * p.homogeneous()).cast<int>();
        return _map_index->get_fine(scaled_pos.x(),
                                    scaled_pos.y(),
                                    scaled_pos.z(),
                                    scale);
    }

    VoxelData operator[](const Eigen::Vector3i& p) const {
      return _map_index->get(p.x(), p.y(), p.z());
    }

    template <typename FieldSelector>
    std::pair<float, int> interp(const Eigen::Vector3f& pos, FieldSelector select) const {
      const float inverseVoxelSize = _size / _extent;
      Eigen::Vector3f discrete_pos = inverseVoxelSize * pos;
      return _map_index->interp(discrete_pos, select);
    }

  /*! \brief Interp voxel value at metric position  (x,y,z)
   * \param pos three-dimensional coordinates in which each component belongs
   * to the interval [0, _extent]
   * \param stride distance between neighbouring sampling point, in voxels.
   * Must be >= 1
   * \return signed distance function value at voxel position (x, y, z)
   */
    template <typename FieldSelector>
    std::pair<float, int> interp(const Eigen::Vector3f& pos, const int h, FieldSelector select) const {
      const float inverseVoxelSize = _size / _extent;
      Eigen::Vector3f discrete_pos = (inverseVoxelSize * pos);
      return _map_index->interp(discrete_pos, h, select);
    }

    /*! \brief Compute gradient at metric position  (x,y,z)
     * \param pos three-dimensional coordinates in which each component belongs
     * to the interval [0, _extent]
     * \return signed distance function value at voxel position (x, y, z)
     */
    template <typename FieldSelector>
    Eigen::Vector3f grad(const Eigen::Vector3f& pos, FieldSelector select) const {

      const float inverseVoxelSize = _size / _extent;
      Eigen::Vector3f discrete_pos = inverseVoxelSize * pos;
      return _map_index->grad(discrete_pos, 1.f, select);
    }

    /*! \brief Compute gradient at metric position  (x,y,z)
     * \param pos three-dimensional coordinates in which each component belongs
     * to the interval [0, _extent]
     * \param stride distance between neighbouring sampling point, in voxels.
     * Must be >= 1
     * \return signed distance function value at voxel position (x, y, z)
     */
    template <typename FieldSelector>
    Eigen::Vector3f grad(const Eigen::Vector3f& pos,
        const int h,
        FieldSelector select) const {
      const float inverseVoxelSize = _size / _extent;
      Eigen::Vector3f discrete_pos = inverseVoxelSize * pos;
      return _map_index->grad(discrete_pos, h, select);
    }

    unsigned int _size;
    float _extent;
    std::vector<se::key_t> _allocationList;
    DiscreteMapT<VoxelType> * _map_index;

  private:

    inline Eigen::Vector3i pos(const Eigen::Vector3f& p) const {
      static const float inverseVoxelSize = _size/_extent;
      return (inverseVoxelSize * p).cast<int>();
    }
};
#endif
