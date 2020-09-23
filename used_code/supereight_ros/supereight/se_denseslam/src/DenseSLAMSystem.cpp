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

#include <se/DenseSLAMSystem.h>
#include <se/voxel_block_ray_iterator.hpp>
#include <se/algorithms/meshing.hpp>
#include <se/geometry/octree_collision.hpp>
#include <se/io/vtk-io.h>
#include <se/io/ply_io.hpp>
#include <se/algorithms/balancing.hpp>
#include <se/algorithms/meshing.hpp>
#include <se/functors/for_each.hpp>
#include <se/timings.h>
#include <se/perfstats.h>
#include "se/rendering.hpp"


extern PerfStats Stats;
static bool print_kernel_timing = false;

DenseSLAMSystem::DenseSLAMSystem(const Eigen::Vector2i& inputSize,
                                 const Eigen::Vector3i& volumeResolution,
                                 const Eigen::Vector3f& volumeDimensions,
                                 const Eigen::Vector3f& initPose,
                                 std::vector<int> & pyramid,
                                 const Configuration& config):
      DenseSLAMSystem(inputSize, volumeResolution, volumeDimensions,
          se::math::toMatrix4f(initPose), pyramid, config) { }

DenseSLAMSystem::DenseSLAMSystem(const Eigen::Vector2i& inputSize,
                                 const Eigen::Vector3i& volumeResolution,
                                 const Eigen::Vector3f& volumeDimensions,
                                 const Eigen::Matrix4f& init_T_WC,
                                 std::vector<int> & pyramid,
                                 const Configuration& config) :
  computation_size_(inputSize),
  config_(config),
  vertex_(computation_size_.x(), computation_size_.y()),
  normal_(computation_size_.x(), computation_size_.y()),
  float_depth_(computation_size_.x(), computation_size_.y()),
  rgba_(computation_size_.x(), computation_size_.y())
  {

    this->init_position_M_ = init_T_WC.block<3,1>(0,3);
    this->volume_dimension_ = volumeDimensions;
    this->volume_resolution_ = volumeResolution;
    this->mu_ = config.mu;
    T_WC_ = init_T_WC;
    raycast_T_WC_ = init_T_WC;

    this->iterations_.clear();
    for (std::vector<int>::iterator it = pyramid.begin();
        it != pyramid.end(); it++) {
      this->iterations_.push_back(*it);
    }

    render_T_WC_ = &T_WC_;

    if (getenv("KERNEL_TIMINGS"))
      print_kernel_timing = true;

    // internal buffers to initialize
    reduction_output_.resize(8 * 32);
    tracking_result_.resize(computation_size_.x() * computation_size_.y());

    for (unsigned int i = 0; i < iterations_.size(); ++i) {
      int downsample = 1 << i;
      scaled_depth_.push_back(se::Image<float>(computation_size_.x() / downsample,
            computation_size_.y() / downsample));

      input_vertex_.push_back(se::Image<Eigen::Vector3f>(computation_size_.x() / downsample,
            computation_size_.y() / downsample));

      input_normal_.push_back(se::Image<Eigen::Vector3f>(computation_size_.x() / downsample,
            computation_size_.y() / downsample));
    }

    // ********* BEGIN : Generate the gaussian *************
    size_t gaussianS = radius * 2 + 1;
    gaussian_.reserve(gaussianS);
    int x;
    for (unsigned int i = 0; i < gaussianS; i++) {
      x = i - 2;
      gaussian_[i] = expf(-(x * x) / (2 * delta * delta));
    }

    // ********* END : Generate the gaussian *************

    discrete_vol_ptr_ = std::make_shared<se::Octree<VoxelImpl::VoxelType> >();
    discrete_vol_ptr_->init(volume_resolution_.x(), volume_dimension_.x());
    volume_ = Volume<VoxelImpl>(volume_resolution_.x(), volume_dimension_.x(),
        discrete_vol_ptr_.get());
}



bool DenseSLAMSystem::preprocessDepth(const uint16_t*        input_depth,
                                      const Eigen::Vector2i& input_size,
                                      const bool             filter_depth){

  mm2metersKernel(float_depth_, input_depth, input_size);

  if (filter_depth) {
    bilateralFilterKernel(scaled_depth_[0], float_depth_, gaussian_,
        e_delta, radius);
    
  } else {
    std::memcpy(scaled_depth_[0].data(), float_depth_.data(),
        sizeof(float) * computation_size_.x() * computation_size_.y());
  }
  return true;
}



bool DenseSLAMSystem::preprocessColor(const uint8_t*         input_RGB,
                                      const Eigen::Vector2i& input_size) {

  downsampleImageKernel(input_RGB, input_size, rgba_);

  return true;
}



bool DenseSLAMSystem::track(const Eigen::Vector4f& k,
                            float                  icp_threshold) {

  //
  std::cout<<"In tracking process" <<std::endl;

  // half sample the input depth maps into the pyramid levels
  for (unsigned int i = 1; i < iterations_.size(); ++i) {
    halfSampleRobustImageKernel(scaled_depth_[i], scaled_depth_[i - 1], e_delta * 3, 1);
  }

  // prepare the 3D information from the input depth maps
  Eigen::Vector2i localimagesize = computation_size_;
  for (unsigned int i = 0; i < iterations_.size(); ++i) {
    Eigen::Matrix4f invK = getInverseCameraMatrix(k / float(1 << i));
    depth2vertexKernel(input_vertex_[i], scaled_depth_[i], invK);
    if(k.y() < 0)
      vertex2normalKernel<true>(input_normal_[i], input_vertex_[i]);
    else
      vertex2normalKernel<false>(input_normal_[i], input_vertex_[i]);
    localimagesize /= 2;;
  }

  previous_T_WC_ = T_WC_;
  const Eigen::Matrix4f projectReference = getCameraMatrix(k) * raycast_T_WC_.inverse();

  for (int level = iterations_.size() - 1; level >= 0; --level) {
    Eigen::Vector2i localimagesize(
        computation_size_.x() / (int) pow(2, level),
        computation_size_.y() / (int) pow(2, level));
    for (int i = 0; i < iterations_[level]; ++i) {

      trackKernel(tracking_result_.data(), input_vertex_[level], input_normal_[level],
          vertex_, normal_, T_WC_, projectReference,
          dist_threshold, normal_threshold);

      reduceKernel(reduction_output_.data(), tracking_result_.data(), computation_size_,
          localimagesize);

      if (updatePoseKernel(T_WC_, reduction_output_.data(), icp_threshold))
        break;

    }
  }
  return checkPoseKernel(T_WC_, previous_T_WC_, reduction_output_.data(),
      computation_size_, track_threshold);
}



bool DenseSLAMSystem::integrate(const Eigen::Vector4f& k,
                                float                  mu,
                                unsigned int           frame) {

  const float voxel_size = volume_._extent / volume_._size;
  const int num_vox_per_pix = volume_._extent
    / ((se::VoxelBlock<VoxelImpl::VoxelType>::side) * voxel_size);

  std::cout << "volume._extent:" << volume_._extent
            << "\n volume._size:" << volume_._size
            << "\n side:" << (se::VoxelBlock<VoxelImpl::VoxelType>::side)
            << "\n num_vox_per_pix:" << num_vox_per_pix
            << "\n computation_size:" << computation_size_.x() << ',' << computation_size_.y()
            <<std::endl;
    
  const size_t total = num_vox_per_pix
    * computation_size_.x() * computation_size_.y();
  allocation_list_.reserve(total);

  const Sophus::SE3f& T_CW = Sophus::SE3f(T_WC_).inverse();
  const Eigen::Matrix4f& K = getCameraMatrix(k);
  
  const size_t allocated = VoxelImpl::buildAllocationList(
      allocation_list_.data(),
      allocation_list_.capacity(),
      *volume_._map_index,
      T_WC_,
      getCameraMatrix(k),
      float_depth_.data(),
      computation_size_,
      mu);

  volume_._map_index->allocate(allocation_list_.data(), allocated);

  VoxelImpl::integrate(
      *volume_._map_index,
      T_CW,
      K,
      float_depth_,
      mu,
      frame);
  return true;
}



bool DenseSLAMSystem::raycast(const Eigen::Vector4f& k,
                              float                  mu) {

  raycast_T_WC_ = T_WC_;
  float step = volume_dimension_.x() / volume_resolution_.x();
  raycastKernel(volume_, vertex_, normal_,
      raycast_T_WC_ * getInverseCameraMatrix(k), nearPlane,
      farPlane, mu, step, step*BLOCK_SIDE);

  return true;
}



void DenseSLAMSystem::dump_volume(std::string ) {

}

void DenseSLAMSystem::renderVolume(unsigned char*         output,
                                   const Eigen::Vector2i& output_size,
                                   const Eigen::Vector4f& k,
                                   const float            large_step) {

  const float step = volume_dimension_.x() / volume_resolution_.x();
  std::cout << "\nvolume_dimension:" << volume_dimension_.x()
            << "\nvolume_resolution:" << volume_resolution_.x()
            << "\nstep:" << step
            << std::endl;
  renderVolumeKernel(volume_, output, output_size,
      *(this->render_T_WC_) * getInverseCameraMatrix(k), nearPlane,
      farPlane * 2.0f, mu_, step, large_step,
      this->render_T_WC_->topRightCorner<3, 1>(), ambient,
      !(this->render_T_WC_->isApprox(raycast_T_WC_)), vertex_,
      normal_);
}

void DenseSLAMSystem::renderTrack(unsigned char* out,
    const Eigen::Vector2i& outputSize) {
        renderTrackKernel(out, tracking_result_.data(), outputSize);
}

void DenseSLAMSystem::renderDepth(unsigned char* out,
    const Eigen::Vector2i& outputSize) {
        renderDepthKernel(out, float_depth_.data(), outputSize, nearPlane, farPlane);
}



void DenseSLAMSystem::renderRGBA(uint8_t*               output_RGBA,
                                 const Eigen::Vector2i& output_size) {

  renderRGBAKernel(output_RGBA, output_size, rgba_);
}



void DenseSLAMSystem::dump_mesh(const std::string filename){

  se::functor::internal::parallel_for_each(volume_._map_index->getBlockBuffer(),
      [](auto block) {
        if(std::is_same<VoxelImpl, MultiresTSDF>::value) {
          block->current_scale(block->min_scale());
        } else {
          block->current_scale(0);
        }
      });

  auto interp_down = [this](auto block) {
    if(block->min_scale() == 0) return;
    const Eigen::Vector3f& offset = this->volume_._map_index->_offset;
    const Eigen::Vector3i base = block->coordinates();
    const int side = block->side;
    for(int z = 0; z < side; ++z)
      for(int y = 0; y < side; ++y)
        for(int x = 0; x < side; ++x) {
          const Eigen::Vector3i vox = base + Eigen::Vector3i(x, y , z);
          auto curr = block->data(vox, 0);
          auto res = this->volume_._map_index->interp_checked(
              vox.cast<float>() + offset, 0, [](const auto& val) { return val.x; });
          if(res.second >= 0) {
            curr.x = res.first;
            curr.y = this->volume_._map_index->interp(
                vox.cast<float>() + offset, [](const auto& val) { return val.y; }).first;
          } else {
            curr.y = 0;
          }
          block->data(vox, 0, curr);
        }
  };

  se::functor::internal::parallel_for_each(volume_._map_index->getBlockBuffer(),
      interp_down);
  se::functor::internal::parallel_for_each(volume_._map_index->getBlockBuffer(),
      [](auto block) {
          block->current_scale(0);
      });

    std::cout << "saving triangle mesh to file :" << filename  << std::endl;

    std::vector<Triangle> mesh;
    auto inside = [](const VoxelImpl::VoxelType::VoxelData& val) {
      return val.x < 0.f;
    };

    auto select = [](const VoxelImpl::VoxelType::VoxelData& val) {
      return val.x;
    };


    se::algorithms::marching_cube(*volume_._map_index, select, inside, mesh);

    //writeVtkMesh(filename.c_str(), mesh, this->init_position_M_);
    writeObjMesh(filename.c_str(), mesh);
    se::meshing::savePointCloudPly(mesh,"/home/wei/super8mapping/test2.ply", this->init_position_M_ );
    //se::print_octree("/home/wei/super8mapping/Octree.ply", *volume_._map_index);
    std::cout << "Finish writeObjMesh"  << std::endl;
}
