/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/

#ifndef GUA_VIDEO3D_UBER_SHADER_HPP
#define GUA_VIDEO3D_UBER_SHADER_HPP

#include <array>

// guacamole headers
#include <gua/utils.hpp>
#include <gua/renderer/GeometryUberShader.hpp>

namespace gua {

class ShaderProgram;
class FrameBufferObject;

class GUA_DLL Video3DUberShader : public GeometryUberShader {
 public:

   enum pass {
     warp_pass  = 0,
     blend_pass = 1
   };

  public:

  Video3DUberShader();

  void create(std::set<std::string> const& material_names);

  /*virtual*/ bool upload_to(RenderContext const& context) const;
  
  /*virtual*/ stage_mask const get_stage_mask() const;

  /*virtual*/ void preframe(RenderContext const& context) const;

  /*virtual*/ void predraw(RenderContext const& context,
                           std::string const& ksfile_name,
                           std::string const& material_name,
                           scm::math::mat4 const& model_matrix,
                           scm::math::mat4 const& normal_matrix,
                           Frustum const& frustum,
                           std::size_t viewid) const;

  /*virtual*/ void draw   (RenderContext const& context,
                           std::string const& ksfile_name,
                           std::string const& material_name,
                           scm::math::mat4 const& model_matrix,
                           scm::math::mat4 const& normal_matrix,
                           Frustum const& frustum,
                           std::size_t viewid) const;

  /*virtual*/ void postdraw(RenderContext const& context,
                            std::string const& ksfile_name,
                            std::string const& material_name,
                            scm::math::mat4 const& model_matrix,
                            scm::math::mat4 const& normal_matrix,
                            Frustum const& frustum,
                            std::size_t viewid) const;

  /*virtual*/ void postframe(RenderContext const& context) const;

  std::string const default_video_material_name() const;

 private: // methods

  void              _create_default_material () const;

  std::string const _warp_pass_vertex_shader   () const;
  std::string const _warp_pass_geometry_shader () const;
  std::string const _warp_pass_fragment_shader () const;

  std::string const _tmp_pass_vertex_shader() const;
  std::string const _tmp_pass_fragment_shader() const;

  std::string const _blend_pass_vertex_shader   () const;
  std::string const _blend_pass_fragment_shader () const;
 
  private: // attributes

  static const unsigned                           MAX_NUM_KINECTS = 6;

  mutable std::vector<scm::gl::texture_2d_ptr>	        warp_depth_result_;
  mutable std::vector<scm::gl::texture_2d_ptr>	        warp_color_result_;
  mutable std::vector<scm::gl::frame_buffer_ptr>        warp_result_fbo_;

  mutable std::vector<scm::gl::rasterizer_state_ptr>    no_bfc_rasterizer_state_;
  mutable std::vector<scm::gl::sampler_state_ptr>       nearest_sampler_state_;
  mutable std::vector<scm::gl::sampler_state_ptr>       linear_sampler_state_;
  
  mutable std::vector<scm::gl::depth_stencil_state_ptr> depth_stencil_state_warp_pass_;
  mutable std::vector<scm::gl::depth_stencil_state_ptr> depth_stencil_state_blend_pass_;

  mutable std::vector<scm::gl::quad_geometry_ptr>       fullscreen_quad_;
  mutable std::vector<scm::gl::quad_geometry_ptr>       fullscreen_quad2_;

  
};

}

#endif  // GUA_VIDEO3D_UBER_SHADER_HPP
