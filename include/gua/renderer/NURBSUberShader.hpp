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

#ifndef GUA_NURBS_UBER_SHADER_HPP
#define GUA_NURBS_UBER_SHADER_HPP

// guacamole headers
#include <gua/renderer/GeometryUberShader.hpp>

namespace gua {

/**
 *
 */
class NURBSUberShader : public GeometryUberShader {

  public : // typedefs, enums

   enum pass {
     transform_feedback_pass = 0,
     final_pass = 1
   };

  public :

  /*virtual*/ void create   (std::set<std::string> const& material_names);

  /*virtual*/ stage_mask get_stage_mask() const override;

  /*virtual*/ void preframe (RenderContext const& ctx) const;

  /*virtual*/ void predraw  (RenderContext const& ctx,
                             std::string const& ksfile_name,
                             std::string const& material_name,
                             scm::math::mat4 const& model_matrix,
                             scm::math::mat4 const& normal_matrix,
                             Frustum const& /*frustum*/,
                             View const& view) const;

  /*virtual*/ void draw     (RenderContext const& ctx,
                             std::string const& ksfile_name,
                             std::string const& material_name,
                             scm::math::mat4 const& model_matrix,
                             scm::math::mat4 const& normal_matrix,
                             Frustum const& /*frustum*/,
                             View const& view) const;

  /*virtual*/ void postdraw (RenderContext const& ctx,
                             std::string const& ksfile_name,
                             std::string const& material_name,
                             scm::math::mat4 const& model_matrix,
                             scm::math::mat4 const& normal_matrix,
                             Frustum const& /*frustum*/,
                             View const& view) const;

  /*virtual*/ void postframe(RenderContext const& ctx) const;

 private:  // auxiliary methods

  std::string _transform_feedback_vertex_shader() const;
  std::string _transform_feedback_geometry_shader() const;
  std::string _transform_feedback_tess_control_shader() const;
  std::string _transform_feedback_tess_evaluation_shader() const;

  std::string _final_vertex_shader() const;
  std::string _final_tess_control_shader() const;
  std::string _final_tess_evaluation_shader() const;
  std::string _final_geometry_shader() const;
  std::string _final_fragment_shader() const;

 private:  // attributes

};

}

#endif  // GUA_NURBS_UBER_SHADER_HPP
