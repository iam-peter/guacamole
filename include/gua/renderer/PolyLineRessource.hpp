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

#ifndef GUA_POLY_LINE_RESSOURCE_HPP
#define GUA_POLY_LINE_RESSOURCE_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/GeometryRessource.hpp>
#include <gua/renderer/PolyLineUberShader.hpp>
#include <gua/utils/DataSet.hpp>
#include <gua/utils/KDTree.hpp>

// external headers
#include <scm/gl_core.h>

#include <algorithm>
#include <mutex>
#include <thread>
#include <vector>

namespace gua {

struct RenderContext;

/**
 * Stores geometry data.
 *
 * A mesh can be loaded from an Assimp mesh and the draw onto multiple
 * contexts.
 * Do not use this class directly, it is just used by the Geometry class to
 * store the individual meshes of a file.
 */
class PolyLineRessource : public GeometryRessource {
 public:

  /**
   * Default constructor.
   *
   * Creates a new and empty PolyLine.
   */
  PolyLineRessource(std::vector<math::vec3> const& points, bool connect_end_points = false);

  /**
   * Draws the Mesh.
   *
   * Draws the Mesh to the given context.
   *
   * \param context          The RenderContext to draw onto.
   */
  void draw(RenderContext const& context) const;

  void ray_test(Ray const& ray, PickResult::Options options,
                node::Node* owner, std::set<PickResult>& hits);

  /*virtual*/ std::shared_ptr<GeometryUberShader> create_ubershader() const;

 private:

  void upload_to(RenderContext const& context) const;

  bool connect_end_points_;
  std::vector<math::vec3> points_;

  mutable unsigned int num_indices_;

  mutable std::vector<scm::gl::buffer_ptr>        vertices_;
  mutable std::vector<scm::gl::buffer_ptr>        indices_;
  mutable std::vector<scm::gl::vertex_array_ptr>  vertex_array_;

  mutable std::mutex upload_mutex_;
  mutable scm::gl::rasterizer_state_ptr rasterizer_state_;
};

}

#endif  // GUA_POLY_LINE_RESSOURCE_HPP
