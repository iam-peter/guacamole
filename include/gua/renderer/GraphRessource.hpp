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

#ifndef GUA_GRAPH_RESSOURCE_HPP
#define GUA_GRAPH_RESSOURCE_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/GeometryRessource.hpp>
#include <gua/renderer/GraphUberShader.hpp>

// external headers
#include <scm/gl_core.h>

#include <mutex>
#include <thread>

#include <vector>

namespace gua
{

struct RenderContext;

class GraphRessource : public GeometryRessource
{
	public:

	GraphRessource();

  void draw(RenderContext const& context) const;

  void ray_test(Ray const& ray,
								PickResult::Options options,
                Node* owner, 
								std::set<PickResult>& hits);

  std::shared_ptr<GeometryUberShader> create_ubershader() const;

	private:

  void upload_to(RenderContext const& context) const;

  mutable std::vector<scm::gl::buffer_ptr> vertices_;
  mutable std::vector<scm::gl::buffer_ptr> indices_;
  mutable std::vector<scm::gl::vertex_array_ptr> vertex_array_;
  mutable std::mutex upload_mutex_;
};

}

#endif

