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

// class header
#include <gua/renderer/GraphRessource.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/utils/Logger.hpp>

namespace 
{

struct Vertex
{
  scm::math::vec3f pos;
  scm::math::vec3f normal;
};

}

namespace gua
{

GraphRessource::GraphRessource()
{}

void GraphRessource::upload_to(RenderContext const& ctx) const
{
	std::unique_lock<std::mutex> lock(upload_mutex_);

  if(vertices_.size() <= ctx.id)
	{
    vertices_.resize(ctx.id + 1);
    indices_.resize(ctx.id + 1);
    vertex_array_.resize(ctx.id + 1);
  }
}

void GraphRessource::draw(RenderContext const& ctx) const {

  // upload to GPU if neccessary
  if (vertices_.size() <= ctx.id || vertices_[ctx.id] == nullptr) {
    upload_to(ctx);
  }

  scm::gl::context_vertex_input_guard vig(ctx.render_context);

  ctx.render_context->bind_vertex_array(vertex_array_[ctx.id]);

  ctx.render_context->bind_index_buffer(
      indices_[ctx.id], scm::gl::PRIMITIVE_TRIANGLE_LIST, scm::gl::TYPE_UINT);

  ctx.render_context->apply();
  //ctx.render_context->draw_elements(mesh_->mNumFaces * 3);
}

void GraphRessource::ray_test(Ray const& ray,
															PickResult::Options options,
                    					Node* owner, std::set<PickResult>& hits)
{}

std::shared_ptr<GeometryUberShader> GraphRessource::create_ubershader() const
{
  return std::make_shared<GraphUberShader>();
}

}

