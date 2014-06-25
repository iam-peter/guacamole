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

namespace gua
{

GraphRessource::GraphRessource() :
face_number_(0)
{}

std::vector<Vertex> const GraphRessource::

generate_sphere_vertices(unsigned int rings,
												 unsigned int sectors,
												 float radius) const
{
  std::vector<Vertex> vertices;

  float R = 1.0f / (rings - 1);
  float S = 1.0f / (sectors - 1);

  for (unsigned int r(0); r < rings; ++r)
    for (unsigned int s(0); s < sectors; ++s)
    {
      Vertex tmp;
      float x = std::cos(2 * M_PI * s * S) * std::sin(M_PI * r * R);
      float y = std::sin(-M_PI_2 + M_PI * r * R);
      float z = std::sin(2 * M_PI * s * S) * std::sin(M_PI * r * R);

      tmp.pos = scm::math::vec3f(x,y,z) * radius;
      tmp.normal = scm::math::vec3f(x,y,z);
      
      vertices.push_back(tmp);
    }

  return vertices;
}

std::vector<unsigned> const GraphRessource::

generate_sphere_indices(unsigned int rings,unsigned int sectors) const
{
  std::vector<unsigned> indices;

  for (unsigned int r(0); r < rings - 1; ++r)
    for (unsigned int s(0); s < sectors - 1; ++s)
    {
      indices.push_back(r * sectors + s);
      indices.push_back(r * sectors + s + 1);
      indices.push_back((r + 1) * sectors + s);
      
      indices.push_back(r * sectors + s + 1);
      indices.push_back((r + 1) * sectors + s + 1);
      indices.push_back((r + 1) * sectors + s);
    }

  return indices;
}

void GraphRessource::upload_to(RenderContext const& ctx) const
{
	std::unique_lock<std::mutex> lock(upload_mutex_);

  if(vertices_.size() <= ctx.id)
	{
    vertices_.resize(ctx.id + 1);
    indices_.resize(ctx.id + 1);
    vertex_array_.resize(ctx.id + 1);
  }


	const unsigned rings = 10 , sectors = 10;

	std::vector<Vertex>   v(generate_sphere_vertices(rings,sectors,5.0));
	std::vector<unsigned> i(generate_sphere_indices(rings,sectors));

	face_number_ = i.size();

  vertices_[ctx.id] =
  
	ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                   scm::gl::USAGE_STATIC_DRAW,
                                   v.size() * sizeof(Vertex),
                                   0);

	Vertex* vdata(static_cast<Vertex*>

	(ctx.render_context->map_buffer(vertices_[ctx.id],
																	scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));


	for(unsigned index = 0 ; index < v.size() ; ++index)
	{
		vdata[index] = v[index];
	}

	ctx.render_context->unmap_buffer(vertices_[ctx.id]);

  indices_[ctx.id] =

  ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                                   scm::gl::USAGE_STATIC_DRAW,
                                   face_number_ * sizeof(unsigned),
                                   &i[0]);

	std::vector<scm::gl::buffer_ptr> buffer_arrays;

  buffer_arrays.push_back(vertices_[ctx.id]);

  vertex_array_[ctx.id] = ctx.render_device->create_vertex_array(

  scm::gl::vertex_format(0,0,scm::gl::TYPE_VEC3F,sizeof(Vertex))
												(0,1,scm::gl::TYPE_VEC3F,sizeof(Vertex)),
      									buffer_arrays);
}

void GraphRessource::draw(RenderContext const& ctx) const
{
  if (vertices_.size() <= ctx.id || vertices_[ctx.id] == nullptr)
	{
    upload_to(ctx);
  }

  scm::gl::context_vertex_input_guard vig(ctx.render_context);

  ctx.render_context->bind_vertex_array(vertex_array_[ctx.id]);

  ctx.render_context->bind_index_buffer(indices_[ctx.id],
																				scm::gl::PRIMITIVE_TRIANGLE_LIST,
																				scm::gl::TYPE_UINT);

  ctx.render_context->apply();
  ctx.render_context->draw_elements(face_number_ / 3);
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

