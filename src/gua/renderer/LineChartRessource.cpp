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
#include <gua/renderer/LineChartRessource.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/utils/Logger.hpp>

namespace {
struct Vertex {
  scm::math::vec3f pos;
  scm::math::vec2f tex;
  scm::math::vec3f normal;
  scm::math::vec3f tangent;
  scm::math::vec3f bitangent;
};
}

namespace gua {

////////////////////////////////////////////////////////////////////////////////

LineChartRessource::LineChartRessource(
      std::vector<float> const& xdata
    , std::vector<float> const& ydata
  )
  : vertices_()
  , indices_()
  , vertex_array_()
  , upload_mutex_()
  , num_indices_(0)
  , num_points_(0)
  , thickness(0.05f)
  , xdata_(xdata)
  , ydata_(ydata) {

  num_points_ = std::min(xdata_.size(), ydata_.size());

  // each line segment has 4 faces of 2 triangles plus two faces at the sides
  num_indices_ = (num_points_ - 1) * 4 * 2 * 3 + 2 * 2 * 3;
}

////////////////////////////////////////////////////////////////////////////////

void LineChartRessource::upload_to(RenderContext const& ctx) const
{
  rasterizer_state_ = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_BACK);

  std::unique_lock<std::mutex> lock(upload_mutex_);

  if (vertices_.size() <= ctx.id) {
    vertices_.resize(ctx.id + 1);
    indices_.resize(ctx.id + 1);
    vertex_array_.resize(ctx.id + 1);
  }

  vertices_[ctx.id] =
      ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       ((num_points_ - 1) * 16 + 8) * sizeof(Vertex),
                                       0);

  // set point vertices
  Vertex* data(static_cast<Vertex*>(ctx.render_context->map_buffer(
      vertices_[ctx.id], scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));

  // each line segment has 4 vertices per face with 4 faces, so 16 vertices
  unsigned int vertices_per_segment = 4 * 4;

  // left face of first line segment
  data[0].pos = scm::math::vec3f(xdata_[0] - 0.5f, ydata_[0] - 0.5f - thickness, -thickness);
  data[0].normal = scm::math::vec3f(-1.0f, 0.0f, 0.0f);
  data[1].pos = scm::math::vec3f(xdata_[0] - 0.5f, ydata_[0] - 0.5f - thickness, thickness);
  data[1].normal = scm::math::vec3f(-1.0f, 0.0f, 0.0f);
  data[2].pos = scm::math::vec3f(xdata_[0] - 0.5f, ydata_[0] - 0.5f + thickness, thickness);
  data[2].normal = scm::math::vec3f(-1.0f, 0.0f, 0.0f);
  data[3].pos = scm::math::vec3f(xdata_[0] - 0.5f, ydata_[0] - 0.5f + thickness, -thickness);
  data[3].normal = scm::math::vec3f(-1.0f, 0.0f, 0.0f);

  // right face of last line segment
  data[4].pos = scm::math::vec3f(xdata_[num_points_ - 1] - 0.5f, ydata_[num_points_ - 1] - 0.5f - thickness, thickness);
  data[4].normal = scm::math::vec3f(1.0f, 0.0f, 0.0f);
  data[5].pos = scm::math::vec3f(xdata_[num_points_ - 1] - 0.5f, ydata_[num_points_ - 1] - 0.5f - thickness, -thickness);
  data[5].normal = scm::math::vec3f(1.0f, 0.0f, 0.0f);
  data[6].pos = scm::math::vec3f(xdata_[num_points_ - 1] - 0.5f, ydata_[num_points_ - 1] - 0.5f + thickness, -thickness);
  data[6].normal = scm::math::vec3f(1.0f, 0.0f, 0.0f);
  data[7].pos = scm::math::vec3f(xdata_[num_points_ - 1] - 0.5f, ydata_[num_points_ - 1] - 0.5f + thickness, thickness);
  data[7].normal = scm::math::vec3f(1.0f, 0.0f, 0.0f);

  // create vertices for every line segment
  for (unsigned int i(0); i < num_points_ - 1; ++i)
  {
    // front face
    data[8 + i * vertices_per_segment + 0].pos = scm::math::vec3f(xdata_[i] - 0.5f, ydata_[i] - 0.5f - thickness, thickness);
    data[8 + i * vertices_per_segment + 0].normal = scm::math::vec3f(0, 0, 1);
    data[8 + i * vertices_per_segment + 1].pos = scm::math::vec3f(xdata_[i + 1] - 0.5f, ydata_[i + 1] - 0.5f - thickness, thickness);
    data[8 + i * vertices_per_segment + 1].normal = scm::math::vec3f(0, 0, 1);
    data[8 + i * vertices_per_segment + 2].pos = scm::math::vec3f(xdata_[i + 1] - 0.5f, ydata_[i + 1] - 0.5f + thickness, thickness);
    data[8 + i * vertices_per_segment + 2].normal = scm::math::vec3f(0, 0, 1);
    data[8 + i * vertices_per_segment + 3].pos = scm::math::vec3f(xdata_[i] - 0.5f, ydata_[i] - 0.5f + thickness, thickness);
    data[8 + i * vertices_per_segment + 3].normal = scm::math::vec3f(0, 0, 1);

    // back face
    data[8 + i * vertices_per_segment + 4].pos = scm::math::vec3f(xdata_[i + 1] - 0.5f, ydata_[i + 1] - 0.5f - thickness, -thickness);
    data[8 + i * vertices_per_segment + 4].normal = scm::math::vec3f(0, 0, -1);
    data[8 + i * vertices_per_segment + 5].pos = scm::math::vec3f(xdata_[i] - 0.5f, ydata_[i] - 0.5f - thickness, -thickness);
    data[8 + i * vertices_per_segment + 5].normal = scm::math::vec3f(0, 0, -1);
    data[8 + i * vertices_per_segment + 6].pos = scm::math::vec3f(xdata_[i] - 0.5f, ydata_[i] - 0.5f + thickness, -thickness);
    data[8 + i * vertices_per_segment + 6].normal = scm::math::vec3f(0, 0, -1);
    data[8 + i * vertices_per_segment + 7].pos = scm::math::vec3f(xdata_[i + 1] - 0.5f, ydata_[i + 1] - 0.5f + thickness, -thickness);
    data[8 + i * vertices_per_segment + 7].normal = scm::math::vec3f(0, 0, -1);

    // top face
    data[8 + i * vertices_per_segment + 8].pos = scm::math::vec3f(xdata_[i] - 0.5f, ydata_[i] - 0.5f + thickness, thickness);
    data[8 + i * vertices_per_segment + 8].normal = scm::math::vec3f(0, 1, 0);
    data[8 + i * vertices_per_segment + 9].pos = scm::math::vec3f(xdata_[i + 1] - 0.5f, ydata_[i + 1] - 0.5f + thickness, thickness);
    data[8 + i * vertices_per_segment + 9].normal = scm::math::vec3f(0, 1, 0);
    data[8 + i * vertices_per_segment + 10].pos = scm::math::vec3f(xdata_[i + 1] - 0.5f, ydata_[i + 1] - 0.5f + thickness, -thickness);
    data[8 + i * vertices_per_segment + 10].normal = scm::math::vec3f(0, 1, 0);
    data[8 + i * vertices_per_segment + 11].pos = scm::math::vec3f(xdata_[i] - 0.5f, ydata_[i] - 0.5f + thickness, -thickness);
    data[8 + i * vertices_per_segment + 11].normal = scm::math::vec3f(0, 1, 0);

    // bottom face
    data[8 + i * vertices_per_segment + 12].pos = scm::math::vec3f(xdata_[i] - 0.5f, ydata_[i] - 0.5f - thickness, -thickness);
    data[8 + i * vertices_per_segment + 12].normal = scm::math::vec3f(0, -1, 0);
    data[8 + i * vertices_per_segment + 13].pos = scm::math::vec3f(xdata_[i + 1] - 0.5f, ydata_[i + 1] - 0.5f - thickness, -thickness);
    data[8 + i * vertices_per_segment + 13].normal = scm::math::vec3f(0, -1, 0);
    data[8 + i * vertices_per_segment + 14].pos = scm::math::vec3f(xdata_[i + 1] - 0.5f, ydata_[i + 1] - 0.5f - thickness, thickness);
    data[8 + i * vertices_per_segment + 14].normal = scm::math::vec3f(0, -1, 0);
    data[8 + i * vertices_per_segment + 15].pos = scm::math::vec3f(xdata_[i] - 0.5f, ydata_[i] - 0.5f - thickness, thickness);
    data[8 + i * vertices_per_segment + 15].normal = scm::math::vec3f(0, -1, 0);
  }

  ctx.render_context->unmap_buffer(vertices_[ctx.id]);

  // for each line segment 4 faces with each 6 vertices
  std::vector<unsigned> index_array(num_indices_);

  // left face of first line segment
  index_array[0] = 0; index_array[1] = 1; index_array[2] = 3;
  index_array[3] = 1; index_array[4] = 2; index_array[5] = 3;

  // right face of last line segment
  index_array[6] = 4; index_array[7] = 5; index_array[8] = 7;
  index_array[9] = 5; index_array[10] = 6; index_array[11] = 7;

  for (unsigned int i(0); i < num_points_ - 1; ++i)
  {
    for (unsigned f(0); f < 4; ++f) // for each of the line segments 4 faces
    {
      // lower left triangle
      index_array[12 + i * 24 + f * 6 + 0] = 8 + i * 16 + f * 4 + 0;
      index_array[12 + i * 24 + f * 6 + 1] = 8 + i * 16 + f * 4 + 1;
      index_array[12 + i * 24 + f * 6 + 2] = 8 + i * 16 + f * 4 + 3;

      // upper right triangle
      index_array[12 + i * 24 + f * 6 + 3] = 8 + i * 16 + f * 4 + 1;
      index_array[12 + i * 24 + f * 6 + 4] = 8 + i * 16 + f * 4 + 2;
      index_array[12 + i * 24 + f * 6 + 5] = 8 + i * 16 + f * 4 + 3;
    }
  }

  indices_[ctx.id] =
      ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       num_indices_ * sizeof(unsigned),
                                       &index_array[0]);

  // set point vertex array
  std::vector<scm::gl::buffer_ptr> buffer_arrays;
  buffer_arrays.push_back(vertices_[ctx.id]);

  vertex_array_[ctx.id] = ctx.render_device->create_vertex_array(
      scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
          0, 1, scm::gl::TYPE_VEC2F, sizeof(Vertex))(
          0, 2, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
          0, 3, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
          0, 4, scm::gl::TYPE_VEC3F, sizeof(Vertex)),
      buffer_arrays);
}

////////////////////////////////////////////////////////////////////////////////

void LineChartRessource::draw(RenderContext const& ctx) const {

  // upload to GPU if neccessary
  if (vertices_.size() <= ctx.id || vertices_[ctx.id] == nullptr) {
    upload_to(ctx);
  }

  scm::gl::context_vertex_input_guard vig(ctx.render_context);
  {
    //scm::gl::context_state_objects_guard contex_guard(ctx.render_context);

    // set line and point size
    //ctx.render_context->set_rasterizer_state(rasterizer_state_, 1.0f, 10.0f);

    ctx.render_context->bind_vertex_array(vertex_array_[ctx.id]);
    ctx.render_context->bind_index_buffer(
        indices_[ctx.id], scm::gl::PRIMITIVE_TRIANGLE_LIST, scm::gl::TYPE_UINT);
    ctx.render_context->apply();
    ctx.render_context->draw_elements(num_indices_);
  }
}

////////////////////////////////////////////////////////////////////////////////

void LineChartRessource::ray_test(Ray const& ray, PickResult::Options options,
                    node::Node* owner, std::set<PickResult>& hits) {

  //kd_tree_.ray_test(ray, mesh_, options, owner, hits);
}

////////////////////////////////////////////////////////////////////////////////

/*virtual*/ std::shared_ptr<GeometryUberShader> LineChartRessource::create_ubershader() const {
  return std::make_shared<LineChartUberShader>();
}

}
