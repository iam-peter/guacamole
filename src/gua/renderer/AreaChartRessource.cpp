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
#include <gua/renderer/AreaChartRessource.hpp>

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

AreaChartRessource::AreaChartRessource(
      std::vector<float> const& xdata
    , std::vector<float> const& ydata
  )
  : vertices_()
  , indices_()
  , vertex_array_()
  , upload_mutex_()
  , num_indices_(0)
  , num_points_(0)
  , xdata_()
  , ydata_() {

  num_points_ = std::min(xdata.size(), ydata.size());

  // add first value pair
  xdata_.push_back(xdata[0]);
  ydata_.push_back(ydata[0]);

  // for all following pairs: check if there was a zero-crossing
  // and eventually insert new points at the zero-crossings
  for (unsigned int i(1); i < num_points_; ++i)
  {
    if ( (ydata[i - 1] < 0.0f) != (ydata[i] < 0.0f) )
    {
      float slope = (ydata[i] - ydata[i - 1]) / (xdata[i] - xdata[i - 1]);
      float delta_x = -ydata[i - 1] / slope;
      xdata_.push_back(xdata[i - 1] + delta_x);
      ydata_.push_back(0.0f);
    }
    xdata_.push_back(xdata[i]);
    ydata_.push_back(ydata[i]);
  }
}

////////////////////////////////////////////////////////////////////////////////

void AreaChartRessource::upload_to(RenderContext const& ctx) const
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
                                       xdata_.size() * 2 * sizeof(Vertex),
                                       0);

  // set point vertices
  Vertex* data(static_cast<Vertex*>(ctx.render_context->map_buffer(
      vertices_[ctx.id], scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));

  // create triangles for every segment
  for (unsigned int i(0); i < xdata_.size(); ++i)
  {
    scm::math::vec3f data_pos = scm::math::vec3f(xdata_[i], ydata_[i], 0.0f);
    scm::math::vec3f anchor_pos = scm::math::vec3f(xdata_[i], 0.0f, 0.0f);
    scm::math::vec3f normal = scm::math::vec3f(0.0f, 0.0f, 1.0f);

    if (ydata_[i] >= 0.0f)
    {
      data[i * 2 + 0].pos = data_pos;
      data[i * 2 + 1].pos = anchor_pos;
    } else
    {
      data[i * 2 + 0].pos = anchor_pos;
      data[i * 2 + 1].pos = data_pos;
    }

    data[i * 2 + 0].normal = normal;
    data[i * 2 + 1].normal = normal;
  }

  ctx.render_context->unmap_buffer(vertices_[ctx.id]);

  // for each line segment 4 faces with each 6 vertices
  std::vector<unsigned> index_array(xdata_.size() * 2);

  for (unsigned int i(0); i < xdata_.size(); ++i)
  {
    index_array[i * 2] = i * 2;
    index_array[i * 2 + 1] = i * 2 + 1;
  }

  indices_[ctx.id] =
      ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       xdata_.size() * 2 * sizeof(unsigned),
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

void AreaChartRessource::draw(RenderContext const& ctx) const {

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
        indices_[ctx.id], scm::gl::PRIMITIVE_TRIANGLE_STRIP, scm::gl::TYPE_UINT);
    ctx.render_context->apply();
    ctx.render_context->draw_elements(xdata_.size() * 2);
  }
}

////////////////////////////////////////////////////////////////////////////////

void AreaChartRessource::ray_test(Ray const& ray, PickResult::Options options,
                    node::Node* owner, std::set<PickResult>& hits) {

  //kd_tree_.ray_test(ray, mesh_, options, owner, hits);
}

////////////////////////////////////////////////////////////////////////////////

/*virtual*/ std::shared_ptr<GeometryUberShader> AreaChartRessource::create_ubershader() const {
  return std::make_shared<AreaChartUberShader>();
}

}