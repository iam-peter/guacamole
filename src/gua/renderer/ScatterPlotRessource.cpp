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
#include <gua/renderer/ScatterPlotRessource.hpp>

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

ScatterPlotRessource::ScatterPlotRessource(
      std::shared_ptr<utils::DataColumn> xdata
    , std::shared_ptr<utils::DataColumn> ydata
    , std::shared_ptr<utils::DataColumn> zdata
  )
  : point_vertices_()
  , point_indices_()
  , point_vertex_array_()
  , upload_mutex_()
  , cube_size_(0.01f, 0.01f, 0.0001f)
  , num_indices_(0)
  , num_points_(0)
  , xdata_(xdata)
  , ydata_(ydata)
  , zdata_(zdata) {

  num_points_ = std::min(xdata_->get_num_values(), ydata_->get_num_values());
  num_indices_ = num_points_ * 6 * 2 * 3 ; // cube has 6 faces of 2 triangles

  bounding_box_.expandBy(scm::math::vec3(-0.5f, -0.5f, 0.0f));
  bounding_box_.expandBy(scm::math::vec3(0.5f, 0.5f, 0.0f));

  if (zdata_ != nullptr)  // 3d-scatterplot
  {
    cube_size_[2] = cube_size_[0];
    num_points_ = std::min(zdata_->get_num_values(), num_points_);
    bounding_box_.expandBy(scm::math::vec3(0.0f, 0.0f, 0.5f));
    bounding_box_.expandBy(scm::math::vec3(0.0f, 0.0f, -0.5f));
  }
}

////////////////////////////////////////////////////////////////////////////////

void ScatterPlotRessource::upload_to(RenderContext const& ctx) const
{
  rasterizer_state_ = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_BACK);

  std::unique_lock<std::mutex> lock(upload_mutex_);

  if (point_vertices_.size() <= ctx.id) {
    point_vertices_.resize(ctx.id + 1);
    point_indices_.resize(ctx.id + 1);
    point_vertex_array_.resize(ctx.id + 1);
  }

  bool third_dim = (zdata_ != nullptr);

  unsigned int vertices_per_point = 6 * 4;   // 6 cube faces with each 4 vertices

  point_vertices_[ctx.id] =
      ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       num_points_ * vertices_per_point * sizeof(Vertex),
                                       0);

  // set point vertices
  Vertex* point_data(static_cast<Vertex*>(ctx.render_context->map_buffer(
      point_vertices_[ctx.id], scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));

  for (unsigned p(0); p < num_points_; ++p)
  {
    float x = -0.5f + xdata_->get_norm_values()[p];
    float y = -0.5f + ydata_->get_norm_values()[p];
    float z = 0.0f;
    if (third_dim)
      z = 0.5f - zdata_->get_norm_values()[p];

    // front face
    point_data[p * vertices_per_point + 0].pos = scm::math::vec3(x - cube_size_[0], y - cube_size_[1], z + cube_size_[2]);
    point_data[p * vertices_per_point + 0].normal = scm::math::vec3(0, 0, 1);
    point_data[p * vertices_per_point + 1].pos = scm::math::vec3(x + cube_size_[0], y - cube_size_[1], z + cube_size_[2]);
    point_data[p * vertices_per_point + 1].normal = scm::math::vec3(0, 0, 1);
    point_data[p * vertices_per_point + 2].pos = scm::math::vec3(x + cube_size_[0], y + cube_size_[1], z + cube_size_[2]);
    point_data[p * vertices_per_point + 2].normal = scm::math::vec3(0, 0, 1);
    point_data[p * vertices_per_point + 3].pos = scm::math::vec3(x - cube_size_[0], y + cube_size_[1], z + cube_size_[2]);
    point_data[p * vertices_per_point + 3].normal = scm::math::vec3(0, 0, 1);


    // back face
    point_data[p * vertices_per_point + 4].pos = scm::math::vec3(x + cube_size_[0], y - cube_size_[1], z - cube_size_[2]);
    point_data[p * vertices_per_point + 4].normal = scm::math::vec3(0, 0, -1);
    point_data[p * vertices_per_point + 5].pos = scm::math::vec3(x - cube_size_[0], y - cube_size_[1], z - cube_size_[2]);
    point_data[p * vertices_per_point + 5].normal = scm::math::vec3(0, 0, -1);
    point_data[p * vertices_per_point + 6].pos = scm::math::vec3(x - cube_size_[0], y + cube_size_[1], z - cube_size_[2]);
    point_data[p * vertices_per_point + 6].normal = scm::math::vec3(0, 0, -1);
    point_data[p * vertices_per_point + 7].pos = scm::math::vec3(x + cube_size_[0], y + cube_size_[1], z - cube_size_[2]);
    point_data[p * vertices_per_point + 7].normal = scm::math::vec3(0, 0, -1);
    

    // right face
    point_data[p * vertices_per_point + 8].pos = scm::math::vec3(x + cube_size_[0], y - cube_size_[1], z + cube_size_[2]);
    point_data[p * vertices_per_point + 8].normal = scm::math::vec3(1, 0, 0);
    point_data[p * vertices_per_point + 9].pos = scm::math::vec3(x + cube_size_[0], y - cube_size_[1], z - cube_size_[2]);
    point_data[p * vertices_per_point + 9].normal = scm::math::vec3(1, 0, 0);
    point_data[p * vertices_per_point + 10].pos = scm::math::vec3(x + cube_size_[0], y + cube_size_[1], z - cube_size_[2]);
    point_data[p * vertices_per_point + 10].normal = scm::math::vec3(1, 0, 0);
    point_data[p * vertices_per_point + 11].pos = scm::math::vec3(x + cube_size_[0], y + cube_size_[1], z + cube_size_[2]);
    point_data[p * vertices_per_point + 11].normal = scm::math::vec3(1, 0, 0);
    

    // left face
    point_data[p * vertices_per_point + 12].pos = scm::math::vec3(x - cube_size_[0], y - cube_size_[1], z - cube_size_[2]);
    point_data[p * vertices_per_point + 12].normal = scm::math::vec3(-1, 0, 0);
    point_data[p * vertices_per_point + 13].pos = scm::math::vec3(x - cube_size_[0], y - cube_size_[1], z + cube_size_[2]);
    point_data[p * vertices_per_point + 13].normal = scm::math::vec3(-1, 0, 0);
    point_data[p * vertices_per_point + 14].pos = scm::math::vec3(x - cube_size_[0], y + cube_size_[1], z + cube_size_[2]);
    point_data[p * vertices_per_point + 14].normal = scm::math::vec3(-1, 0, 0);
    point_data[p * vertices_per_point + 15].pos = scm::math::vec3(x - cube_size_[0], y + cube_size_[1], z - cube_size_[2]);
    point_data[p * vertices_per_point + 15].normal = scm::math::vec3(-1, 0, 0);

    // top face
    point_data[p * vertices_per_point + 16].pos = scm::math::vec3(x - cube_size_[0], y + cube_size_[1], z + cube_size_[2]);
    point_data[p * vertices_per_point + 16].normal = scm::math::vec3(0, 1, 0);
    point_data[p * vertices_per_point + 17].pos = scm::math::vec3(x + cube_size_[0], y + cube_size_[1], z + cube_size_[2]);
    point_data[p * vertices_per_point + 17].normal = scm::math::vec3(0, 1, 0);
    point_data[p * vertices_per_point + 18].pos = scm::math::vec3(x + cube_size_[0], y + cube_size_[1], z - cube_size_[2]);
    point_data[p * vertices_per_point + 18].normal = scm::math::vec3(0, 1, 0);
    point_data[p * vertices_per_point + 19].pos = scm::math::vec3(x - cube_size_[0], y + cube_size_[1], z - cube_size_[2]);
    point_data[p * vertices_per_point + 19].normal = scm::math::vec3(0, 1, 0);

    // bottom face
    point_data[p * vertices_per_point + 20].pos = scm::math::vec3(x - cube_size_[0], y - cube_size_[1], z - cube_size_[2]);
    point_data[p * vertices_per_point + 20].normal = scm::math::vec3(0, -1, 0);
    point_data[p * vertices_per_point + 21].pos = scm::math::vec3(x + cube_size_[0], y - cube_size_[1], z - cube_size_[2]);
    point_data[p * vertices_per_point + 21].normal = scm::math::vec3(0, -1, 0);
    point_data[p * vertices_per_point + 22].pos = scm::math::vec3(x + cube_size_[0], y - cube_size_[1], z + cube_size_[2]);
    point_data[p * vertices_per_point + 22].normal = scm::math::vec3(0, -1, 0);
    point_data[p * vertices_per_point + 23].pos = scm::math::vec3(x - cube_size_[0], y - cube_size_[1], z + cube_size_[2]);
    point_data[p * vertices_per_point + 23].normal = scm::math::vec3(0, -1, 0);
  }
  ctx.render_context->unmap_buffer(point_vertices_[ctx.id]);

  // set point indices
  std::vector<unsigned> point_index_array(num_indices_);

  for (unsigned p(0); p < num_points_; ++p)
  {
    for (unsigned f(0); f < 6; ++f) // for each of the cube's 6 faces
    {
      // lower left triangle
      point_index_array[p * 36 + f * 6 + 0] = p * 24 + f * 4 + 0;
      point_index_array[p * 36 + f * 6 + 1] = p * 24 + f * 4 + 1;
      point_index_array[p * 36 + f * 6 + 2] = p * 24 + f * 4 + 3;

      // upper rpght triangle
      point_index_array[p * 36 + f * 6 + 3] = p * 24 + f * 4 + 1;
      point_index_array[p * 36 + f * 6 + 4] = p * 24 + f * 4 + 2;
      point_index_array[p * 36 + f * 6 + 5] = p * 24 + f * 4 + 3;
    }
  }

  point_indices_[ctx.id] =
      ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       num_indices_ * sizeof(unsigned),
                                       &point_index_array[0]);

  // set point vertex array
  std::vector<scm::gl::buffer_ptr> point_buffer_arrays;
  point_buffer_arrays.push_back(point_vertices_[ctx.id]);

  point_vertex_array_[ctx.id] = ctx.render_device->create_vertex_array(
      scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
          0, 1, scm::gl::TYPE_VEC2F, sizeof(Vertex))(
          0, 2, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
          0, 3, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
          0, 4, scm::gl::TYPE_VEC3F, sizeof(Vertex)),
      point_buffer_arrays);
}

////////////////////////////////////////////////////////////////////////////////

void ScatterPlotRessource::draw(RenderContext const& ctx) const {

  // upload to GPU if neccessary
  if (point_vertices_.size() <= ctx.id || point_vertices_[ctx.id] == nullptr) {
    upload_to(ctx);
  }

  scm::gl::context_vertex_input_guard vig(ctx.render_context);
  {
    scm::gl::context_state_objects_guard contex_guard(ctx.render_context);
    
    // set line and point size
    ctx.render_context->set_rasterizer_state(rasterizer_state_, 1.0f, 10.0f);

    // draw points
    ctx.render_context->bind_vertex_array(point_vertex_array_[ctx.id]);
    ctx.render_context->bind_index_buffer(
        point_indices_[ctx.id], scm::gl::PRIMITIVE_TRIANGLE_LIST, scm::gl::TYPE_UINT);
    ctx.render_context->apply();
    ctx.render_context->draw_elements(num_indices_);
  }
}

////////////////////////////////////////////////////////////////////////////////

void ScatterPlotRessource::ray_test(Ray const& ray, PickResult::Options options,
                    node::Node* owner, std::set<PickResult>& hits) {

  //kd_tree_.ray_test(ray, mesh_, options, owner, hits);
}

////////////////////////////////////////////////////////////////////////////////

/*virtual*/ std::shared_ptr<GeometryUberShader> ScatterPlotRessource::create_ubershader() const {
  return std::make_shared<ScatterPlotUberShader>();
}

}
