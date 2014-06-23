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
  , axes_vertices_()
  , axes_indices_()
  , axes_vertex_array_()
  , upload_mutex_()
  , cube_size_(0.1f, 0.1f, 0.001f)
  , num_point_vertices_(0)
  , num_axes_(2)
  , xdata_(xdata)
  , ydata_(ydata)
  , zdata_(zdata) {

  num_point_vertices_ = std::min(xdata_->get_num_values(), ydata_->get_num_values());

  bounding_box_.expandBy(scm::math::vec3(-0.5f, -0.5f, 0.0f));
  bounding_box_.expandBy(scm::math::vec3(0.5f, 0.5f, 0.0f));

  if (zdata_ != nullptr)  // 3d-scatterplot
  {
    cube_size_[2] = cube_size_[0];
    num_point_vertices_ = std::min(zdata_->get_num_values(), num_point_vertices_);
    num_axes_ = 3;
    bounding_box_.expandBy(scm::math::vec3(0.0f, 0.0f, 0.5f));
    bounding_box_.expandBy(scm::math::vec3(0.0f, 0.0f, -0.5f));
  }
}

////////////////////////////////////////////////////////////////////////////////

void ScatterPlotRessource::upload_to(RenderContext const& ctx) const
{
  rasterizer_state_ = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_FRONT);

  std::unique_lock<std::mutex> lock(upload_mutex_);

  if (point_vertices_.size() <= ctx.id) {
    point_vertices_.resize(ctx.id + 1);
    point_indices_.resize(ctx.id + 1);
    point_vertex_array_.resize(ctx.id + 1);
    axes_vertices_.resize(ctx.id + 1);
    axes_indices_.resize(ctx.id + 1);
    axes_vertex_array_.resize(ctx.id + 1);
  }

  bool third_dim = (zdata_ != nullptr);

  point_vertices_[ctx.id] =
      ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       num_point_vertices_ * sizeof(Vertex),
                                       0);
  axes_vertices_[ctx.id] =
      ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       (num_axes_ + 1) * sizeof(Vertex),
                                       0);

  // set point vertices
  Vertex* point_data(static_cast<Vertex*>(ctx.render_context->map_buffer(
      point_vertices_[ctx.id], scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));
  for (unsigned v(0); v < num_point_vertices_; ++v)
  {
    float x = -0.5f + xdata_->get_norm_values()[v];
    float y = -0.5f + ydata_->get_norm_values()[v];
    float z = 0.0f;
    if (third_dim)
      z = 0.5f - zdata_->get_norm_values()[v];
    point_data[v].pos = scm::math::vec3(x, y, z);

    // set default values for the other vertex attribs
    point_data[v].tex = scm::math::vec2(0.f, 0.f);
    point_data[v].normal = scm::math::vec3(0.f, 0.f, 0.f);
    point_data[v].tangent = scm::math::vec3(0.f, 0.f, 0.f);
    point_data[v].bitangent = scm::math::vec3(0.f, 0.f, 0.f);
  }
  ctx.render_context->unmap_buffer(point_vertices_[ctx.id]);

  // set point indices
  std::vector<unsigned> point_index_array(num_point_vertices_);

  for (unsigned i(0); i < num_point_vertices_; ++i)
    point_index_array[i] = i;

  point_indices_[ctx.id] =
      ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       num_point_vertices_ * sizeof(unsigned),
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

  // set axes vertices
  Vertex* axes_data(static_cast<Vertex*>(ctx.render_context->map_buffer(
      axes_vertices_[ctx.id], scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));
  scm::math::vec2 xrange = scm::math::vec2(-0.5, 0.5f);
  scm::math::vec2 yrange = scm::math::vec2(-0.5, 0.5f);
  scm::math::vec2 zrange = scm::math::vec2(0.0f, 0.0f);
  if (third_dim)
    zrange = scm::math::vec2(0.5, -0.5f);
  axes_data[0].pos = scm::math::vec3(xrange[0], yrange[0], zrange[0]); // origin
  axes_data[1].pos = scm::math::vec3(xrange[1], yrange[0], zrange[0]); // x-axis
  axes_data[2].pos = scm::math::vec3(xrange[0], yrange[1], zrange[0]); // y-axis
  axes_data[3].pos = scm::math::vec3(xrange[0], yrange[0], zrange[1]); // z-axis
  ctx.render_context->unmap_buffer(axes_vertices_[ctx.id]);

  // set axes indices
  std::vector<unsigned> axes_index_array(num_axes_ * 2);

  axes_index_array[0] = 0;
  axes_index_array[1] = 1;  // x-axis
  axes_index_array[2] = 0;
  axes_index_array[3] = 2;  // y-axis
  if (third_dim)
  {
    axes_index_array[4] = 0;
    axes_index_array[5] = 3;  // z-axis
  }

  axes_indices_[ctx.id] =
      ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       num_point_vertices_ * sizeof(unsigned),
                                       &axes_index_array[0]);

  // set axes vertex array
  std::vector<scm::gl::buffer_ptr> axes_buffer_arrays;
  axes_buffer_arrays.push_back(axes_vertices_[ctx.id]);

  axes_vertex_array_[ctx.id] = ctx.render_device->create_vertex_array(
      scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
          0, 1, scm::gl::TYPE_VEC2F, sizeof(Vertex))(
          0, 2, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
          0, 3, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
          0, 4, scm::gl::TYPE_VEC3F, sizeof(Vertex)),
      axes_buffer_arrays);

}

////////////////////////////////////////////////////////////////////////////////

void ScatterPlotRessource::draw(RenderContext const& ctx) const {

  // upload to GPU if neccessary
  if (point_vertices_.size() <= ctx.id || point_vertices_[ctx.id] == nullptr) {
    upload_to(ctx);
  }

  scm::gl::context_vertex_input_guard vig(ctx.render_context);
  {
    // set line and point size
    ctx.render_context->set_rasterizer_state(rasterizer_state_, 1.0f, 10.0f);

    // draw points
    ctx.render_context->bind_vertex_array(point_vertex_array_[ctx.id]);
    ctx.render_context->bind_index_buffer(
        point_indices_[ctx.id], scm::gl::PRIMITIVE_POINT_LIST, scm::gl::TYPE_UINT);
    ctx.render_context->apply();
    ctx.render_context->draw_elements(num_point_vertices_);

    // draw axes
    ctx.render_context->bind_vertex_array(axes_vertex_array_[ctx.id]);
    ctx.render_context->bind_index_buffer(
        axes_indices_[ctx.id], scm::gl::PRIMITIVE_LINE_LIST, scm::gl::TYPE_UINT);
    ctx.render_context->apply();
    ctx.render_context->draw_elements(num_axes_ * 2);
  }
}

////////////////////////////////////////////////////////////////////////////////

void ScatterPlotRessource::ray_test(Ray const& ray, PickResult::Options options,
                    Node* owner, std::set<PickResult>& hits) {

  //kd_tree_.ray_test(ray, mesh_, options, owner, hits);
}

////////////////////////////////////////////////////////////////////////////////

//unsigned int ScatterPlotRessource::num_vertices() const { return mesh_->mNumVertices; }

////////////////////////////////////////////////////////////////////////////////

//unsigned int ScatterPlotRessource::num_faces() const { return mesh_->mNumFaces; }

////////////////////////////////////////////////////////////////////////////////

/*scm::math::vec3 ScatterPlotRessource::get_vertex(unsigned int i) const {

  return scm::math::vec3(
      mesh_->mVertices[i].x, mesh_->mVertices[i].y, mesh_->mVertices[i].z);
}*/

////////////////////////////////////////////////////////////////////////////////

/*std::vector<unsigned int> ScatterPlotRessource::get_face(unsigned int i) const {

  std::vector<unsigned int> face(mesh_->mFaces[i].mNumIndices);
  for (unsigned int j = 0; j < mesh_->mFaces[i].mNumIndices; ++j)
    face[j] = mesh_->mFaces[i].mIndices[j];
  return face;
}*/

////////////////////////////////////////////////////////////////////////////////

/*virtual*/ std::shared_ptr<GeometryUberShader> ScatterPlotRessource::create_ubershader() const {
  return std::make_shared<ScatterPlotUberShader>();
}

}
