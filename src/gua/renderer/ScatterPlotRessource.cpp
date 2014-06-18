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
  : vertices_()
  , indices_()
  , vertex_array_()
  , upload_mutex_()
  , num_vertices_(0)
  , xdata_(xdata)
  , ydata_(ydata)
  , zdata_(zdata) {

  num_vertices_ = std::min(xdata_->get_num_values(), ydata_->get_num_values());

  scm::math::vec3 min, max;

  min[0] = *std::min_element(xdata_->get_norm_values().begin(), xdata_->get_norm_values().end());
  max[0] = *std::max_element(xdata_->get_norm_values().begin(), xdata_->get_norm_values().end());

  min[1] = *std::min_element(ydata_->get_norm_values().begin(), ydata_->get_norm_values().end());
  max[1] = *std::max_element(ydata_->get_norm_values().begin(), ydata_->get_norm_values().end());

  if (zdata_ != nullptr)
  {
    num_vertices_ = std::min(zdata_->get_num_values(), num_vertices_);
    min[2] = *std::min_element(zdata_->get_norm_values().begin(), zdata_->get_norm_values().end());
    max[2] = *std::max_element(zdata_->get_norm_values().begin(), zdata_->get_norm_values().end());
  }

  bounding_box_.expandBy(min);
  bounding_box_.expandBy(max); 
}

////////////////////////////////////////////////////////////////////////////////

void ScatterPlotRessource::upload_to(RenderContext const& ctx) const
{
  rasterizer_state_ = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_FRONT);

  std::unique_lock<std::mutex> lock(upload_mutex_);

  if (vertices_.size() <= ctx.id) {
    vertices_.resize(ctx.id + 1);
    indices_.resize(ctx.id + 1);
    vertex_array_.resize(ctx.id + 1);
  }

  bool third_dim = (zdata_ != nullptr);

  vertices_[ctx.id] =
      ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       num_vertices_ * sizeof(Vertex),
                                       0);

  Vertex* data(static_cast<Vertex*>(ctx.render_context->map_buffer(
      vertices_[ctx.id], scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));

  for (unsigned v(0); v < num_vertices_; ++v)
  {
    float x = -0.5f + xdata_->get_norm_values()[v];
    float y = -0.5f + ydata_->get_norm_values()[v];
    float z = 0.0f;
    if (third_dim)
      z = 0.5f - zdata_->get_norm_values()[v];
    data[v].pos = scm::math::vec3(x, y, z);

    // set default values for the other vertex attribs
    data[v].tex = scm::math::vec2(0.f, 0.f);
    data[v].normal = scm::math::vec3(0.f, 0.f, 0.f);
    data[v].tangent = scm::math::vec3(0.f, 0.f, 0.f);
    data[v].bitangent = scm::math::vec3(0.f, 0.f, 0.f);
  }

  ctx.render_context->unmap_buffer(vertices_[ctx.id]);

  std::vector<unsigned> index_array(num_vertices_);

  // TODO set indices
  for (unsigned i(0); i < num_vertices_; ++i)
    index_array[i] = i;

  indices_[ctx.id] =
      ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       num_vertices_ * sizeof(unsigned),
                                       &index_array[0]);

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

void ScatterPlotRessource::draw(RenderContext const& ctx) const {

  // upload to GPU if neccessary
  if (vertices_.size() <= ctx.id || vertices_[ctx.id] == nullptr) {
    upload_to(ctx);
  }

  scm::gl::context_vertex_input_guard vig(ctx.render_context);
  {
    ctx.render_context->set_rasterizer_state(rasterizer_state_, 1.0f, 10.0f);

    ctx.render_context->bind_vertex_array(vertex_array_[ctx.id]);

    ctx.render_context->bind_index_buffer(
        indices_[ctx.id], scm::gl::PRIMITIVE_POINT_LIST, scm::gl::TYPE_UINT);

    ctx.render_context->apply();
    ctx.render_context->draw_elements(num_vertices_);
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
