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

ScatterPlotRessource::ScatterPlotRessource()
    : vertices_(), indices_(), vertex_array_(), upload_mutex_() {
  bounding_box_.expandBy(scm::math::vec3(-0.5, -0.5, -0.5));
  bounding_box_.expandBy(scm::math::vec3(0.5, 0.5, 0.5));
}

////////////////////////////////////////////////////////////////////////////////

void ScatterPlotRessource::upload_to(RenderContext const& ctx) const
{

  std::unique_lock<std::mutex> lock(upload_mutex_);

  if (vertices_.size() <= ctx.id) {
    vertices_.resize(ctx.id + 1);
    indices_.resize(ctx.id + 1);
    vertex_array_.resize(ctx.id + 1);
  }

  unsigned int numVertices = 0;
  unsigned int numFaces = 0;

  vertices_[ctx.id] =
      ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       numVertices * sizeof(Vertex),
                                       0);



  Vertex* data(static_cast<Vertex*>(ctx.render_context->map_buffer(
      vertices_[ctx.id], scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));

  // TODO set vertices

  ctx.render_context->unmap_buffer(vertices_[ctx.id]);

  std::vector<unsigned> index_array(numFaces); // TODO init with correct amount of indices

  // TODO set indices

  indices_[ctx.id] =
      ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       numFaces * sizeof(unsigned),
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

  ctx.render_context->bind_vertex_array(vertex_array_[ctx.id]);

  ctx.render_context->bind_index_buffer(
      indices_[ctx.id], scm::gl::PRIMITIVE_TRIANGLE_LIST, scm::gl::TYPE_UINT);

  ctx.render_context->apply();
  ctx.render_context->draw_elements(0);
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
