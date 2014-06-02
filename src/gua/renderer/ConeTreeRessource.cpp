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
#include <gua/renderer/ConeTreeRessource.hpp>

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

ConeTreeRessource::ConeTreeRessource(CTNode const& root)
    : vertices_(), indices_(), vertex_array_(), upload_mutex_(), cone_tree_root_(root) 
    {
      bounding_box_ = math::BoundingBox<math::vec3>(scm::math::vec3(0,0,0),
                                                  scm::math::vec3(1,1,1));
    }

////////////////////////////////////////////////////////////////////////////////

/*ConeTreeRessource::ConeTreeRessource(aiMesh* mesh, std::shared_ptr<Assimp::Importer> const& importer,
           bool build_kd_tree)
    : vertices_(),
      indices_(),
      vertex_array_(),
      upload_mutex_(),
      mesh_(mesh),
      importer_(importer) {

  if (mesh_->HasPositions()) {
    bounding_box_ = math::BoundingBox<math::vec3>(scm::math::vec3(-1,-1,-1),
                                                  scm::math::vec3(1,1,1));

    if (build_kd_tree) {
      kd_tree_.generate(mesh_);
    }
  }
}*/

////////////////////////////////////////////////////////////////////////////////

void ConeTreeRessource::upload_to(RenderContext const& ctx) const
{


  /*if (!mesh_->HasPositions()) {
    Logger::LOG_WARNING << "Unable to load Mesh! Has no vertex data." << std::endl;
    return;
  }*/


  std::unique_lock<std::mutex> lock(upload_mutex_);

  if (vertices_.size() <= ctx.id) {
    vertices_.resize(ctx.id + 1);
    indices_.resize(ctx.id + 1);
    vertex_array_.resize(ctx.id + 1);
  }


  int numVertice = 8;
  int numFaces = 12;


  vertices_[ctx.id] =  ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                                        scm::gl::USAGE_STATIC_DRAW,
                                                        numVertice * sizeof(Vertex),
                                                        0);



  Vertex* data(static_cast<Vertex*>(ctx.render_context->map_buffer(vertices_[ctx.id], scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));

  // for (unsigned v(0); v < numVertice; ++v)
  // {
    data[0].pos = scm::math::vec3(0.0f, 0.0f, 0.0f);
    data[0].tex = scm::math::vec2(0.f, 0.f);
    data[0].normal = scm::math::vec3(-1.f, -1.f, -1.f);
    data[0].tangent = scm::math::vec3(0.f, 0.f, 0.f);
    data[0].bitangent = scm::math::vec3(0.f, 0.f, 0.f);

    data[1].pos = scm::math::vec3(1.0f, 0.0f, 0.0f);
    data[1].tex = scm::math::vec2(1.f, 0.f);
    data[1].normal = scm::math::vec3(1.f, -1.f, -1.f);
    data[1].tangent = scm::math::vec3(0.f, 0.f, 0.f);
    data[1].bitangent = scm::math::vec3(0.f, 0.f, 0.f);

    data[2].pos = scm::math::vec3(1.0f, 0.0f, 1.0f);
    data[2].tex = scm::math::vec2(0.f, 1.f);
    data[2].normal = scm::math::vec3(1.f, -1.f, 1.f);
    data[2].tangent = scm::math::vec3(0.f, 0.f, 0.f);
    data[2].bitangent = scm::math::vec3(0.f, 0.f, 0.f);

    data[3].pos = scm::math::vec3(0.0f, 0.0f, 1.0f);
    data[3].tex = scm::math::vec2(0.f, 0.f);
    data[3].normal = scm::math::vec3(-1.f, -1.f, 1.f);
    data[3].tangent = scm::math::vec3(0.f, 0.f, 0.f);
    data[3].bitangent = scm::math::vec3(0.f, 0.f, 0.f);

    data[4].pos = scm::math::vec3(0.0f, 1.0f, 0.0f);
    data[4].tex = scm::math::vec2(1.f, 0.f);
    data[4].normal = scm::math::vec3(-1.f, 1.f, -1.f);
    data[4].tangent = scm::math::vec3(0.f, 0.f, 0.f);
    data[4].bitangent = scm::math::vec3(0.f, 0.f, 0.f);

    data[5].pos = scm::math::vec3(1.0f, 1.0f, 0.0f);
    data[5].tex = scm::math::vec2(0.f, 1.f);
    data[5].normal = scm::math::vec3(1.f, 1.f, -1.f);
    data[5].tangent = scm::math::vec3(0.f, 0.f, 0.f);
    data[5].bitangent = scm::math::vec3(0.f, 0.f, 0.f);

    data[6].pos = scm::math::vec3(1.0f, 1.0f, 1.0f);
    data[6].tex = scm::math::vec2(0.f, 1.f);
    data[6].normal = scm::math::vec3(1.f, 1.f, 1.f);
    data[6].tangent = scm::math::vec3(0.f, 0.f, 0.f);
    data[6].bitangent = scm::math::vec3(0.f, 0.f, 0.f);

    data[7].pos = scm::math::vec3(0.0f, 1.0f, 1.0f);
    data[7].tex = scm::math::vec2(0.f, 1.f);
    data[7].normal = scm::math::vec3(-1.f, 1.f, 1.f);
    data[7].tangent = scm::math::vec3(0.f, 0.f, 0.f);
    data[7].bitangent = scm::math::vec3(0.f, 0.f, 0.f);

  // }

  ctx.render_context->unmap_buffer(vertices_[ctx.id]);

  std::vector<unsigned> index_array(numFaces * 3);

  // for (unsigned t = 0; t < numFaces; ++t)
  // {
    index_array[0] = 0;
    index_array[1] = 1;
    index_array[2] = 4;

    index_array[3] = 1;
    index_array[4] = 4;
    index_array[5] = 5;

    index_array[6] = 0;
    index_array[7] = 1;
    index_array[8] = 3;

    index_array[9] = 1;
    index_array[10] = 2;
    index_array[11] = 3;

    index_array[12] = 1;
    index_array[13] = 2;
    index_array[14] = 5;

    index_array[15] = 2;
    index_array[16] = 5;
    index_array[17] = 6;

    index_array[18] = 4;
    index_array[19] = 5;
    index_array[20] = 7;

    index_array[21] = 5;
    index_array[22] = 6;
    index_array[23] = 7;

    index_array[24] = 0;
    index_array[25] = 3;
    index_array[26] = 4;

    index_array[27] = 3;
    index_array[28] = 4;
    index_array[29] = 7;

    index_array[30] = 2;
    index_array[31] = 3;
    index_array[32] = 7;

    index_array[33] = 2;
    index_array[34] = 6;
    index_array[35] = 7;
  // }

  indices_[ctx.id] = ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                                                      scm::gl::USAGE_STATIC_DRAW,
                                                      numFaces * 3 * sizeof(unsigned),
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

void ConeTreeRessource::draw(RenderContext const& ctx) const {

  // upload to GPU if neccessary
  if (vertices_.size() <= ctx.id || vertices_[ctx.id] == nullptr)
  {
    upload_to(ctx);
  }

  scm::gl::context_vertex_input_guard vig(ctx.render_context);

  ctx.render_context->bind_vertex_array(vertex_array_[ctx.id]);
  ctx.render_context->bind_index_buffer(indices_[ctx.id], scm::gl::PRIMITIVE_TRIANGLE_LIST, scm::gl::TYPE_UINT);

  ctx.render_context->apply();
  ctx.render_context->draw_elements(36);
}

////////////////////////////////////////////////////////////////////////////////

void ConeTreeRessource::ray_test(Ray const& ray, PickResult::Options options,
                    Node* owner, std::set<PickResult>& hits) {

  //kd_tree_.ray_test(ray, mesh_, options, owner, hits);
}

////////////////////////////////////////////////////////////////////////////////

unsigned int ConeTreeRessource::num_vertices() const { return 0; }

////////////////////////////////////////////////////////////////////////////////

unsigned int ConeTreeRessource::num_faces() const { return 0; }

////////////////////////////////////////////////////////////////////////////////

scm::math::vec3 ConeTreeRessource::get_vertex(unsigned int i) const {

  return scm::math::vec3(
      0.f,0.f,0.f);
}

////////////////////////////////////////////////////////////////////////////////
/*
std::vector<unsigned int> ConeTreeRessource::get_face(unsigned int i) const {

  std::vector<unsigned int> face(mesh_->mFaces[i].mNumIndices);
  for (unsigned int j = 0; j < mesh_->mFaces[i].mNumIndices; ++j)
    face[j] = mesh_->mFaces[i].mIndices[j];
  return face;
}
*/
////////////////////////////////////////////////////////////////////////////////

/*virtual*/ GeometryUberShader* ConeTreeRessource::get_ubershader() const {
  return Singleton<ConeTreeUberShader>::instance();
}

}

