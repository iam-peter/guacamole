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

// external headers
#include <queue>
#include <cmath>
#define PI 3.14159265  

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

int CTNode::id_counter = 0;

void CTNode::create_layout(int depth, int num_elements, int index, scm::math::vec3f parent_pos){
  double angle = index * 360/num_elements;
  angle = angle * PI / 180.0;
  double radius = num_elements ;
  double lower = -1.0;

  if(num_elements == 1){
    pos = parent_pos + scm::math::vec3f(0, lower, 0);
  }else{
    pos = parent_pos + scm::math::vec3f(std::cos(angle), lower, std::sin(angle));
  }

  int count(0);
  for(auto & i: children){
    i.create_layout(depth+1, children.size(), count, pos);
    count++;
  }
  return ;
}


////////////////////////////////////////////////////////////////////////////////

ConeTreeRessource::ConeTreeRessource(CTNode const& root)
    : vertices_(), indices_lines_(), indices_spheres_(), vertex_array_(), upload_mutex_(), cone_tree_root_(root), num_nodes_(1) 
    {

      // traverse the tree and count the number of nodes
      std::queue<CTNode> queue;
      queue.push(cone_tree_root_);
      while (!queue.empty())
      {
        num_nodes_ += queue.front().children.size();
        for (unsigned int i(0); i < queue.front().children.size(); ++i)
          queue.push(queue.front().children[i]);
        queue.pop();
      }

      bounding_box_ = math::BoundingBox<math::vec3>(scm::math::vec3(0,0,0));
      bounding_box_expand(root);
    }

////////////////////////////////////////////////////////////////////////////////

void ConeTreeRessource::bounding_box_expand(CTNode const& node)
{
  bounding_box_.expandBy(node.pos);
  for(auto & i: node.children){
    bounding_box_expand(i);
  }
}

////////////////////////////////////////////////////////////////////////////////


void ConeTreeRessource::upload_to(RenderContext const& ctx) const
{

  std::unique_lock<std::mutex> lock(upload_mutex_);

  if (vertices_.size() <= ctx.id) {
    vertices_.resize(ctx.id + 1);
    indices_lines_.resize(ctx.id + 1);
    indices_spheres_.resize(ctx.id + 1);
    vertex_array_.resize(ctx.id + 1);
  }

  std::vector<unsigned> index_array;
  std::vector<unsigned> index_array2;


  vertices_[ctx.id] =  ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                                        scm::gl::USAGE_STATIC_DRAW,
                                                        (num_nodes_+3) * sizeof(Vertex),  //TODO Correct number
                                                        0);



  Vertex* data(static_cast<Vertex*>(ctx.render_context->map_buffer(vertices_[ctx.id], scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));

  std::queue<CTNode> queue;
  queue.push(cone_tree_root_);
  while (!queue.empty())
  {
    CTNode& current = queue.front();

    data[current.id].pos = queue.front().pos;
    data[current.id].tex = scm::math::vec2(0.f, 0.f);
    data[current.id].normal = scm::math::vec3(0.f, 0.f, -1.f);
    data[current.id].tangent = scm::math::vec3(0.f, 0.f, 0.f);
    data[current.id].bitangent = scm::math::vec3(0.f, 0.f, 0.f);

    for (unsigned int i(0); i < queue.front().children.size(); ++i)
    {
      queue.push(queue.front().children[i]);
      index_array.push_back(queue.front().id);
      index_array.push_back(queue.front().children[i].id);
    }
    queue.pop();
  }

  //TESTTESTTESTTESTTESTTESTTESTTESTTEST  //TODO create spheres
  CTNode& tmp = queue.front();
  data[tmp.id_counter].pos = scm::math::vec3(-0.1f, 0.0f, 0.0f);
  data[tmp.id_counter].tex = scm::math::vec2(0.f, 0.f);
  data[tmp.id_counter].normal = scm::math::vec3(0.f, 0.f, -1.f);
  data[tmp.id_counter].tangent = scm::math::vec3(0.f, 0.f, 0.f);
  data[tmp.id_counter].bitangent = scm::math::vec3(0.f, 0.f, 0.f);

  data[tmp.id_counter+1].pos = scm::math::vec3(0.1f, 0.0f, 0.0f);
  data[tmp.id_counter+1].tex = scm::math::vec2(0.f, 0.f);
  data[tmp.id_counter+1].normal = scm::math::vec3(0.f, 0.f, -1.f);
  data[tmp.id_counter+1].tangent = scm::math::vec3(0.f, 0.f, 0.f);
  data[tmp.id_counter+1].bitangent = scm::math::vec3(0.f, 0.f, 0.f);

  data[tmp.id_counter+2].pos = scm::math::vec3(0.0f, 0.15f, 0.0f);
  data[tmp.id_counter+2].tex = scm::math::vec2(0.f, 0.f);
  data[tmp.id_counter+2].normal = scm::math::vec3(0.f, 0.f, -1.f);
  data[tmp.id_counter+2].tangent = scm::math::vec3(0.f, 0.f, 0.f);
  data[tmp.id_counter+2].bitangent = scm::math::vec3(0.f, 0.f, 0.f);

  index_array2.push_back(tmp.id_counter);
  index_array2.push_back(tmp.id_counter+1);
  index_array2.push_back(tmp.id_counter+2);
  //TESTTESTTESTTESTTESTTESTTESTTESTTEST

  for (unsigned int i(0); i < index_array.size(); ++i)
    Logger::LOG_WARNING << "IA1  " << index_array[i] << std::endl;

  for (unsigned int i(0); i < index_array2.size(); ++i)
    Logger::LOG_WARNING << "IA2  " << index_array2[i] << std::endl;

  ctx.render_context->unmap_buffer(vertices_[ctx.id]);


  indices_lines_[ctx.id] = ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                                                      scm::gl::USAGE_STATIC_DRAW,
                                                      index_array.size() * sizeof(unsigned),
                                                      &index_array[0]);

  indices_spheres_[ctx.id] = ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                                                      scm::gl::USAGE_STATIC_DRAW,
                                                      index_array2.size() * sizeof(unsigned),
                                                      &index_array2[0]);

  
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
  
  //Bind all verticies
  ctx.render_context->bind_vertex_array(vertex_array_[ctx.id]);
  
  // Draw the Lines
  ctx.render_context->bind_index_buffer(indices_lines_[ctx.id], scm::gl::PRIMITIVE_LINE_LIST, scm::gl::TYPE_UINT);
  ctx.render_context->apply();
  ctx.render_context->draw_elements(100);  //TODO correct number

  // Draw the Triangles (Spheres)
  ctx.render_context->bind_index_buffer(indices_spheres_[ctx.id], scm::gl::PRIMITIVE_TRIANGLE_LIST, scm::gl::TYPE_UINT);
  ctx.render_context->apply();
  ctx.render_context->draw_elements(100);  //TODO correct number
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


/*virtual*/ std::shared_ptr<GeometryUberShader> ConeTreeRessource::create_ubershader() const {
  return std::make_shared<ConeTreeUberShader>();
}

} //namespace

