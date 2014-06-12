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
#include <scm/gl_core/render_device/context_guards.h>


#define PI 3.14159265  


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

ConeTreeRessource::ConeTreeRessource(CTNode const& root, unsigned sphere_resolution)
    : vertices_lines_(), vertices_spheres_(),
      indices_lines_(), indices_spheres_(),
      vertex_array_lines_(), vertex_array_spheres_(),
      upload_mutex_(),
      cone_tree_root_(root), num_nodes_(1), sphere_resolution_(64)
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

std::vector<Vertex> ConeTreeRessource::generate_sphere_vertices(unsigned int rings, unsigned int sectors, float radius) const
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
      tmp.tex = scm::math::vec2(0.f, 0.f);
      tmp.tangent = scm::math::vec3(0.f, 0.f, 0.f);
      tmp.bitangent = scm::math::vec3(0.f, 0.f, 0.f);
      
      vertices.push_back(tmp);  
    }

  return vertices;
}

////////////////////////////////////////////////////////////////////////////////

std::vector<unsigned> ConeTreeRessource::generate_sphere_indices(unsigned int rings, unsigned int sectors) const
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

////////////////////////////////////////////////////////////////////////////////

void ConeTreeRessource::upload_to(RenderContext const& ctx) const
{

  rasterizer_state_ = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_FRONT);

  std::unique_lock<std::mutex> lock(upload_mutex_);

  if (vertices_lines_.size() <= ctx.id) {
    vertices_lines_.resize(ctx.id + 1);
    vertices_spheres_.resize(ctx.id + 1);
    indices_lines_.resize(ctx.id + 1);
    indices_spheres_.resize(ctx.id + 1);
    vertex_array_lines_.resize(ctx.id + 1);
    vertex_array_spheres_.resize(ctx.id + 1);
  }

  std::vector<unsigned> index_array_lines;
  std::vector<unsigned> index_array_spheres;


  vertices_lines_[ctx.id] =  ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                                        scm::gl::USAGE_STATIC_DRAW,
                                                        (num_nodes_) * sizeof(Vertex),  //TODO Correct number
                                                        0);

  vertices_spheres_[ctx.id] =  ctx.render_device->create_buffer(scm:: gl::BIND_VERTEX_BUFFER,
                                                        scm::gl::USAGE_STATIC_DRAW,
                                                        num_nodes_ * (sphere_resolution_) * (sphere_resolution_) * sizeof(Vertex),  //TODO Correct number
                                                        0);


  Vertex* data(static_cast<Vertex*>(ctx.render_context->map_buffer(vertices_lines_[ctx.id], scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));


  std::queue<CTNode> queue; //queue to traverse the coneTree
  //first time to generate the lines
  queue.push(cone_tree_root_);
  while (!queue.empty())
  {
    //verticies
    CTNode& current = queue.front();

    data[current.id].pos = queue.front().pos;
    data[current.id].tex = scm::math::vec2(0.f, 0.f);
    data[current.id].normal = scm::math::vec3(0.f, 0.f, -1.f);
    data[current.id].tangent = scm::math::vec3(0.f, 0.f, 0.f);
    data[current.id].bitangent = scm::math::vec3(0.f, 0.f, 0.f);

    //indices
    for (unsigned int i(0); i < queue.front().children.size(); ++i)
    {
      queue.push(queue.front().children[i]);
      index_array_lines.push_back(queue.front().id);
      index_array_lines.push_back(queue.front().children[i].id);
    }
    queue.pop();
  }
  ctx.render_context->unmap_buffer(vertices_lines_[ctx.id]);


  Vertex* data2(static_cast<Vertex*>(ctx.render_context->map_buffer(vertices_spheres_[ctx.id], scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));
  
  //second traversal to generate the spheres
  queue.push(cone_tree_root_);
  
  std::vector<Vertex> verts = generate_sphere_vertices(sphere_resolution_,sphere_resolution_, 0.1);
  unsigned num_verticies_for_sphere(verts.size());
  std::vector<unsigned> indis = generate_sphere_indices(sphere_resolution_,sphere_resolution_);
  unsigned num_indices_for_sphere(indis.size());
    
  while (!queue.empty())
  {
    //verticies
    CTNode& current = queue.front();
    // TODO create copy of verts with changed positions
    for (int i = 0; i < num_verticies_for_sphere; i++){
      unsigned index = num_verticies_for_sphere*current.id + i;
      data2[index] = verts[i];
      data2[index].pos = data2[index].pos + current.pos;
      // Logger::LOG_WARNING << "VERTEX: Current ID "<< current.id << "  Index: " << index << "  pos: " << data2[index].pos << std::endl;
    }

    for (int i = 0; i < num_indices_for_sphere; i++){
      unsigned index = num_indices_for_sphere*current.id + i;
      index_array_spheres.push_back(indis[i] + num_verticies_for_sphere*current.id);
      // Logger::LOG_WARNING << "INDEX: Current ID "<< current.id << "  Index: " << index << "  Index: " << indis[i] + num_verticies_for_sphere*current.id << std::endl;
    }
    

    for (auto const& child: current.children)
      queue.push(child);
    queue.pop();
  }
  ctx.render_context->unmap_buffer(vertices_spheres_[ctx.id]);
  
  




  indices_lines_[ctx.id] = ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                                                      scm::gl::USAGE_STATIC_DRAW,
                                                      index_array_lines.size() * sizeof(unsigned),
                                                      &index_array_lines[0]);

  indices_spheres_[ctx.id] = ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                                                      scm::gl::USAGE_STATIC_DRAW,
                                                      index_array_spheres.size() * sizeof(unsigned),
                                                      &index_array_spheres[0]);

  
  std::vector<scm::gl::buffer_ptr> buffer_arrays;
  buffer_arrays.push_back(vertices_lines_[ctx.id]);

  vertex_array_lines_[ctx.id] = ctx.render_device->create_vertex_array(
      scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
                             0, 1, scm::gl::TYPE_VEC2F, sizeof(Vertex))(
                             0, 2, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
                             0, 3, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
                             0, 4, scm::gl::TYPE_VEC3F, sizeof(Vertex)),
                             buffer_arrays);  


  std::vector<scm::gl::buffer_ptr> buffer_arrays2;
  buffer_arrays2.push_back(vertices_spheres_[ctx.id]);

  vertex_array_spheres_[ctx.id] = ctx.render_device->create_vertex_array(
      scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
                             0, 1, scm::gl::TYPE_VEC2F, sizeof(Vertex))(
                             0, 2, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
                             0, 3, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
                             0, 4, scm::gl::TYPE_VEC3F, sizeof(Vertex)),
                             buffer_arrays2);
}

////////////////////////////////////////////////////////////////////////////////

void ConeTreeRessource::draw(RenderContext const& ctx) const {

  // upload to GPU if neccessary
  if (vertices_lines_.size() <= ctx.id || vertices_lines_[ctx.id] == nullptr)
  {
    upload_to(ctx);
  }

  scm::gl::context_vertex_input_guard vig(ctx.render_context);
  scm::gl::context_state_objects_guard contex_guard(ctx.render_context);

  ctx.render_context->set_rasterizer_state(rasterizer_state_, 10.0f);
  
  // Draw the Lines
  ctx.render_context->bind_vertex_array(vertex_array_lines_[ctx.id]);
  ctx.render_context->bind_index_buffer(indices_lines_[ctx.id], scm::gl::PRIMITIVE_LINE_LIST, scm::gl::TYPE_UINT);
  ctx.render_context->apply();
  ctx.render_context->draw_elements(100);  //TODO correct number

  // Draw the Triangles (Spheres)
  ctx.render_context->bind_vertex_array(vertex_array_spheres_[ctx.id]);
  ctx.render_context->bind_index_buffer(indices_spheres_[ctx.id], scm::gl::PRIMITIVE_TRIANGLE_LIST, scm::gl::TYPE_UINT);
  ctx.render_context->apply();
  ctx.render_context->draw_elements(num_nodes_ * (sphere_resolution_-1)*(sphere_resolution_-1)*6); 
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

