
#include <gua/renderer/GraphRessource.hpp>

namespace gua
{

GraphRessource::GraphRessource()
{

}


std::vector<Vertex> const GraphRessource::

sphere_vertices(unsigned rings,unsigned sectors,float radius) const
{
  std::vector<Vertex> vertices;

  float const R = 1.0f / (rings - 1);
  float const S = 1.0f / (sectors - 1);

  for (unsigned r(0); r < rings; ++r)
    for (unsigned s(0); s < sectors; ++s)
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

std::vector<unsigned> const GraphRessource::

sphere_indices(unsigned rings,unsigned sectors) const
{
  std::vector<unsigned> indices;

  for (unsigned r(0); r < rings - 1; ++r)
    for (unsigned s(0); s < sectors - 1; ++s)
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

  float    const radius = 1.0f;
  unsigned const rings  = 20 , sectors = 20;

  std::vector<Vertex>   vertices(sphere_vertices(rings,sectors,radius));
  std::vector<unsigned> indices(sphere_indices(rings,sectors));

  face_number_ = indices.size() / 3;

  vertices_[ctx.id] = 

  ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                   scm::gl::USAGE_STATIC_DRAW,
                                   sizeof(Vertex) * vertices.size(),
                                   nullptr);

  auto access_mode = scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER;

  Vertex * data(static_cast<Vertex *> 

  (ctx.render_context->map_buffer(vertices_[ctx.id],access_mode)));

  for(unsigned index = 0 ; index < vertices.size() ; ++index)
  {
    data[index] = vertices[index];
  }

  ctx.render_context->unmap_buffer(vertices_[ctx.id]);

  indices_[ctx.id] = 

  ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                                   scm::gl::USAGE_STATIC_DRAW,
                                   sizeof(unsigned) * indices.size(),
                                   &indices[0]);

  std::vector<scm::gl::buffer_ptr> buffer_arrays;
  buffer_arrays.push_back(vertices_[ctx.id]);

  vertex_array_[ctx.id] = 

  ctx.render_device->create_vertex_array(
  
  scm::gl::vertex_format(0,0,scm::gl::TYPE_VEC3F,sizeof(Vertex))
                        (0,1,scm::gl::TYPE_VEC2F,sizeof(Vertex))
                        (0,2,scm::gl::TYPE_VEC3F,sizeof(Vertex))
                        (0,3,scm::gl::TYPE_VEC3F,sizeof(Vertex))
                        (0,4,scm::gl::TYPE_VEC3F,sizeof(Vertex)),
                        buffer_arrays);
}

void GraphRessource::draw(RenderContext const& ctx) const
{
  if(vertices_.size() <= ctx.id || vertices_[ctx.id] == nullptr)
  {
    upload_to(ctx);
  }

  scm::gl::context_vertex_input_guard vig(ctx.render_context);

  ctx.render_context->bind_vertex_array(vertex_array_[ctx.id]);

  ctx.render_context->bind_index_buffer(indices_[ctx.id],
                                        scm::gl::PRIMITIVE_TRIANGLE_LIST,
                                        scm::gl::TYPE_UINT);

  ctx.render_context->apply();
  ctx.render_context->draw_elements(face_number_ * 3);
}

void GraphRessource::ray_test(Ray const& ray,
                              PickResult::Options options,
                              Node * owner,
                              std::set<PickResult> & hits)
{

}

std::shared_ptr<GeometryUberShader> GraphRessource::create_ubershader() const
{
  return std::make_shared<GraphUberShader>();
}

}

