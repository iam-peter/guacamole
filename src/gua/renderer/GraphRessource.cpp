
#include <gua/renderer/GraphRessource.hpp>

#include <ogdf/basic/GraphAttributes.h>
#include <ogdf/basic/graph_generators.h>
#include <ogdf/layered/MedianHeuristic.h>
#include <ogdf/layered/OptimalHierarchyLayout.h>
#include <ogdf/layered/OptimalRanking.h>
#include <ogdf/layered/SugiyamaLayout.h>

namespace gua
{

GraphRessource::GraphRessource()
{

}

void GraphRessource::upload_to(RenderContext const& ctx) const
{
  ogdf::Graph graph;

  ogdf::randomSimpleGraph(graph,19,19);

  ogdf::GraphAttributes g_attr(graph);

  for(ogdf::node n = graph.firstNode() ; n ; n = n->succ())
  {
    g_attr.width(n)  = 1.0;
    g_attr.height(n) = 1.0;
  }

  ogdf::SugiyamaLayout sl;

  sl.setRanking(new ogdf::OptimalRanking);
  sl.setCrossMin(new ogdf::MedianHeuristic);

  ogdf::OptimalHierarchyLayout * ohl = new ogdf::OptimalHierarchyLayout;

  sl.setLayout(ohl);
  sl.call(g_attr);

  float width  = g_attr.boundingBox().width() / 2.0 ,
        height = g_attr.boundingBox().height() / 2.0;

  std::unique_lock<std::mutex> lock(upload_mutex_);

  if(vertices_.size() <= ctx.id)
  {
    vertices_.resize(ctx.id + 1);
    indices_.resize(ctx.id + 1);
    vertex_array_.resize(ctx.id + 1);
  }

  float    const radius = 10.0f;
  unsigned const rings  = 20 , sectors = 20;

  std::vector<Vertex>   vertices;
  std::vector<unsigned> indices;

  for(ogdf::node n = graph.firstNode() ; n ; n = n->succ())
  {
    scm::math::vec3f center(g_attr.x(n)-width,g_attr.y(n)-height,0.0f);

    std::vector<Vertex>   v_tmp(node_vertices(rings,sectors,radius,center));
    std::vector<unsigned> i_tmp(node_indices(rings,sectors,vertices.size()));

    vertices.insert(vertices.end(),v_tmp.begin(),v_tmp.end());
    indices.insert(indices.end(),i_tmp.begin(),i_tmp.end());
  }

  node_faces_ = indices.size() / 3;

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

void GraphRessource::

draw(RenderContext const& ctx) const
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
  ctx.render_context->draw_elements(node_faces_ * 3);
}

void GraphRessource::

ray_test(Ray const& ray,PickResult::Options options,
         Node * owner  ,std::set<PickResult> & hits)
{

}

std::shared_ptr<GeometryUberShader> GraphRessource::

create_ubershader() const
{
  return std::make_shared<GraphUberShader>();
}

std::vector<Vertex> const GraphRessource::

node_vertices(unsigned rings,unsigned sectors,
                float radius,scm::math::vec3f const& center) const
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

      tmp.pos       = center + scm::math::vec3f(x,y,z) * radius;
      tmp.normal    = scm::math::vec3f(x,y,z);
      tmp.tex       = scm::math::vec2f(0.f, 0.f);
      tmp.tangent   = scm::math::vec3f(0.f, 0.f, 0.f);
      tmp.bitangent = scm::math::vec3f(0.f, 0.f, 0.f);
      
      vertices.push_back(tmp);
    }

  return vertices;
}

std::vector<Vertex> const GraphRessource::

edge_vertices(scm::math::vec3f const& source,
              scm::math::vec3f const& target) const
{
  unsigned const sectors = 20;
  float    const radius  = 1.0f;

  std::vector<Vertex> v;

  

  return v;
}

std::vector<unsigned> const GraphRessource::

node_indices(unsigned rings,unsigned sectors,unsigned offset) const
{
  std::vector<unsigned> indices;

  for (unsigned r(0); r < rings - 1; ++r)
    for (unsigned s(0); s < sectors - 1; ++s)
    {
      indices.push_back(r * sectors + s + offset);
      indices.push_back(r * sectors + s + 1 + offset);
      indices.push_back((r + 1) * sectors + s + offset);
      
      indices.push_back(r * sectors + s + 1 + offset);
      indices.push_back((r + 1) * sectors + s + 1 + offset);
      indices.push_back((r + 1) * sectors + s + offset);
    }

  return indices;
}

}

