
#include <gua/renderer/GraphRessource.hpp>

#include <ogdf/basic/graph_generators.h>
#include <ogdf/layered/MedianHeuristic.h>
#include <ogdf/layered/OptimalHierarchyLayout.h>
#include <ogdf/layered/OptimalRanking.h>
#include <ogdf/layered/SugiyamaLayout.h>

namespace gua
{

GraphRessource::

GraphRessource()
{}


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
  ctx.render_context->draw_elements(faces_ * 3);
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

  std::vector<Vertex>   vertices;
  std::vector<unsigned> indices;

  create_geometry(vertices,indices);

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


std::shared_ptr<GeometryUberShader> GraphRessource::

create_ubershader() const
{
  return std::make_shared<GraphUberShader>();
}


void GraphRessource::ray_test(Ray const& ray,
                              PickResult::Options options,
                              Node * owner,
                              std::set<PickResult> & hits)
{}


void GraphRessource::

layout_apply() const
{
  ogdf::SugiyamaLayout sl;

  sl.setRanking(new ogdf::OptimalRanking);
  sl.setCrossMin(new ogdf::MedianHeuristic);

  ogdf::OptimalHierarchyLayout * ohl = new ogdf::OptimalHierarchyLayout;

  sl.setLayout(ohl);
  sl.call(g_attr_);

  layout_correction();
}


void GraphRessource::

layout_correction() const
{
  double x_shift = g_attr_.boundingBox().width()  / 1.5,
         y_shift = g_attr_.boundingBox().height() / 1.5;

  for(ogdf::node n = graph_.firstNode() ; n ; n = n->succ())
  {
    g_attr_.x(n) -= x_shift;
    g_attr_.y(n) -= y_shift;
  }  
}


void GraphRessource::

generate_graph(unsigned short nodes,unsigned short edges) const
{
  ogdf::randomSimpleGraph(graph_,nodes,edges);

  //g_attr_.init(graph_,g_attr_.attributes());
  g_attr_ = ogdf::GraphAttributes(graph_);

  for(ogdf::node n = graph_.firstNode() ; n ; n = n->succ())
  {
    g_attr_.width(n)  = 3.0;
    g_attr_.height(n) = 3.0;
  }
}


void GraphRessource::

create_geometry(std::vector<Vertex>   & vertices,
                std::vector<unsigned> & indices) const
{
  unsigned const rings  = 20 , sectors = 20;

  for(ogdf::edge e = graph_.firstEdge() ; e ; e = e->succ())
  {
    ogdf::node source = e->source(),
               target = e->target();

    scm::math::vec3 pos_source(g_attr_.x(source),g_attr_.y(source),0.0),
                    pos_target(g_attr_.x(target),g_attr_.y(target),0.0);

    std::vector<Vertex> v_tmp(edge_vertices(pos_source,pos_target));
    std::vector<unsigned> i_tmp(edge_indices(vertices.size()));

    vertices.insert(vertices.end(),v_tmp.begin(),v_tmp.end());
    indices.insert(indices.end(),i_tmp.begin(),i_tmp.end());
  }

  for(ogdf::node n = graph_.firstNode() ; n ; n = n->succ())
  {
    scm::math::vec3f center(g_attr_.x(n),g_attr_.y(n),0.0f);

    std::vector<Vertex>   v_tmp(node_vertices(rings,sectors,3.0,center));
    std::vector<unsigned> i_tmp(node_indices(rings,sectors,vertices.size()));

    vertices.insert(vertices.end(),v_tmp.begin(),v_tmp.end());
    indices.insert(indices.end(),i_tmp.begin(),i_tmp.end());
  }

  faces_ = indices.size() / 3;
}


std::vector<Vertex> const GraphRessource::

node_vertices(unsigned short rings,
              unsigned short sectors,
              double radius,
              scm::math::vec3f const& center) const
{
  std::vector<Vertex> vertices;

  double const R = 1.0 / (rings - 1);
  double const S = 1.0 / (sectors - 1);

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
  unsigned const sectors = 40;
  float    const radius  = 0.5 , rad_increment = 2 * M_PI / sectors;

  std::vector<Vertex> vertices;

  scm::math::vec3f normal(target-source);
  scm::math::vec3f u(normal.y,-normal.x,0.0);
  scm::math::vec3f v(scm::math::cross(normal,u));

  u = scm::math::normalize(u) * radius;
  v = scm::math::normalize(v) * radius;

  Vertex vertex;

  vertex.tex       = scm::math::vec2f(0.f,0.f);
  vertex.tangent   = scm::math::vec3f(0.f,0.f,0.f);
  vertex.bitangent = scm::math::vec3f(0.f,0.f,0.f); 

  for(double rad = 0.0 ; rad < 2.0 * M_PI ; rad += rad_increment)
  {
    scm::math::vec3f pos(source);

    pos += u * std::cos(rad) + v * std::sin(rad);

    vertex.pos    = pos;

    vertex.normal = scm::math::normalize(pos - source);

    vertices.push_back(vertex);

    vertex.pos += normal;

    vertices.push_back(vertex);
  }

  return vertices;
}


std::vector<unsigned> const GraphRessource::

node_indices(unsigned short rings,
             unsigned short sectors,
             unsigned offset) const
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


std::vector<unsigned> const GraphRessource::

edge_indices(unsigned offset) const
{
  std::vector<unsigned> indices;

  unsigned const segments = 40;

  for(unsigned short triangle = 0 ; triangle < segments * 2 - 2; ++triangle)
  {
    unsigned short index = offset + triangle;

    indices.push_back(index);
    indices.push_back(index+1);
    indices.push_back(index+2);
  }

  return indices;
}

}

