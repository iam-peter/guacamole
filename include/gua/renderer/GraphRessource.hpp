
#ifndef GUA_GRAPH_RESSOURCE_HPP
#define GUA_GRAPH_RESSOURCE_HPP

#include <gua/renderer/GeometryRessource.hpp>
#include <gua/renderer/GraphUberShader.hpp>

#include <scm/gl_core.h>

#include <mutex>
#include <thread>
#include <vector>

#include <ogdf/basic/GraphAttributes.h>

struct Vertex
{
  scm::math::vec3 pos;
  scm::math::vec3 normal;
  scm::math::vec3 color;
};

namespace gua
{

class GraphRessource : public GeometryRessource
{
  public:

  GraphRessource();

  void draw(RenderContext const& context) const;

  void ray_test(Ray const& ray,
                PickResult::Options options,
                Node * owner, 
                std::set<PickResult> & hits);

  std::shared_ptr<GeometryUberShader> create_ubershader() const;


  void layout_apply() const;
  void generate_graph(unsigned short nodes,unsigned short edges) const;


  private:

  void upload_to(RenderContext const& context) const;


  void layout_correction() const;  

  void create_geometry(std::vector<Vertex>   & vertices,
                       std::vector<unsigned> & indices) const;

  std::vector<Vertex> const node_vertices(unsigned short rings,
                                          unsigned short sectors,
                                          double radius,
                                          scm::math::vec3 const& pos) const;

  std::vector<Vertex> const edge_vertices(scm::math::vec3 const& source,
                                          scm::math::vec3 const& target) const;

  std::vector<unsigned> const node_indices(unsigned short rings,
                                           unsigned short sectors,
                                           unsigned offset) const;

  std::vector<unsigned> const edge_indices(unsigned offset) const;



  mutable std::vector<scm::gl::buffer_ptr>          vertices_;
  mutable std::vector<scm::gl::buffer_ptr>          indices_;
  mutable std::vector<scm::gl::vertex_array_ptr>    vertex_array_;
  mutable unsigned                                  faces_;

  mutable std::mutex                                upload_mutex_;

  mutable ogdf::Graph                               graph_;
  mutable ogdf::GraphAttributes                     g_attr_;
};

}

#endif

