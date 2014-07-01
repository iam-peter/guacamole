
#ifndef GUA_GRAPH_RESSOURCE_HPP
#define GUA_GRAPH_RESSOURCE_HPP

#include <gua/renderer/GeometryRessource.hpp>
#include <gua/renderer/GraphUberShader.hpp>

#include <scm/gl_core.h>

#include <mutex>
#include <thread>
#include <vector>

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

  private:

  void upload_to(RenderContext const& context) const;


  mutable std::vector<scm::gl::buffer_ptr> vertices_;
  mutable std::vector<scm::gl::buffer_ptr> indices_;
  mutable std::vector<scm::gl::vertex_array_ptr> vertex_array_;
  mutable std::mutex upload_mutex_;
};

}

#endif
