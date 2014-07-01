
#include <gua/renderer/GraphRessource.hpp>

struct Vertex
{
  scm::math::vec3 pos , normal;
};

namespace gua
{

GraphRessource::GraphRessource()
{

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

  vertices_[ctx.id] = 

  ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                   scm::gl::USAGE_STATIC_DRAW,
                                   sizeof(Vertex) * 3,
                                   0);

  auto access_mode = scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER;

  Vertex * data(static_cast<Vertex *> 

  (ctx.render_context->map_buffer(vertices_[ctx.id],access_mode)));

  data[0].pos = scm::math::vec3(-1.5,-1.5,-1.0);
  data[0].pos = scm::math::vec3( 1.5,-1.5,-1.0);
  data[2].pos = scm::math::vec3( 1.0, 1.5,-1.0);

  data[0].normal = scm::math::vec3(0.0,0.0,1.0);
  data[1].normal = scm::math::vec3(0.0,0.0,1.0);
  data[2].normal = scm::math::vec3(0.0,0.0,1.0);

  ctx.render_context->unmap_buffer(vertices_[ctx.id]);


  std::vector<unsigned> index_array(3);

  index_array[0] = 0;
  index_array[1] = 1;
  index_array[2] = 2;

  indices_[ctx.id] = 

  ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                                   scm::gl::USAGE_STATIC_DRAW,
                                   sizeof(unsigned) * index_array.size(),
                                   &index_array[0]);

  std::vector<scm::gl::buffer_ptr> buffer_arrays;
  buffer_arrays.push_back(vertices_[ctx.id]);

  vertex_array_[ctx.id] = ctx.render_device->create_vertex_array(
      scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
          0, 1, scm::gl::TYPE_VEC3F, sizeof(Vertex)),
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
  ctx.render_context->draw_elements(3);
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

