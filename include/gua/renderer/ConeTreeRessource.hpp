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

#ifndef GUA_CONE_TREE_RESSOURCE_HPP
#define GUA_CONE_TREE_RESSOURCE_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/GeometryRessource.hpp>
#include <gua/renderer/ConeTreeUberShader.hpp>
#include <gua/utils/KDTree.hpp>

// external headers
#include <scm/gl_core.h>

#include <mutex>
#include <thread>

#include <vector>

namespace{
struct Vertex {
  scm::math::vec3f pos;
  scm::math::vec2f tex;
  scm::math::vec3f normal;
  scm::math::vec3f tangent;
  scm::math::vec3f bitangent;
};
}

namespace gua {


struct RenderContext;

/**
 * Struct to store the representation of the Scenegraph
 *
 */
struct CTNode {

  void create_layout(int depth, int num_elements, int i, scm::math::vec3f parent_pos);

  int id;
  scm::math::vec3f pos;
  std::vector<CTNode> children;
  static int id_counter;
};


/**
 * storees geometry data.
 *
 * A mesh can be loaded from an Assimp mesh and the draw onto multiple
 * contexts.
 * Do not use this class directly, it is just used by the Geometry class to
 * store the individual meshes of a file.
 */
class ConeTreeRessource : public GeometryRessource {
 public:

  /**
   * Default constructor.
   *
   * Creates a new and empty Mesh.
   */
   ConeTreeRessource(CTNode const& root);

  /**
   * Draws the Mesh.
   *
   * Draws the Mesh to the given context.
   *
   * \param context          The RenderContext to draw onto.
   */
  void draw(RenderContext const& context) const;

  /**
  * Traverses the Cone Tree and expandes the bounding Box
  */
  void bounding_box_expand(CTNode const& node);

  void ray_test(Ray const& ray, PickResult::Options options,
                Node* owner, std::set<PickResult>& hits);

  unsigned int num_vertices() const;

  unsigned int num_faces() const;

  scm::math::vec3 get_vertex(unsigned int i) const;

  //std::vector<unsigned int> get_face(unsigned int i) const;

  /*virtual*/ std::shared_ptr<GeometryUberShader> create_ubershader() const;

 private:
  /*
  * create Spheres
  */
  std::vector<Vertex> generate_sphere_vertices(unsigned int rings, unsigned int sectors, float radius) const;
  std::vector<unsigned> generate_sphere_indices(unsigned int rings, unsigned int sectors) const;
  
  void upload_to(RenderContext const& context) const;

  mutable std::vector<scm::gl::buffer_ptr> vertices_lines_;
  mutable std::vector<scm::gl::buffer_ptr> vertices_spheres_;
  mutable std::vector<scm::gl::buffer_ptr> indices_lines_;
  mutable std::vector<scm::gl::buffer_ptr> indices_spheres_;
  mutable std::vector<scm::gl::vertex_array_ptr> vertex_array_lines_;
  mutable std::vector<scm::gl::vertex_array_ptr> vertex_array_spheres_;
  mutable std::mutex upload_mutex_;

  CTNode cone_tree_root_;
  unsigned int num_nodes_;

 public:

  KDTree kd_tree_;
};



}

#endif  // GUA_CONE_TREE_RESSOURCE_HPP
