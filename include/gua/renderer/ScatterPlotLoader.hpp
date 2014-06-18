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

#ifndef GUA_SCATTER_PLOT_LOADER_HPP
#define GUA_SCATTER_PLOT_LOADER_HPP

// guacamole headers
#include <gua/renderer/ScatterPlotRessource.hpp>
#include <gua/renderer/GeometryLoader.hpp>
#include <gua/scenegraph/SceneGraph.hpp>
#include <gua/utils/DataSet.hpp> 


// external headers
#include <string>
#include <list>
#include <memory>

namespace gua {

class Node;
class InnerNode;
class GeometryNode;

/**
 * Loads and draws meshes.
 *
 * This class can load mesh data from files and display them in multiple
 * contexts. A MeshLoader object is made of several Mesh objects.
 */
class GUA_DLL ScatterPlotLoader : public GeometryLoader {


public:

  /**
   * Default constructor.
   *
   * Constructs a new and empty MeshLoader.
   */
   ScatterPlotLoader();

   /**
   *
   */
   std::shared_ptr<Node> create_from_dataset(
      std::string const& node_name
    , std::string const& material
    , utils::DataSet const& data_set
    , std::string const& xattrib_name
    , std::string const& yattrib_name
    , std::string const& zattrib_name = ""
  );
  /**
  *
  */
  bool is_supported(std::string const& file_name) const;

private: // methods

  /*std::shared_ptr<Node> get_tree(std::shared_ptr<Assimp::Importer> const& importer,
                                 aiScene const* ai_scene,
                                 aiNode* ai_root,
                                 std::string const& file_name,
                                 unsigned flags, unsigned& mesh_count);*/

};

}

#endif  // GUA_SCATTER_PLOT_LOADER_HPP
