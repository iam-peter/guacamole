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

#ifndef GUA_INFOVIS_LOADER_HPP
#define GUA_INFOVIS_LOADER_HPP

// guacamole headers
#include <gua/renderer/LineChartRessource.hpp>
#include <gua/renderer/ScatterPlotRessource.hpp>
#include <gua/renderer/GeometryLoader.hpp>
#include <gua/scenegraph/SceneGraph.hpp>
#include <gua/utils/DataSet.hpp>


// external headers
#include <map>
#include <string>
#include <list>
#include <memory>

namespace gua {

namespace node {
class Node;
class InnerNode;
class GeometryNode;
}

/**
 * Loads and draws meshes.
 *
 * This class can load mesh data from files and display them in multiple
 * contexts. A MeshLoader object is made of several Mesh objects.
 */
class GUA_DLL InfoVisLoader : public GeometryLoader {


public:

  /**
   * Default constructor.
   *
   * Constructs a new and empty MeshLoader.
   */
   InfoVisLoader();

   /**
   *
   */
   std::shared_ptr<node::Node> create_scatterplot(
      std::string const& node_name
    , std::string const& material
    , std::vector<float> const& xdata
    , std::vector<float> const& ydata
    , std::vector<float> const& zdata = std::vector<float>()
  );

   std::shared_ptr<node::Node> create_linechart(
      std::string const& node_name
    , std::string const& material
    , std::vector<float> const& xdata
    , std::vector<float> const& ydata
  );
  /**
  *
  */
  bool is_supported(std::string const& file_name) const;

};

}

#endif  // GUA_INFOVIS_LOADER_HPP
