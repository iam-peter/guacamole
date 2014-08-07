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
#include <gua/renderer/InfoVisLoader.hpp>

// guacamole headers
#include <gua/utils/DataSet.hpp>
#include <gua/utils/TextFile.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/utils/string_utils.hpp>
#include <gua/node/AreaChartNode.hpp>
#include <gua/node/LineChartNode.hpp>
#include <gua/node/PolyLineNode.hpp>
#include <gua/node/ScatterPlotNode.hpp>
#include <gua/node/TransformNode.hpp>
#include <gua/renderer/Material.hpp>
#include <gua/renderer/MaterialLoader.hpp>
#include <gua/renderer/InfoVisLoader.hpp>
#include <gua/renderer/LineChartRessource.hpp>
#include <gua/renderer/ScatterPlotRessource.hpp>
#include <gua/databases/MaterialDatabase.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/ShadingModelDatabase.hpp>

//external
#include <queue>

namespace gua {


/////////////////////////////////////////////////////////////////////////////

InfoVisLoader::InfoVisLoader()
  : GeometryLoader()
{}


////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::Node> InfoVisLoader::create_scatterplot(
    std::string const& node_name
  , std::string const& material
  , std::vector<float> const& xdata
  , std::vector<float> const& ydata
  , std::vector<float> const& zdata
  )
{
  GeometryDatabase::instance()->add(node_name, std::make_shared<ScatterPlotRessource>(xdata, ydata, zdata));

  return std::make_shared<node::ScatterPlotNode>(node_name, node_name, material);
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::Node> InfoVisLoader::create_linechart(
    std::string const& node_name
  , std::string const& material
  , std::vector<float> const& xdata
  , std::vector<float> const& ydata
  )
{
  GeometryDatabase::instance()->add(node_name, std::make_shared<LineChartRessource>(xdata, ydata));

  return std::make_shared<node::LineChartNode>(node_name, node_name, material);
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::Node> InfoVisLoader::create_areachart(
    std::string const& node_name
  , std::string const& material
  , std::vector<float> const& xdata
  , std::vector<float> const& ydata
  )
{
  GeometryDatabase::instance()->add(node_name, std::make_shared<AreaChartRessource>(xdata, ydata));

  return std::make_shared<node::AreaChartNode>(node_name, node_name, material);
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::Node> InfoVisLoader::create_polyline(
    std::string const& node_name
  , std::string const& material
  , std::vector<math::vec3> const& points
  , bool connect_end_points
  )
{
  GeometryDatabase::instance()->add(node_name, std::make_shared<PolyLineRessource>(points, connect_end_points));

  return std::make_shared<node::PolyLineNode>(node_name, node_name, material);
}

////////////////////////////////////////////////////////////////////////////////

bool InfoVisLoader::is_supported(std::string const& file_name) const {

  return true;
}

}
