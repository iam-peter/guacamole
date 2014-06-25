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
#include <gua/scenegraph/GraphNode.hpp>

#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/MaterialDatabase.hpp>
#include <gua/renderer/GraphLoader.hpp>

namespace gua
{
	GraphNode::GraphNode(std::string const& name,
                       std::string const& filename,
                       std::string const& material,
                       math::mat4 const& transform) :
	GeometryNode(name, filename, material)
  {}

  void GraphNode::update_cache()
	{
    GeometryNode::update_cache();
  }

  std::shared_ptr<Node> GraphNode::copy() const 
	{
    auto result(std::make_shared<GraphNode>(get_name(),
																					  filename_,
																						material_,
																						get_transform()));
    result->shadow_mode_ = shadow_mode_;
    return result;
  }
}

