
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

#ifndef GUA_GRAPH_NODE_HPP
#define GUA_GRAPH_NODE_HPP

// guacamole headers
#include <gua/scenegraph/GeometryNode.hpp>

namespace gua
{

class GUA_DLL GraphNode : public GeometryNode
{
	public :

  GraphNode(std::string const& name,
            std::string const& geometry  = "gua_default_geometry",
            std::string const& material  = "gua_default_material",
            math::mat4  const& transform = math::mat4::identity());

	void update_cache();

	protected:

	std::shared_ptr<Node> copy() const;
};

}

#endif

