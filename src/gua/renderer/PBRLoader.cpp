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


// guacamole headers
#include <gua/guacamole.hpp>
#include <gua/renderer/PBRLoader.hpp>
#include <gua/utils/TextFile.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/utils/string_utils.hpp>
#include <gua/node/PBRNode.hpp>
#include <gua/node/TransformNode.hpp>
#include <gua/renderer/Material.hpp>
#include <gua/renderer/MaterialLoader.hpp>
#include <gua/renderer/GeometryLoader.hpp>
#include <gua/renderer/PBRRessource.hpp>
#include <gua/databases/MaterialDatabase.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/ShadingModelDatabase.hpp>

#include <iostream>

namespace gua {

unsigned PBRLoader::model_counter_ = 0;

  ////////////////////////////////////////////////////////////////////////////////

  PBRLoader::PBRLoader() 
    : GeometryLoader(), 
      _supported_file_extensions(),
      node_counter_(0)
  {
    _supported_file_extensions.insert("xyz_all");
   if (!MaterialDatabase::instance()->is_supported("gua_pbr"))
   {
       create_resource_material("gua_pbr",
       Resources::materials_gua_pbr_gsd,
       Resources::materials_gua_pbr_gmd);
   }
  }

  /////////////////////////////////////////////////////////////////////////////

std::shared_ptr<::gua::node::Node> PBRLoader::create_geometry_from_file(std::string const& node_name,
                                                           std::string const& file_name) {

  node_counter_ = 0;
    
    std::shared_ptr<::gua::node::Node> new_node;

     std::shared_ptr<pbr::ren::RawPointCloud> point_cloud = std::make_shared<pbr::ren::RawPointCloud>(model_counter_);
     if (point_cloud->Load(file_name))
     {

	     // load point cloud
	     std::string model_name("type=file&file=" + file_name);
	     GeometryDatabase::instance()->add(model_name, std::make_shared<PBRRessource>(point_cloud));
	     
	     ++model_counter_;
	     

	     auto node(std::make_shared<::gua::node::PBRNode>(model_name));
	     node->set_filename(model_name);
             node->set_material("gua_pbr");

           return node;

    }
    else
    {
	    Logger::LOG_WARNING << "Failed to load object \"" << file_name << "\": File does not exist!" << std::endl;

    	return nullptr;
    }
}

  /////////////////////////////////////////////////////////////////////////////

  bool PBRLoader::is_supported(std::string const& file_name) const 
  {
    std::vector<std::string> filename_decomposition =
      gua::string_utils::split(file_name, '.');
    return filename_decomposition.empty()
      ? false
      : _supported_file_extensions.count(filename_decomposition.back()) > 0;
  }


}
