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
#include <gua/renderer/ScatterPlotLoader.hpp>

// guacamole headers
#include <gua/utils/DataSet.hpp>
#include <gua/utils/TextFile.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/utils/string_utils.hpp>
#include <gua/node/ScatterPlotNode.hpp>
#include <gua/node/TransformNode.hpp>
#include <gua/renderer/Material.hpp>
#include <gua/renderer/MaterialLoader.hpp>
#include <gua/renderer/ScatterPlotLoader.hpp>
#include <gua/renderer/ScatterPlotRessource.hpp>
#include <gua/databases/MaterialDatabase.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/ShadingModelDatabase.hpp>

//external
#include <queue>

namespace gua {

std::map<std::string, std::shared_ptr<utils::DataSet>> ScatterPlotLoader::loaded_csvs_;


/////////////////////////////////////////////////////////////////////////////

ScatterPlotLoader::ScatterPlotLoader()
  : GeometryLoader()
{}


////////////////////////////////////////////////////////////////////////////////

/*std::shared_ptr<Node> ScatterPlotLoader::create() 
{
  Logger::LOG_WARNING << "CREATE" << std::endl;

  // TODO set meaningful name
  std::string name = "instance_name123";
  GeometryDatabase::instance()->add(name, std::make_shared<ScatterPlotRessource>());
  
  auto node = std::make_shared<ScatterPlotNode>(name,"data/objects/teapot.obj","data/materials/Red.gmd");
  //node->set_material("data/materials/Red.gmd");
  
  return node;
}*/

std::shared_ptr<node::Node> ScatterPlotLoader::create_from_csvfile(
    std::string const& node_name
  , std::string const& material
  , std::string const& csv_file_name
  , std::string const& separator
  , std::string const& escape
  , std::string const& quote
  , std::string const& xattrib_name
  , std::string const& yattrib_name
  , std::string const& zattrib_name
  )
{

  std::shared_ptr<utils::DataColumn> xdata = nullptr;
  std::shared_ptr<utils::DataColumn> ydata = nullptr;
  std::shared_ptr<utils::DataColumn> zdata = nullptr;

  auto cached_csv_data = ScatterPlotLoader::loaded_csvs_.find(csv_file_name);
  std::shared_ptr<utils::DataSet> csv_data;

  // csv-file was not loaded yet, load it and cache its data
  if (cached_csv_data == loaded_csvs_.end())
  {
    csv_data = std::make_shared<utils::DataSet>();
    csv_data->load_from_csv(csv_file_name, separator, escape, quote);
    ScatterPlotLoader::loaded_csvs_.insert(std::pair<std::string, std::shared_ptr<utils::DataSet>>(csv_file_name, csv_data));
  } else
    csv_data = cached_csv_data->second;

  // TODO load first/second/third attribute when no attribute name is given

  if (!xattrib_name.empty())
    xdata = csv_data->get_column_by_name(xattrib_name);
  if (!yattrib_name.empty())
    ydata = csv_data->get_column_by_name(yattrib_name);
  if (!zattrib_name.empty())
    zdata = csv_data->get_column_by_name(zattrib_name);

  GeometryDatabase::instance()->add(node_name, std::make_shared<ScatterPlotRessource>(xdata, ydata, zdata));

  return std::make_shared<node::ScatterPlotNode>(node_name, node_name, material);
}

////////////////////////////////////////////////////////////////////////////////

bool ScatterPlotLoader::is_supported(std::string const& file_name) const {
  
  return true;
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/*
std::shared_ptr<Node> ScatterPlotLoader::get_tree(std::shared_ptr<Assimp::Importer> const& importer,
                                              aiScene const* ai_scene,
                                              aiNode* ai_root,
                                              std::string const& file_name,
                                              unsigned flags, unsigned& mesh_count) {

  // creates a geometry node and returns it
  auto load_geometry = [&](int i) {
    // load geometry
    std::string mesh_name("type=file&file=" + file_name + "&id=" + string_utils::to_string(mesh_count++) + "&flags=" + string_utils::to_string(flags));
    GeometryDatabase::instance()->add(mesh_name, std::make_shared<ScatterPlotRessource>(ai_scene->mMeshes[ai_root->mMeshes[i]], importer, flags & ScatterPlotLoader::MAKE_PICKABLE));

    // load material
    std::string material_name("");
    unsigned material_index(ai_scene->mMeshes[ai_root->mMeshes[i]]->mMaterialIndex);

    if (material_index != 0 && flags & ScatterPlotLoader::LOAD_MATERIALS) {
      MaterialLoader material_loader;
      aiMaterial const* material(ai_scene->mMaterials[material_index]);
      material_name = material_loader.load_material(material, file_name);
    }

    return std::make_shared<ScatterPlotNode>(mesh_name, mesh_name, material_name);
  };

  // there is only one child -- skip it!
  if (ai_root->mNumChildren == 1 && ai_root->mNumMeshes == 0) {
    return get_tree(
      importer, ai_scene, ai_root->mChildren[0],
      file_name, flags, mesh_count
    );
  }

  // there is only one geometry --- return it!
  if (ai_root->mNumChildren == 0 && ai_root->mNumMeshes == 1) {
    return load_geometry(0);
  }

  // else: there are multiple children and meshes
  auto group(std::make_shared<TransformNode>());

  for (unsigned i(0); i < ai_root->mNumMeshes; ++i) {
    group->add_child(load_geometry(i));
  }

  for (unsigned i(0); i < ai_root->mNumChildren; ++i) {
    group->add_child(
      get_tree(
        importer, ai_scene, ai_root->mChildren[i],
        file_name, flags, mesh_count
      )
    );
  }

  return group;
}*/

}
