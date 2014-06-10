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
#include <gua/renderer/ConeTreeLoader.hpp>

// guacamole headers
#include <gua/utils/TextFile.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/utils/string_utils.hpp>
#include <gua/scenegraph/ConeTreeNode.hpp>
#include <gua/scenegraph/TransformNode.hpp>
#include <gua/renderer/Material.hpp>
#include <gua/renderer/MaterialLoader.hpp>
#include <gua/renderer/ConeTreeLoader.hpp>
#include <gua/renderer/ConeTreeRessource.hpp>
#include <gua/databases/MaterialDatabase.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/ShadingModelDatabase.hpp>

//external
#include <queue>

namespace gua {


/////////////////////////////////////////////////////////////////////////////

ConeTreeLoader::ConeTreeLoader()
  : GeometryLoader() 
{}


////////////////////////////////////////////////////////////////////////////////

/*std::shared_ptr<Node> ConeTreeLoader::create() 
{
  Logger::LOG_WARNING << "CREATE" << std::endl;

  // TODO set meaningful name
  std::string name = "instance_name123";
  GeometryDatabase::instance()->add(name, std::make_shared<ConeTreeRessource>());
  
  auto node = std::make_shared<ConeTreeNode>(name,"data/objects/teapot.obj","data/materials/Red.gmd");
  //node->set_material("data/materials/Red.gmd");
  
  return node;
}*/

std::shared_ptr<Node> ConeTreeLoader::create(std::string const& node_name,
                                             std::string const& material,
                                             SceneGraph const& graph)
{
  CTNode root = scenegraph_to_CT_Node(graph.get_root());
  root.create_layout(0, 1 , 0, scm::math::vec3f(0, 1, 0));

  
  GeometryDatabase::instance()->add(node_name, std::make_shared<ConeTreeRessource>(root));
      Logger::LOG_WARNING << "lauft" << std::endl;
  
  return std::make_shared<ConeTreeNode>(node_name, node_name, material);
  
}
////////////////////////////////////////////////////////////////////////////////

/**
* Creates a representation of the Scenegraph for the Cone Tree
*/
CTNode ConeTreeLoader::scenegraph_to_CT_Node(std::shared_ptr<Node> node) const {
  CTNode tmp;
  tmp.id = tmp.id_counter;
  ++tmp.id_counter;
  for (auto const& i: node->get_children()){
    CTNode child = scenegraph_to_CT_Node(i);
    tmp.children.push_back(child);
  }
  return tmp;
}

////////////////////////////////////////////////////////////////////////////////


bool ConeTreeLoader::is_supported(std::string const& file_name) const {
  
  return true;
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/*
std::shared_ptr<Node> ConeTreeLoader::get_tree(std::shared_ptr<Assimp::Importer> const& importer,
                                              aiScene const* ai_scene,
                                              aiNode* ai_root,
                                              std::string const& file_name,
                                              unsigned flags, unsigned& mesh_count) {

  // creates a geometry node and returns it
  auto load_geometry = [&](int i) {
    // load geometry
    std::string mesh_name("type=file&file=" + file_name + "&id=" + string_utils::to_string(mesh_count++) + "&flags=" + string_utils::to_string(flags));
    GeometryDatabase::instance()->add(mesh_name, std::make_shared<ConeTreeRessource>(ai_scene->mMeshes[ai_root->mMeshes[i]], importer, flags & ConeTreeLoader::MAKE_PICKABLE));

    // load material
    std::string material_name("");
    unsigned material_index(ai_scene->mMeshes[ai_root->mMeshes[i]]->mMaterialIndex);

    if (material_index != 0 && flags & ConeTreeLoader::LOAD_MATERIALS) {
      MaterialLoader material_loader;
      aiMaterial const* material(ai_scene->mMaterials[material_index]);
      material_name = material_loader.load_material(material, file_name);
    }

    return std::make_shared<ConeTreeNode>(mesh_name, mesh_name, material_name);
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
