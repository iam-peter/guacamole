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
#include <gua/renderer/GBufferPass.hpp>

// guacamole headers
#include <algorithm>

#include <gua/platform.hpp>
#include <gua/utils.hpp>
#include <gua/traverse.hpp>

#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/TriMeshUberShader.hpp>
#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/renderer/GeometryRessource.hpp>
#include <gua/renderer/GeometryUberShader.hpp>

#include <gua/node/GeometryNode.hpp>
#include <gua/scenegraph/SceneGraph.hpp>
#include <gua/node/TriMeshNode.hpp>

#include <gua/databases.hpp>
#include <gua/databases/GeometryDatabase.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

GBufferPass::GBufferPass(Pipeline* pipeline)
    : GeometryPass(pipeline),
      bfc_rasterizer_state_(),
      no_bfc_rasterizer_state_(),
      bbox_rasterizer_state_(),
      depth_stencil_state_()
{
  initialize_trimesh_ubershader(pipeline->get_context());
}

////////////////////////////////////////////////////////////////////////////////

GBufferPass::~GBufferPass() {}

////////////////////////////////////////////////////////////////////////////////

void GBufferPass::create(
    RenderContext const& ctx,
    std::vector<std::pair<BufferComponent, scm::gl::sampler_state_desc> > const&
        layers) {

  scm::gl::sampler_state_desc state(scm::gl::FILTER_MIN_MAG_MIP_NEAREST,
                                    scm::gl::WRAP_MIRRORED_REPEAT,
                                    scm::gl::WRAP_MIRRORED_REPEAT);

  auto tmp(layers);
  tmp.insert(tmp.begin(), std::make_pair(BufferComponent::DEPTH_24, state));

  Pass::create(ctx, tmp);
}


////////////////////////////////////////////////////////////////////////////////

void GBufferPass::cleanup(RenderContext const& ctx) {
  Pass::cleanup(ctx);
}

////////////////////////////////////////////////////////////////////////////////

bool GBufferPass::pre_compile_shaders(const gua::RenderContext& ctx) {
  bool success(true);

  for (auto const& shader : ubershaders_) {
    success &= shader.second->upload_to(ctx);
  }

  return success;
}

////////////////////////////////////////////////////////////////////////////////

void GBufferPass::print_shaders(std::string const& directory,
                             std::string const& name) const {

  for (auto const& shader : ubershaders_) {
    const std::string file_name = name + "/" + string_utils::sanitize(
          string_utils::demangle_type_name(shader.first.name()));
    shader.second->get_program()->save_to_file(directory, file_name);
  }
}

////////////////////////////////////////////////////////////////////////////////

void GBufferPass::rendering(SerializedScene const& scene,
                            SceneGraph const& graph,
                            RenderContext const& ctx,
                            CameraMode eye,
                            Camera const& camera,
                            FrameBufferObject* target,
                            View const& view) {

//    pipeline_->camera_block_left_->update(ctx.render_context, pipeline_->get_current_scene(CameraMode::LEFT).frustum);
 //   pipeline_->camera_block_right_->update(ctx.render_context, pipeline_->get_current_scene(CameraMode::RIGHT).frustum);


  if (!depth_stencil_state_ || !bfc_rasterizer_state_ ||
      !no_bfc_rasterizer_state_) {
    initialize_state_objects(ctx);
  }

  ctx.render_context->set_rasterizer_state(
      pipeline_->config.enable_backface_culling() ? bfc_rasterizer_state_
                                                  : no_bfc_rasterizer_state_);

  ctx.render_context->set_depth_stencil_state(depth_stencil_state_);

  // make sure all ubershaders are available
  update_ubershader_from_scene(ctx, scene, graph);

  // draw all drawable geometries
  for (auto const& type_ressource_pair : scene.geometrynodes_) {
    auto const& type = type_ressource_pair.first;
    auto const& ressource_container = type_ressource_pair.second;
    auto ubershader = ubershaders_.at(type);

    // set frame-consistent per-ubershader uniforms
    ubershader->set_left_resolution(pipeline_->config.get_left_resolution());
    ubershader->set_right_resolution(pipeline_->config.get_right_resolution());

    ubershader->set_material_uniforms(
        scene.materials_, ShadingModel::GBUFFER_VERTEX_STAGE, ctx);
    ubershader->set_material_uniforms(
        scene.materials_, ShadingModel::GBUFFER_FRAGMENT_STAGE, ctx);

    ubershader->set_uniform(ctx,
                            scene.enable_global_clipping_plane,
                            "gua_enable_global_clipping_plane");
    ubershader->set_uniform(
        ctx, scene.global_clipping_plane, "gua_global_clipping_plane");
    ubershader->set_uniform(ctx, false, "gua_render_shadow_map");

    for (auto const& program : ubershader->programs()) {
      Pass::bind_inputs(*program, eye, ctx);
      program->set_uniform(ctx, static_cast<int>(eye), "gua_eye");

      if (eye == CameraMode::LEFT || eye == CameraMode::CENTER) {
        ctx.render_context->bind_uniform_buffer(pipeline_->camera_block_left_->block().block_buffer(), 0);
      } else {
        ctx.render_context->bind_uniform_buffer(pipeline_->camera_block_right_->block().block_buffer(), 0);
      }
    }


    // 1. call preframe callback if available for type
    if (ubershader->get_stage_mask() & GeometryUberShader::PRE_FRAME_STAGE) {
      ubershader->preframe(ctx);
    }

    // 2. iterate all drawables of current type and call predraw of current
    // ubershader
    if (ubershader->get_stage_mask() & GeometryUberShader::PRE_DRAW_STAGE) {
      for (auto const& node : ressource_container) {
        auto const& ressource =
            GeometryDatabase::instance()->lookup(node->get_filename());
        auto const& material =
            MaterialDatabase::instance()->lookup(node->get_material());

        if (ressource && material) {
          ubershader->predraw(ctx,
                              node->get_filename(),
                              node->get_material(),
                              node->get_cached_world_transform(),
                              scm::math::transpose(scm::math::inverse(
                                  node->get_cached_world_transform())),
                              scene.frustum,
                              view);
        } else {
          if (!material) {
            Logger::LOG_WARNING
                << "GBufferPass::rendering() Cannot find material. " << material
                << std::endl;
          }
          if (!ressource) {
            Logger::LOG_WARNING
                << "GBufferPass::rendering() Cannot find geometry ressource."
                << ressource << std::endl;
          }
        }
      }
    }


    // 3. iterate all drawables of current type and call draw of current
    // ubershader
    if (ubershader->get_stage_mask() & GeometryUberShader::DRAW_STAGE) {
      for (auto const& node : ressource_container) {
        auto const& ressource =
            GeometryDatabase::instance()->lookup(node->get_filename());
        auto const& material =
            MaterialDatabase::instance()->lookup(node->get_material());

        if (ressource && material) {
          ubershader->draw(ctx,
                           node->get_filename(),
                           node->get_material(),
                           node->get_cached_world_transform(),
                           scm::math::transpose(scm::math::inverse(
                               node->get_cached_world_transform())),
                           scene.frustum,
                           view);
        } else {
          if (!material) {
            Logger::LOG_WARNING
                << "GBufferPass::rendering() Cannot find material. " << material
                << std::endl;
          }
          if (!ressource) {
            Logger::LOG_WARNING
                << "GBufferPass::rendering() Cannot find geometry ressource."
                << ressource << std::endl;

          }
        }
      }
    }

    // 4. iterate all drawables of current type and call postdraw of current
    // ubershader
    if (ubershader->get_stage_mask() & GeometryUberShader::POST_DRAW_STAGE) {
      for (auto const& node : ressource_container) {
        auto const& ressource =
            GeometryDatabase::instance()->lookup(node->get_filename());
        auto const& material =
            MaterialDatabase::instance()->lookup(node->get_material());

        if (ressource && material) {
          ubershader->postdraw(ctx,
                               node->get_filename(),
                               node->get_material(),
                               node->get_cached_world_transform(),
                               scm::math::transpose(scm::math::inverse(
                                   node->get_cached_world_transform())),
                               scene.frustum,
                               view);
        } else {
          if (!material) {
            Logger::LOG_WARNING
                << "GBufferPass::rendering() Cannot find material. " << material
                << std::endl;
          }
          if (!ressource) {
            Logger::LOG_WARNING
                << "GBufferPass::rendering() Cannot find geometry ressource."
                << ressource << std::endl;

          }
        }
      }
    }

    // 5. call postframe callback if available for type
    if (ubershader->get_stage_mask() & GeometryUberShader::POST_FRAME_STAGE) {
      ubershader->postframe(ctx);
    }
  }

  ///////////////////////////////////////////////////////////////
  // draw debug and helper information
  ///////////////////////////////////////////////////////////////
  display_quads(ctx, scene, eye, view);

  ctx.render_context->set_rasterizer_state(bbox_rasterizer_state_);
  display_bboxes(ctx, scene, view);
  display_rays(ctx, scene, view);


  ctx.render_context->reset_state_objects();
}


////////////////////////////////////////////////////////////////////////////////

void GBufferPass::display_bboxes(RenderContext const& ctx,
                                 SerializedScene const& scene,
                                 View const& view) {


  auto meshubershader = ubershaders_[typeid(node::TriMeshNode)];

  if (pipeline_->config.enable_bbox_display()) {
    meshubershader->get_program()->use(ctx);

    for (auto const& bbox : scene.bounding_boxes_) {

      auto scale(scm::math::make_scale((bbox.max - bbox.min) * 1.001f));
      auto translation(
          scm::math::make_translation((bbox.max + bbox.min) / 2.f));

      scm::math::mat4 bbox_transform;
      scm::math::set_identity(bbox_transform);

      bbox_transform *= translation;
      bbox_transform *= scale;

      meshubershader->draw(
          ctx,
          "gua_bounding_box_geometry",
          "gua_bounding_box",
          bbox_transform,
          scm::math::transpose(scm::math::inverse(bbox_transform)),
          scene.frustum,
          view);
    }
    meshubershader->get_program()->unuse(ctx);
  }
}

////////////////////////////////////////////////////////////////////////////////

void GBufferPass::display_rays(RenderContext const& ctx,
                               SerializedScene const& scene,
                               View const& view)
{
  auto meshubershader = ubershaders_[typeid(node::TriMeshNode)];

  if (pipeline_->config.enable_ray_display()) {
    meshubershader->get_program()->use(ctx);
    for (auto const& ray : scene.rays_) {
      meshubershader->get_program()->use(ctx);
      for (auto const& ray : scene.rays_) {
        meshubershader->draw(
            ctx,
            "gua_ray_geometry",
            "gua_bounding_box",
            ray->get_cached_world_transform(),
            scm::math::inverse(ray->get_cached_world_transform()),
            scene.frustum,
            view);
      }
    }
    meshubershader->get_program()->unuse(ctx);
  }
}

////////////////////////////////////////////////////////////////////////////////

void GBufferPass::display_quads(RenderContext const& ctx,
                                SerializedScene const& scene,
                                CameraMode eye,
                                View const& view)
{
  auto meshubershader = ubershaders_[typeid(node::TriMeshNode)];

  if (!scene.textured_quads_.empty()) {
    meshubershader->get_program()->use(ctx);
    {
      for (auto const& node : scene.textured_quads_) {
        std::string texture_name(node->get_texture());
        if (node->is_stereo_texture()) {

          if (eye == CameraMode::LEFT) {
            texture_name += "_left";
          } else if (eye == CameraMode::RIGHT) {
            texture_name += "_right";
          }
        }

        if (TextureDatabase::instance()->is_supported(texture_name)) {
          auto texture = TextureDatabase::instance()->lookup(texture_name);
          auto mapped_texture(
              meshubershader->get_uniform_mapping()->get_mapping(
                  "gua_textured_quad", "texture"));

          meshubershader->set_uniform(
              ctx, texture, mapped_texture.first, mapped_texture.second);

          auto mapped_flip_x(meshubershader->get_uniform_mapping()->get_mapping(
              "gua_textured_quad", "flip_x"));
          meshubershader->set_uniform(
              ctx, node->flip_x(), mapped_flip_x.first, mapped_flip_x.second);

          auto mapped_flip_y(meshubershader->get_uniform_mapping()->get_mapping(
              "gua_textured_quad", "flip_y"));
          meshubershader->set_uniform(
              ctx, node->flip_y(), mapped_flip_y.first, mapped_flip_y.second);
        }

        meshubershader->draw(
            ctx,
            "gua_plane_geometry",
            "gua_textured_quad",
            node->get_scaled_world_transform(),
            scm::math::inverse(node->get_scaled_world_transform()),
            scene.frustum,
            view);
      }
    }
    meshubershader->get_program()->unuse(ctx);
  }
}

////////////////////////////////////////////////////////////////////////////////

void GBufferPass::update_ubershader_from_scene(RenderContext const& ctx,
                                               SerializedScene const& scene,
                                               SceneGraph const& graph) {
  bool ubershader_available = true;
  for (auto const& geometry_pair : scene.geometrynodes_) {
    ubershader_available =
        ubershader_available && ubershaders_.count(geometry_pair.first);
  }

  if (!ubershader_available) {
    auto get_ubershader = [&](node::Node * n) {
      node::GeometryNode* geode = dynamic_cast<node::GeometryNode*>(n);
      if (geode) {
        std::type_index type(typeid(*geode));
        if (!ubershaders_.count(type)) {
          auto const& ressource =
              GeometryDatabase::instance()->lookup(geode->get_filename());
          if (ressource) {
            auto ubershader = ressource->create_ubershader();
            ubershader->cleanup(ctx);
            ubershader->create(cached_materials_);
            ubershaders_[type] = ubershader;
          }
        }
      }
    }
    ;
    gua::dfs_traverse(graph.get_root().get(), get_ubershader);
  }
}

////////////////////////////////////////////////////////////////////////////////

void GBufferPass::initialize_state_objects(RenderContext const& ctx) {
  if (!depth_stencil_state_)
    depth_stencil_state_ =
        ctx.render_device->create_depth_stencil_state(true, true);

  if (!bfc_rasterizer_state_)
    bfc_rasterizer_state_ = ctx.render_device->create_rasterizer_state(
        pipeline_->config.enable_wireframe() ? scm::gl::FILL_WIREFRAME
                                             : scm::gl::FILL_SOLID,
        scm::gl::CULL_BACK,
        scm::gl::ORIENT_CCW,
        false);

  if (!no_bfc_rasterizer_state_)
    no_bfc_rasterizer_state_ = ctx.render_device->create_rasterizer_state(
        pipeline_->config.enable_wireframe() ? scm::gl::FILL_WIREFRAME
                                             : scm::gl::FILL_SOLID,
        scm::gl::CULL_NONE);

  if (!bbox_rasterizer_state_)
    bbox_rasterizer_state_ = ctx.render_device
        ->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE);
}

////////////////////////////////////////////////////////////////////////////////

void GBufferPass::apply_material_mapping(
    std::set<std::string> const& materials)
{
  cached_materials_ = materials;

  for ( auto const& shader : ubershaders_ )
  {
    shader.second->create(cached_materials_);
  }

}

////////////////////////////////////////////////////////////////////////////////

LayerMapping const* GBufferPass::get_gbuffer_mapping() const {
  std::type_index trimesh_type = typeid(node::TriMeshNode);

  if (!ubershaders_.count(trimesh_type)) {
    // trimesh shader has not been created yet -> return dummy mapping
    return TriMeshRessource().create_ubershader()->get_gbuffer_mapping();
  } else {
    return ubershaders_[trimesh_type]->get_gbuffer_mapping();
  }
}

////////////////////////////////////////////////////////////////////////////////

void GBufferPass::initialize_trimesh_ubershader(RenderContext const& ctx) const
{
  std::type_index trimesh_type = typeid(node::TriMeshNode);

  if (!ubershaders_.count(trimesh_type)) {
    auto ubershader = TriMeshRessource().create_ubershader();
    ubershader->cleanup(ctx);
    ubershader->create(cached_materials_);
    ubershaders_[trimesh_type] = ubershader;
  }
}

}
