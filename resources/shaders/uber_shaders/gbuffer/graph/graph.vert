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

@include "shaders/common/header.glsl"

// input -----------------------------------------------------------------------
layout(location=0) in vec3 gua_in_position;
layout(location=1) in vec3 gua_in_normal;
layout(location=2) in vec3 gua_in_color;

// uniforms
@include "shaders/uber_shaders/common/gua_camera_uniforms.glsl"

uniform uint gua_material_id;
uniform bool gua_render_shadow_map;

// 1: LOW_QUALITY
// 2: HIGH_QUALITY
uniform int  gua_shadow_quality;

// material specific uniforms
@uniform_definition

// outputs ---------------------------------------------------------------------
out vec3 gua_position_varying;
out vec3 gua_object_color;

@output_definition

// global variables
vec2 gua_texcoords;

vec3 gua_world_normal;
vec3 gua_world_position;

vec3 gua_object_normal;
vec3 gua_object_position;

// methods ---------------------------------------------------------------------

// global gua_* methods
@include "shaders/uber_shaders/common/get_sampler_casts.glsl"

uint gua_get_material_id() {
  return gua_material_id;
}

// material specific methods
@material_methods

// main ------------------------------------------------------------------------
void main() {

  if (gua_render_shadow_map && gua_shadow_quality == 1) {

    // perform only material independent projection
    gl_Position = gua_projection_matrix * gua_view_matrix
                 * gua_model_matrix
                 * vec4(gua_in_position, 1.0);

  } else {
    gua_position_varying = vec3(0);

    gua_object_normal    = gua_in_normal;
    gua_object_position  = gua_in_position;
    gua_object_color     = gua_in_color;

    gua_world_normal =      normalize((gua_normal_matrix * vec4(gua_in_normal, 0.0)).xyz);
    gua_world_position =    (gua_model_matrix * vec4(gua_in_position, 1.0)).xyz;

    // big switch, one case for each material
    @material_switch

    gua_uint_gbuffer_varying_0.x = gua_material_id;
    gl_Position = gua_projection_matrix * gua_view_matrix * vec4(gua_position_varying.xyz, 1.0);
  }
}