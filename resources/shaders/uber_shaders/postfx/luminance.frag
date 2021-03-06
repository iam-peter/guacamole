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

uniform uvec2 gua_in_texture;

in vec2 gua_quad_coords;

// write outputs
layout(location=0) out float gua_out_color;

@include "shaders/uber_shaders/common/get_sampler_casts.glsl"

// God Rays
void main() {
    // only sample center of image
    vec3 color = texture2D( gua_get_float_sampler(gua_in_texture), gua_quad_coords*0.75 + 0.125).rgb;
    gua_out_color = dot(vec3(0.2126, 0.7152, 0.0722), color);
}
