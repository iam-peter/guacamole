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

#ifndef GUA_DATACOLUMNFLOAT_HPP
#define GUA_DATACOLUMNFLOAT_HPP

#include <limits>
#include <string>
#include <vector>

#include <gua/utils/string_utils.hpp>
#include <gua/utils/DataColumn.hpp>

namespace gua {
namespace utils {

class DataColumnFloat : public DataColumn
{
public:
								DataColumnFloat(std::string const& label);
	virtual 			~DataColumnFloat();

	virtual void 	normalize();

	virtual bool 	add_string_value(std::string const& str);

	void 					add_value(float value);
	void 					add_values(std::vector<float> const& values);

protected:
	float 							min_;
	float 							max_;
	std::vector<float>	values_;
};



}	// utils namespace
}	// gua namespace

#endif // GUA_DATACOLUMNFLOAT_HPP