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
#include <gua/utils/DataColumnFloat.hpp>

namespace gua {
namespace utils {

DataColumnFloat::DataColumnFloat(std::string const& label, DataType const& data_type):
	DataColumn(label, data_type)
{
}

/*virtual*/ DataColumnFloat::~DataColumnFloat()
{}

/*virtual*/ void DataColumnFloat::normalize()
{
	norm_values_.clear();
	if (!values_.empty())
	{
		float min(std::numeric_limits<float>::max());
		float max(std::numeric_limits<float>::min());

		for (float f: values_) {
			if (f < min) min = f;
			if (f > max) max = f;
		}

		float range(max - min);
		for (float f: values_) {
			norm_values_.push_back((f - min) / range);
		}
	}
}

void DataColumnFloat::add_value(float value)
{
	values_.push_back(value);
}

void DataColumnFloat::add_values(std::vector<float> const& values)
{
	values_.insert(values_.end(), values.begin(), values.end());
}

}	// utils namespace
}	// gua namespace
