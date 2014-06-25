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
#include <gua/utils/DataColumnInt.hpp>

namespace gua {
namespace utils {

DataColumnInt::DataColumnInt(std::string const& label):
	  DataColumn(label, INT)
	, min_(std::numeric_limits<float>::max())
	, max_(std::numeric_limits<float>::min())
{
}

/*virtual*/ DataColumnInt::~DataColumnInt()
{}

/*virtual*/ void DataColumnInt::normalize()
{
	norm_values_.clear();
	if (!values_.empty())
	{
		float range(max_ - min_);
		for (int value: values_) {
			norm_values_.push_back((value - min_) / range);
		}
	}
}

/*virtual*/ bool DataColumnInt::add_string_value(std::string const& str)
{
	add_value(gua::string_utils::from_string<int>(str));
}

void DataColumnInt::add_value(int value)
{
	if (value > max_) max_ = value;
	if (value < min_) min_ = value;
	values_.push_back(value);
}

void DataColumnInt::add_values(std::vector<int> const& values)
{
	for (int value: values)
	{
		if (value > max_) max_ = value;
		if (value < min_) min_ = value;
	}
	values_.insert(values_.end(), values.begin(), values.end());
}

}	// utils namespace
}	// gua namespace