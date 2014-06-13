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
#include <gua/utils/DataColumnString.hpp>

namespace gua {
namespace utils {

DataColumnString::DataColumnString(std::string const& label):
	DataColumn(label, STRING)
{
}

/*virtual*/ DataColumnString::~DataColumnString()
{}

/*virtual*/ void DataColumnString::normalize()
{
	norm_values_.clear();
	if (!values_.empty())
	{
		float range(values_.size() - 1.0f);
		for (std::string value: values_)
		{
			// find string in our set; norm value is the index normalized to set size
			auto it = values_.find(value);
			norm_values_.push_back(std::distance(values_.begin(), it) / range);
		}
	}
}

/*virtual*/ bool DataColumnString::add_string_value(std::string const& str)
{
	add_value(str);
}

void DataColumnString::add_value(std::string const& value)
{
	values_.insert(value);
}

void DataColumnString::add_values(std::vector<std::string> const& values)
{
	std::copy(values.begin(), values.end(), std::inserter(values_, values_.end()));
}

}	// utils namespace
}	// gua namespace
