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
#include <gua/utils/DataColumn.hpp>

namespace gua {
namespace utils {

DataColumn::DataColumn(std::string const& label, DataType const& data_type):
	label_(label)
, data_type_(data_type)
, norm_values_()
{
}

/*virtual*/ DataColumn::~DataColumn()
{}

std::string const DataColumn::get_label() const
{
	return label_;
}

DataType const DataColumn::get_data_type() const
{
	return data_type_;
}

std::vector<float> const& DataColumn::get_norm_values() const
{
	return norm_values_;
}

unsigned int DataColumn::get_num_values() const
{
	return norm_values_.size();
}


}	// utils namespace
}	// gua namespace
