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

#ifndef GUA_DATASET_HPP
#define GUA_DATASET_HPP

#include <gua/utils/DataColumn.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace gua {
namespace utils {

class DataSet
{
public:
	DataSet();

	bool load_from_csv(std::string const& filename);

	void print_data_records(std::ostream& stream);

private:
	std::vector< std::shared_ptr<DataColumn> > columns_;
};

}	// utils namespace
}	// gua namespace

#endif // GUA_DATASET_HPP