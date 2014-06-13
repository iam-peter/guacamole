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

#include <gua/utils/Logger.hpp>
#include <gua/utils/DataColumn.hpp>
#include <gua/utils/DataColumnFloat.hpp>
#include <gua/utils/DataColumnInt.hpp>
#include <gua/utils/DataColumnString.hpp>
#include <gua/utils/string_utils.hpp>

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>

namespace gua {
namespace utils {

class DataSet
{
public:
	DataSet();

	bool load_from_csv(
			std::string const& filename
		, std::string const& separator = ","
		, std::string const& escape = "\\"
    , std::string const& quote = "\""
	);

	void print_norm_data(std::ostream& stream);

private:
	std::vector<std::string> parse_csv_line(
			std::string const& line
		, std::string const& separator = ","
		, std::string const& escape = "\\"
		, std::string const& quote = "\""
	);

private:
	std::string 	labels_keyword_;
	std::string 	types_keyword_;

	std::vector<std::shared_ptr<DataColumn>> columns_;
};

}	// utils namespace
}	// gua namespace

#endif // GUA_DATASET_HPP