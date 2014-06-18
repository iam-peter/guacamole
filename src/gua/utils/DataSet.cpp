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
#include <gua/utils/DataSet.hpp>

namespace gua {
namespace utils {

DataSet::DataSet():
		labels_keyword_("LABELS:")
	, types_keyword_("TYPES:")
	, columns_()
{
}

bool DataSet::load_from_csv(
			std::string const& filename
		, std::string const& separator
		, std::string const& escape
    , std::string const& quote
)
{
	std::ifstream file(filename.c_str(), std::ifstream::in);
	if (file.is_open())
	{
		// read all lines into this vector
		std::vector<std::string> lines;
		std::string crrntline;
		while (std::getline(file, crrntline))
		{
			lines.push_back(crrntline);
		}
		file.close();

		// find header data (labels and data types)
		std::vector<std::string> labels;
		std::vector<std::string> types;

		Logger::LOG_MESSAGE << "parsing header..." << std::endl;
		for (std::string line: lines)
		{
			if (line.find(labels_keyword_) != std::string::npos)
			{
				gua::string_utils::replace(line, labels_keyword_, "");		// remove keyword
				labels = parse_csv_line(line, escape, quote, separator);	// extract labels list
				Logger::LOG_MESSAGE << " found labels: " << gua::string_utils::join(labels) << std::endl;
			} else if (line.find(types_keyword_) != std::string::npos)
			{
				gua::string_utils::replace(line, types_keyword_, "");		// remove keyword
				types = parse_csv_line(line, escape, quote, separator);		// extract types list
				Logger::LOG_MESSAGE << " found types: " << gua::string_utils::join(types) << std::endl;
			}
		}
		Logger::LOG_MESSAGE << "parsing header... done" << std::endl;

		// create column classes according to the types extracted above
		for (unsigned int i(0); i < labels.size(); ++i)
		{
			if (types[i] == "FLOAT")
			{
				columns_.push_back(std::make_shared<DataColumnFloat>(labels[i]));
			} else if (types[i] == "INT")
			{
				columns_.push_back(std::make_shared<DataColumnInt>(labels[i]));
			} else if (types[i] == "STRING")
			{
				columns_.push_back(std::make_shared<DataColumnString>(labels[i]));
			} else {
				Logger::LOG_ERROR << "cannot handle type '" << types[i] << "'!" << std::endl;
			}
		}

		Logger::LOG_MESSAGE << "parsing values..." << std::endl;
		int counter(0);
		for (std::string line: lines)
		{
			boost::algorithm::trim(line);
			if (line.empty()) 			continue;		// empty line, skip it
			if (line.at(0) == '#')	continue;		// comment, skip it
			if ((line.find(labels_keyword_) != std::string::npos) || (line.find(types_keyword_) != std::string::npos))
				continue;		// header line, skip it

			std::vector<std::string> linedata = parse_csv_line(line, escape, quote, separator);
			for (unsigned int i(0); i < linedata.size(); ++i)
			{
				columns_[i]->add_string_value(linedata[i]);
			}
			++counter;
		}
		Logger::LOG_MESSAGE << "parsing of " << counter << " data records done" << std::endl;

		for (std::shared_ptr<DataColumn> col: columns_)
			col->normalize();

		return true;

	} else
	{
		Logger::LOG_ERROR << "unable to open csv-file '" << filename << "'!" << std::endl;
		return false;
	}
}

std::vector<std::string> DataSet::parse_csv_line(
			std::string const& line
		, std::string const& escape
		, std::string const& quote
		, std::string const& separator
)
{
	boost::tokenizer< boost::escaped_list_separator<char> , std::string::const_iterator, std::string> tokenizer(line, boost::escaped_list_separator<char>(escape, separator, quote));
  std::vector<std::string> result;
  for (auto item: tokenizer)
  {
    boost::algorithm::trim(item); // strip whitespace
    result.push_back(item);
  }
  return result;
};

void DataSet::print_norm_data(std::ostream& stream)
{
	stream << "printing data records:" << std::endl;
	for (std::shared_ptr<DataColumn> col: columns_)
	{
		stream << col->get_label() << ": ";
		for (float val: col->get_norm_values())
			stream << val << " ";
		stream << std::endl;
	}
}

std::shared_ptr<DataColumn> DataSet::get_column_by_name(std::string const& colname) const
{
	for (std::shared_ptr<DataColumn> col: columns_)
		if (col->get_label() == colname)
			return col;

	Logger::LOG_ERROR << "requested column with label '" << colname << "' not found!" << std::endl;
	return nullptr;
}

}	// utils namespace
}	// gua namespace
