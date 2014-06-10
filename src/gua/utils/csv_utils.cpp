#include <gua/utils/csv_utils.hpp>

namespace gua {
namespace csv_utils {

Attribute::Attribute( std::string const& name
                    , std::string const& type
  ):
    name_(name)
  , type_(type)
{
}

void Attribute::set_name(std::string const& name)
{
  name_ = name;
}

void Attribute::set_type(std::string const& type)
{
  type_ = type;
}

std::string const Attribute::get_name() const
{
  return name_;
}

std::string const Attribute::get_type() const
{
  return type_;
}

std::vector<std::string> const Attribute::get_values() const
{
  return values_;
}

unsigned int Attribute::get_num_values() const
{
  return values_.size();
}

void Attribute::add_value(std::string const& value)
{
  values_.push_back(value);
}

std::string const Attribute::get_value(unsigned index) const
{
  if (index < values_.size())
    return values_[index];
  else
    return "";  // TODO handle illegal request
}


Parser::Parser( std::string const& filename
                    , std::string const& escape
                    , std::string const& quote
                    , std::string const& separator
  )
  : filename_(filename)
  , escape_(escape)
  , quote_(quote)
  , separator_(separator)
{
}

bool Parser::parse()
{
  std::ifstream file_(filename_.c_str());
  if (file_.is_open())
  {
    std::string line;
   
    // read the first two lines containing attribute information to create attribute vector
    std::getline(file_, line);
    std::vector<std::string> attrib_names = parse_line(line);
    std::getline(file_, line);
    std::vector<std::string> attrib_types = parse_line(line);
    if (attrib_names.size() == attrib_types.size())
    {
      for (unsigned int i(0); i < attrib_names.size(); ++i)
      {
        attributes_.push_back(Attribute(attrib_names[i], attrib_types[i]));
      }
    } else
    {
      std::cout << "Error: Illegal header: Amount of attribute names and attribute types not equal!" << std::endl;
      return false;
    }

    // read all other lines containing the actual csv-data and store them
    while (std::getline(file_, line))
    {
      std::vector<std::string> linedata = parse_line(line);
      if (linedata.size() == attributes_.size())
      {
        for (unsigned int i(0); i < linedata.size(); ++i)
          attributes_[i].add_value(linedata[i]);
      } else
        std::cout << "Error: Line has illegal amount of values:" << std::endl << "  '" << line << "'" << std::endl;
    }

    return true;
  } else
  {
    std::cout << "Error: File '" << filename_ << "' is not open!" << std::endl;
    return false;
  }
}

std::vector<std::string> Parser::parse_line(std::string const& line) const
{
  boost::tokenizer< boost::escaped_list_separator<char> , std::string::const_iterator, std::string> tokenizer(line, boost::escaped_list_separator<char>(escape_, separator_, quote_));
  std::vector<std::string> result;
  
  for (auto item: tokenizer)
  {
    boost::algorithm::trim(item); // strip whitespace
    result.push_back(item);
  }

  return result;
}

unsigned int Parser::get_num_data_records() const
{
  if (attributes_.empty())
    return 0;
  else
    return attributes_[0].get_num_values();
}

void Parser::print_data() const
{
  if (attributes_.empty())
    return;

  for (unsigned int datarecord_idx(0); datarecord_idx < attributes_[0].get_num_values(); ++datarecord_idx)
  {
    for (unsigned int attrib_idx(0); attrib_idx < attributes_.size(); ++attrib_idx)
    {
      std::cout << attributes_[attrib_idx].get_name() << ": " << attributes_[attrib_idx].get_value(datarecord_idx) << std::endl;
    }
    std::cout << std::endl;
  }
}



} // gua namespace
} // csv_utils namespace