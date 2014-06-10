#ifndef GUA_CSV_UTILS_HPP
#define GUA_CSV_UTILS_HPP

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>

namespace gua {
namespace csv_utils {

class Attribute
{
public:
  Attribute(std::string const& name, std::string const& type);

  void set_name(std::string const& name);
  void set_type(std::string const& type);

  std::string const get_name() const;
  std::string const get_type() const;
  std::vector<std::string> const get_values() const;
  
  unsigned int get_num_values() const;

  void add_value(std::string const& value);
  std::string const get_value(unsigned index) const;

private:
  std::string               name_;
  std::string               type_;
  std::vector<std::string>  values_;
};

class Parser
{
public:
  Parser(
      std::string const& filename
    , std::string const& escape = "\\"
    , std::string const& quote = "\""
    , std::string const& separator = ","
  );

  bool parse();
  void print_data() const;

private:
  std::string   filename_;
  std::string   escape_;
  std::string   quote_;
  std::string   separator_;
  std::ifstream file_;

  std::vector<std::string>  header_;
  std::vector<std::string>  types_;
  std::vector<Attribute> attributes_;
  /* std::vector<std::vector<std::string>> data_; */

private:
  std::vector<std::string> parse_line(std::string const& line) const;
  unsigned int get_num_data_records() const;
};

} // gua namespace
} // csv_utils namespace

#endif  // GUA_CSV_UTILS_HPP