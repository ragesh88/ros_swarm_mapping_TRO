//
// Created by root on 5/5/18.
//

#ifndef MAP_SHARING_INFO_BASED_EXPLORATION_COMMAND_LINE_PARSER_H
#define MAP_SHARING_INFO_BASED_EXPLORATION_COMMAND_LINE_PARSER_H

////////////////////////////////////////////////////////////////////////////////
// My naming conventions for the project
// Put header files separated different category and in alphabetical order
// unless there some compatibility issue between the header files
// macros and const : ALL_IN_CAPS
// namespace : NS_prefix_small_letter_with_underscore
// class and structs : First_letter_caps_with_underscore
// functions and variables: all_small_letters
/////////////////////////////////////////////////////////////////////////////////

/**
 * This is the header for a command line parser
 */

#include<iostream>
#include <string>


namespace NS_rag_command_line_parser{


class CommandLineParser {

 private:
  int argc;
  char **argv;

 public:
  // Constructor
  CommandLineParser(int _argc, char **_argv);

  // Modules
  bool operator[] (std::string param);
  std::string operator() (std::string param, std::string def_value="-1");

};

}

#endif //MAP_SHARING_INFO_BASED_EXPLORATION_COMMAND_LINE_PARSER_H
