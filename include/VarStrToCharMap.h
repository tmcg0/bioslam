// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// VarStrToCharMap holds a map of string/character variable identifiers to their associated GTSAM keys

#pragma once

#include <string>
#include <map>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

class VarStrToCharMap{
public:
    static unsigned char insert(const std::string& newVariableString,bool printKeyAlreadyIncluded=true);
    static void print();
    static void print_graph(const gtsam::NonlinearFactorGraph& graph); // prints graph in context of these variables
    static bool isKey(const std::string& key);
    static uint getSize();
    static unsigned char getChar(const std::string& key); // return the char associated with a string key
    static std::string find_variable_by_char(unsigned char c);
    static void clear();
private:
    static std::map<std::string, unsigned char> my_map;
    VarStrToCharMap(){}; // disallow construction
};

