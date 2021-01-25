// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

#include <gtsam/inference/Symbol.h>
#include "VarStrToCharMap.h"

std::map<std::string,unsigned char> VarStrToCharMap::my_map;

unsigned char getCharByInt(uint i);

unsigned char VarStrToCharMap::insert(const std::string& newVariableString, bool printKeyAlreadyIncluded){
    if(isKey(newVariableString)){
        if (printKeyAlreadyIncluded){
            std::cout << "ERROR: already is a key named " << newVariableString << std::endl;
        }
        return getChar(newVariableString);
    }else{
        uint nVariablesNow = getSize();
        unsigned char charToMapTo = getCharByInt(nVariablesNow);
        my_map.insert(std::pair<std::string, unsigned char>(newVariableString, charToMapTo));
        return charToMapTo;
    }
};

void VarStrToCharMap::print(){
    std::cout<<"VarStrToCharMap ("<<getSize()<<") members:"<<std::endl;
    for(std::map<std::string,unsigned char>::const_iterator it=my_map.begin();it!=my_map.end();++it){
        std::cout<<"    Variable: '"<<it->first<<"'  -->  char: '"<<it->second<<"'"<<std::endl;
    }
};

void VarStrToCharMap::print_graph(const gtsam::NonlinearFactorGraph& graph){
    std::map<unsigned char,uint> charCountMap;
    gtsam::FastSet<gtsam::Key> keySet=graph.keys();
    for(std::set<unsigned long>::iterator it=keySet.begin();it!=keySet.end();++it){
        //cout<<"Key @ idx="<<symbolIndex(*it)<<" - "<<symbolChr(*it)<<" ("<<*it<<")"<<endl;
        if(charCountMap.count(gtsam::symbolChr(*it))!=0){ // key exists
            charCountMap[gtsam::symbolChr(*it)]=charCountMap[gtsam::symbolChr(*it)]+1;
        }else{ // key does not exist--add it
            charCountMap.insert(std::pair<unsigned char,uint>(gtsam::symbolChr(*it),1));
        }
    }
    // now print graph stats
    std::cout<<"in graph there are the following variables:"<<std::endl;
    for (auto const& x : charCountMap){
        std::cout << "   "<<x.second<<" of "<<find_variable_by_char(x.first)<<"   (char "<< x.first<<")" << std::endl ;
    }
}
std::string VarStrToCharMap::find_variable_by_char(unsigned char c){
    // loop over map to do linear search to find variable name
    for(auto const& x: my_map){
        if(x.second==c){ // found it!
            return x.first;
        }
    }
    std::cerr<<"ERROR: char not found"<<std::endl;
    return std::string("");
}

void VarStrToCharMap::clear(){
    my_map.clear();
}

bool VarStrToCharMap::isKey(const std::string& key){ // does this key exist in the map?
    return (my_map.count(key) > 0);
}

uint VarStrToCharMap::getSize(){ // return size of m_map
    return my_map.size();
};

unsigned char VarStrToCharMap::getChar(const std::string& key){
    return my_map[key];
}

// ========= helper functions ===========

unsigned char getCharByInt(uint i){
    std::string alphabets = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
    return alphabets[i];
}