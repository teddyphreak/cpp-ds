//============================================================================
// Name        : ds.cpp
// Author      : Alejandro Cook
// Version     :
// Copyright   : 
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include "ds_common.h"
#include "ds_lg.h"
#include "ds_library.h"
#include "ds_lg.h"
#include "boost/spirit/include/qi.hpp"
using namespace std;
using namespace ds_library;

int main() {
	cout << "Hello  1World" << endl; // prints Hello World

	LibraryFactory *factory = LibraryFactory::getInstance();
	cout << "Hello  2World" << endl;
	std::string path = "default_lib_error";
	Library *defaultLib = factory->loadLibrary(path);
	//Library *defaultLib = factory->loadLibrary();
	cout << "Hello  3World" << endl;
	//	BOOST_REQUIRE(defaultLib !=0);
	factory->remove_library();
	delete factory;
	return 0;
}
