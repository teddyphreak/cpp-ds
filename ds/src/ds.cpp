//============================================================================
// Name        : ds.cpp
// Author      : Alejandro Cook
// Version     :
// Copyright   : 
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include "ds_common.h"
#include "ds_library.h"
#include "ds_lg.h"
using namespace std;

int main() {
	cout << "Hello  1World" << endl; // prints Hello World
	string file = "../p45k.v";
	string top ="top";
	ds_library::import(file,top);
	return 0;
}
