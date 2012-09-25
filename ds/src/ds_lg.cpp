/*
 * ds_lg.cpp

 *
 *  Created on: Aug 29, 2012
 *      Author: cookao
 */

#include "ds_common.h"
#include "ds_library.h"
#include "ds_lg.h"

namespace ds_lg {

	int64* LGNode1I::getBinding(const char& name) {
		switch(name){
		case 'a': return a;
		case 'o': return &o;
		default: return 0;
		}
	}

	int64* LGNode2I::getBinding(const char& name) {
		switch(name){
		case 'a': return a;
		case 'b': return b;
		case 'o': return &o;
		default: return 0;
		}
	}

	int64* LGNode3I::getBinding(const char& name) {
		switch(name){
		case 'a': return a;
		case 'b': return b;
		case 'c': return c;
		case 'o': return &o;
		default: return 0;
		}
	}

	int64* LGNode4I::getBinding(const char& name) {
		switch(name){
		case 'a': return a;
		case 'b': return b;
		case 'c': return c;
		case 'd': return d;
		case 'o': return &o;
		default: return 0;
		}
	}

	int64* LGNode5I::getBinding(const char& name) {
		switch(name){
		case 'a': return a;
		case 'b': return b;
		case 'c': return c;
		case 'd': return d;
		case 'e': return e;
		case 'o': return &o;
		default: return 0;
		}
	}

	int64* LGNode6I::getBinding(const char& name) {
		switch(name){
		case 'a': return a;
		case 'b': return b;
		case 'c': return c;
		case 'd': return d;
		case 'e': return e;
		case 'f': return f;
		case 'o': return &o;
		default: return 0;
		}
	}

	int64* LGNode7I::getBinding(const char& name) {
		switch(name){
		case 'a': return a;
		case 'b': return b;
		case 'c': return c;
		case 'd': return d;
		case 'e': return e;
		case 'f': return f;
		case 'g': return g;
		case 'o': return &o;
		default: return 0;
		}
	}

	int64* LGNode8I::getBinding(const char& name) {
		switch(name){
		case 'a': return a;
		case 'b': return b;
		case 'c': return c;
		case 'd': return d;
		case 'e': return e;
		case 'f': return f;
		case 'g': return g;
		case 'h': return h;
		case 'o': return &o;
		default: return 0;
		}
	}

	int64* LGNodeArr::getBinding(const char& name) {

		std::size_t disp = name - 'A';
		int64** p = input_array;
		for (std::size_t i=0;i<disp;i++)
			p++;
		return *p;

	}
}

