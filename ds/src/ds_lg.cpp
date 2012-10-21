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

	int64** LGNode1I::get_binding(const char& name) {
		switch(name){
		case 'a': return &a;
		default: return 0;
		}
	}

	int64** LGNode2I::get_binding(const char& name) {
		switch(name){
		case 'a': return &a;
		case 'b': return &b;
		default: return 0;
		}
	}

	int64** LGNode3I::get_binding(const char& name) {
		switch(name){
		case 'a': return &a;
		case 'b': return &b;
		case 'c': return &c;
		default: return 0;
		}
	}

	int64** LGNode4I::get_binding(const char& name) {
		switch(name){
		case 'a': return &a;
		case 'b': return &b;
		case 'c': return &c;
		case 'd': return &d;
		default: return 0;
		}
	}

	int64** LGNode5I::get_binding(const char& name) {
		switch(name){
		case 'a': return &a;
		case 'b': return &b;
		case 'c': return &c;
		case 'd': return &d;
		case 'e': return &e;
		default: return 0;
		}
	}

	int64** LGNode6I::get_binding(const char& name) {
		switch(name){
		case 'a': return &a;
		case 'b': return &b;
		case 'c': return &c;
		case 'd': return &d;
		case 'e': return &e;
		case 'f': return &f;
		default: return 0;
		}
	}

	int64** LGNode7I::get_binding(const char& name) {
		switch(name){
		case 'a': return &a;
		case 'b': return &b;
		case 'c': return &c;
		case 'd': return &d;
		case 'e': return &e;
		case 'f': return &f;
		case 'g': return &g;
		default: return 0;
		}
	}

	int64** LGNode8I::get_binding(const char& name) {
		switch(name){
		case 'a': return &a;
		case 'b': return &b;
		case 'c': return &c;
		case 'd': return &d;
		case 'e': return &e;
		case 'f': return &f;
		case 'g': return &g;
		case 'h': return &h;
		default: return 0;
		}
	}

	int64** LGNodeArr::get_binding(const char& name) {

		std::size_t disp = name - 'a';
		int64** p = input_array;
		for (std::size_t i=0;i<disp;i++)
			p++;
		return p;

	}

	bool LeveledGraph::sanity_check(){
		bool c = true;
		if (nodes.size() <= 0)
			c = false;
		BOOST_FOREACH(LGNode *n, nodes)
		{
			if (n->level > max_level)
				c = false;

			BOOST_FOREACH(LGNode *o, n->outputs)
			{
				if (n->level >= o->level){
					c = false;
				}
			}
		}
		return c;
	}
}
