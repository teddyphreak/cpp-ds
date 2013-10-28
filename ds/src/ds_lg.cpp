/*
 * ds_lg.cpp

 *
 *  Created on: Aug 29, 2012
 *      Author: cookao
 */

#include "ds_common.h"
#include "ds_library.h"
#include "ds_lg.h"
#include "ds_pattern.h"
#include "boost/multi_array.hpp"

namespace ds_lg {

	lg_v64** LGNode1I::get_binding(const std::string& name) {
		if (name == "a")return &a;
		return 0;
	}

	lg_v64** LGNode2I::get_binding(const std::string& name) {
		if (name == "a")return &a;
		if (name == "b")return &b;
		return 0;
	}

	lg_v64** LGNode3I::get_binding(const std::string& name) {
		if (name == "a")return &a;
		if (name == "b")return &b;
		if (name == "c")return &c;
		return 0;
	}

	lg_v64** LGNode4I::get_binding(const std::string& name) {
		if (name == "a")return &a;
		if (name == "b")return &b;
		if (name == "c")return &c;
		if (name == "d")return &d;
		return 0;
	}

	lg_v64** LGNode5I::get_binding(const std::string& name) {
		if (name == "a")return &a;
		if (name == "b")return &b;
		if (name == "c")return &c;
		if (name == "d")return &d;
		if (name == "e")return &e;
		return 0;
	}

	lg_v64** LGNode6I::get_binding(const std::string& name) {
		if (name == "a")return &a;
		if (name == "b")return &b;
		if (name == "c")return &c;
		if (name == "d")return &d;
		if (name == "e")return &e;
		if (name == "f")return &f;
		return 0;
	}

	lg_v64** LGNode7I::get_binding(const std::string& name) {
		if (name == "a")return &a;
		if (name == "b")return &b;
		if (name == "c")return &c;
		if (name == "d")return &d;
		if (name == "e")return &e;
		if (name == "f")return &f;
		if (name == "g")return &g;
		return 0;
	}

	lg_v64** LGNode8I::get_binding(const std::string& name) {
		if (name == "a")return &a;
		if (name == "b")return &b;
		if (name == "c")return &c;
		if (name == "d")return &d;
		if (name == "e")return &e;
		if (name == "f")return &f;
		if (name == "g")return &g;
		if (name == "h")return &g;
		return 0;
	}

	lg_v64** LGState::get_binding(const std::string& name) {
		if (name == "D")return &d;
		if (name == "CD")return &cd;
		return 0;
	}

	lg_v64** LGNodeArr::get_binding(const std::string& name) {

		char c = name[0];
		std::size_t disp = c - 'a';
		lg_v64** p = input_array;
		for (std::size_t i=0;i<disp;i++)
			p++;
		return p;

	}

	bool LeveledGraph::sanity_check(){
		bool c = true;
		if (nodes.size() <= 0){
			c = false;
			std::cout << "no nodes found!"<< std::endl;
		}
		for (LGNode *n: nodes)
		{
			if (n->level > num_levels){
				c = false;
				std::cout << "> max level"<< std::endl;
			}

			for (LGNode *o: n->outputs)
			{
				if (!o->has_state())
					if (n->level >= o->level){
						c = false;
						std::cout << "no state and wrong"<< std::endl;
						std::cout << n->get_gate()->get_instance_name() << ":" << n->level << "  " << o->get_gate()->get_instance_name() << ":" << o->level<<std::endl;
					}
			}
		}

		for (int i=0;i<num_levels;i++){
			unsigned int cnt = 0;
			std::for_each(nodes.begin(), nodes.end(),
				[&] (LGNode* n) { if (n->level == i) cnt++;}
			);
			if (cnt != level_width[i]){
				c = false;
				std::cout << "Inconsistent level (" << i <<") size: "<< cnt << "!=" << level_width[i] << std::endl;
			}
		}

		return c;
	}


	void LeveledGraph::adapt(const ds_pattern::CombinationalPatternAdapter* adapter){

		for (Input* in:inputs){
			in->set_pattern_block(&pattern_block);
			std::string name = in->get_name();
			std::string port_name = name.substr(name.find('/') + 1);
			std::size_t offset = adapter->get_offset(port_name);
			in->set_offset(offset);
		}

		for (Output* out:outputs){
			std::string name = out->get_name();
			std::string port_name = name.substr(name.find('/') + 1);
			std::size_t offset = adapter->get_offset(port_name);
			OutputObserver* observer= new OutputObserver(offset, &pattern_block);
			out->add_monitor(observer);
		}
	}


	void LeveledGraph::sim(ds_pattern::SimPatternBlock *pb){

		pattern_block = pb;
		for (auto it=nodes.begin();it!=nodes.end();it++){
			LGNode *n = *it;
			n->propagate();
		}
	}
}
