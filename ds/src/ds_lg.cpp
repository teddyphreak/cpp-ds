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
#include "ds_faults.h"
#include "boost/multi_array.hpp"
#include <boost/log/trivial.hpp>

namespace ds_lg {

	void LGNode::hook_inputs(){
		for (ds_faults::SimulationHook *h : hooks){
			lg_v64** input_address = get_input(h->get_hook_port());
			if (input_address!=0){
				lg_v64* p_port = *input_address;
				if (p_port!=0){
					int64 activation = h->hook(lg);
					p_port->v ^= activation & ~p_port->x;
				}
			}
		}
	}

	void LGNode::hook_outputs(){
		for (ds_faults::SimulationHook *h : hooks){
			lg_v64* p_port = get_output(h->get_hook_port());
			if (p_port!=0){
				int64 activation = h->hook(lg);
				p_port->v ^= activation & ~p_port->x;
			}
		}
	}

	lg_v64** LGNode1I::get_input(const std::string& name) {
		if (name == "a")return &a;
		return 0;
	}

	lg_v64** LGNode2I::get_input(const std::string& name) {
		if (name == "a")return &a;
		if (name == "b")return &b;
		return 0;
	}

	lg_v64** LGNode3I::get_input(const std::string& name) {
		if (name == "a")return &a;
		if (name == "b")return &b;
		if (name == "c")return &c;
		return 0;
	}

	lg_v64** LGNode4I::get_input(const std::string& name) {
		if (name == "a")return &a;
		if (name == "b")return &b;
		if (name == "c")return &c;
		if (name == "d")return &d;
		return 0;
	}

	lg_v64** LGNode5I::get_input(const std::string& name) {
		if (name == "a")return &a;
		if (name == "b")return &b;
		if (name == "c")return &c;
		if (name == "d")return &d;
		if (name == "e")return &e;
		return 0;
	}

	lg_v64** LGNode6I::get_input(const std::string& name) {
		if (name == "a")return &a;
		if (name == "b")return &b;
		if (name == "c")return &c;
		if (name == "d")return &d;
		if (name == "e")return &e;
		if (name == "f")return &f;
		return 0;
	}

	lg_v64** LGNode7I::get_input(const std::string& name) {
		if (name == "a")return &a;
		if (name == "b")return &b;
		if (name == "c")return &c;
		if (name == "d")return &d;
		if (name == "e")return &e;
		if (name == "f")return &f;
		if (name == "g")return &g;
		return 0;
	}

	lg_v64** LGNode8I::get_input(const std::string& name) {
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

	lg_v64** LGState::get_input(const std::string& name) {
		if (name == "d")return &d;
		if (name == "cd")return &cd;
		if (name == "rst")return &rst;
		if (name == "rst_n")return &rst_n;
		if (name == "load")return &load;
		if (name == "load_n")return &load_n;
		return 0;
	}

	lg_v64** LGNodeArr::get_input(const std::string& name) {

		char c = name[0];
		int disp = c - 'a';
		if (disp>=input_size)
			return 0;
		lg_v64** p = input_array;
		for (int i=0;i<disp;i++)
			p++;
		return p;
	}

	void Output::hook() {
		sim();
		for (ds_faults::SimulationHook *h : hooks){
			o.v ^=  h->hook(lg) & ~o.x;
		}
	}

	void Input::hook() {
		sim();
		for (ds_faults::SimulationHook *h : hooks){
			o.v ^=  h->hook(lg) & ~o.x;
		}
	}

	void LGNode1I::hook() {
		lg_v64 *sa = a;
		lg_v64 va = *a;
		a = &va;
		hook_inputs();
		sim();
		hook_outputs();
		a = sa;
	}

	void LGNode2I::hook() {
		lg_v64 *sa = a;
		lg_v64 va = *a;
		a = &va;
		lg_v64 *sb = b;
		lg_v64 vb = *b;
		b = &vb;
		hook_inputs();
		sim();
		hook_outputs();
		a = sa;
		b = sb;
	}

	void LGNode3I::hook() {
		lg_v64 *sa = a;
		lg_v64 va = *a;
		a = &va;
		lg_v64 *sb = b;
		lg_v64 vb = *b;
		b = &vb;
		lg_v64 *sc = c;
		lg_v64 vc = *c;
		c = &vc;
		hook_inputs();
		sim();
		hook_outputs();
		a = sa;
		b = sb;
		c = sc;
	}

	void LGNode4I::hook() {
		lg_v64 *sa = a;
		lg_v64 va = *a;
		a = &va;
		lg_v64 *sb = b;
		lg_v64 vb = *b;
		b = &vb;
		lg_v64 *sc = c;
		lg_v64 vc = *c;
		c = &vc;
		lg_v64 *sd = d;
		lg_v64 vd = *d;
		d = &vd;
		hook_inputs();
		sim();
		hook_outputs();
		a = sa;
		b = sb;
		c = sc;
		d = sd;
	}

	void LGNode5I::hook() {
		lg_v64 *sa = a;
		lg_v64 va = *a;
		a = &va;
		lg_v64 *sb = b;
		lg_v64 vb = *b;
		b = &vb;
		lg_v64 *sc = c;
		lg_v64 vc = *c;
		c = &vc;
		lg_v64 *sd = d;
		lg_v64 vd = *d;
		d = &vd;
		lg_v64 *se = e;
		lg_v64 ve = *e;
		e = &ve;
		hook_inputs();
		sim();
		hook_outputs();
		a = sa;
		b = sb;
		c = sc;
		d = sd;
		e = se;
	}

	void LGNode6I::hook() {
		lg_v64 *sa = a;
		lg_v64 va = *a;
		a = &va;
		lg_v64 *sb = b;
		lg_v64 vb = *b;
		b = &vb;
		lg_v64 *sc = c;
		lg_v64 vc = *c;
		c = &vc;
		lg_v64 *sd = d;
		lg_v64 vd = *d;
		d = &vd;
		lg_v64 *se = e;
		lg_v64 ve = *e;
		e = &ve;
		lg_v64 *sf = f;
		lg_v64 vf = *f;
		f = &vf;
		hook_inputs();
		sim();
		hook_outputs();
		a = sa;
		b = sb;
		c = sc;
		d = sd;
		e = se;
		f = sf;
	}

	void LGNode7I::hook() {
		lg_v64 *sa = a;
		lg_v64 va = *a;
		a = &va;
		lg_v64 *sb = b;
		lg_v64 vb = *b;
		b = &vb;
		lg_v64 *sc = c;
		lg_v64 vc = *c;
		c = &vc;
		lg_v64 *sd = d;
		lg_v64 vd = *d;
		d = &vd;
		lg_v64 *se = e;
		lg_v64 ve = *e;
		e = &ve;
		lg_v64 *sf = f;
		lg_v64 vf = *f;
		f = &vf;
		lg_v64 *sg = g;
		lg_v64 vg = *g;
		g = &vg;
		hook_inputs();
		sim();
		hook_outputs();
		a = sa;
		b = sb;
		c = sc;
		d = sd;
		e = se;
		f = sf;
		g = sg;
	}

	void LGNode8I::hook() {
		lg_v64 *sa = a;
		lg_v64 va = *a;
		a = &va;
		lg_v64 *sb = b;
		lg_v64 vb = *b;
		b = &vb;
		lg_v64 *sc = c;
		lg_v64 vc = *c;
		c = &vc;
		lg_v64 *sd = d;
		lg_v64 vd = *d;
		d = &vd;
		lg_v64 *se = e;
		lg_v64 ve = *e;
		e = &ve;
		lg_v64 *sf = f;
		lg_v64 vf = *f;
		f = &vf;
		lg_v64 *sg = g;
		lg_v64 vg = *g;
		g = &vg;
		lg_v64 *sh = h;
		lg_v64 vh = *h;
		h = &vh;
		hook_inputs();
		sim();
		hook_outputs();
		a = sa;
		b = sb;
		c = sc;
		d = sd;
		e = se;
		f = sf;
		g = sg;
		h = sh;
	}

	void LGNodeArr::hook(){
		lg_v64** si = new lg_v64*[input_size];
		lg_v64* vi = new lg_v64[input_size];
		for (int i=0;i<input_size;i++){
			si[i] = input_array[i];
			vi[i] = *input_array[i];
			input_array[i] = &vi[i];
		}
		hook_inputs();
		sim();
		hook_outputs();
		for (int i=0;i<input_size;i++){
			input_array[i] = si[i];
		}
		delete[] si;
		delete [] vi;
	}

	bool LeveledGraph::sanity_check(){
		bool c = true;
		if (nodes.size() <= 0){
			c = false;
			std::cout << "no nodes found!"<< std::endl;
		}
		for (LGNode *n: nodes)
		{
			if (n->level >= num_levels){
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

	void LGState::hook() {
		lg_v64 *sd = d;
		lg_v64 vd = *d;
		d = &vd;
		lg_v64 *srst = rst;
		lg_v64 vrst = *rst;
		rst = &vrst;
		lg_v64 *srst_n = rst_n;
		lg_v64 vrst_n = *rst_n;
		rst_n = &vrst_n;
		lg_v64 *sload = load;
		lg_v64 vload = *load;
		load = &vload;
		lg_v64 *sload_n = load_n;
		lg_v64 vload_n = *load_n;
		load_n = &vload_n;
		hook_inputs();
		sim();
		hook_outputs();
		d = sd;
		rst = srst;
		rst_n = srst_n;
		load = sload;
		load_n = sload_n;
	}


	void LeveledGraph::adapt(const ds_pattern::CombinationalPatternAdapter* adapter){

		// prepare input for simulation: set pattern block and offset
		for (Input* in:inputs){
			in->set_pattern_block(&pattern_block);
			std::string name = in->get_name();
			std::string port_name = name.substr(name.find('/') + 1);
			std::size_t offset = adapter->get_offset(port_name);
			in->set_offset(offset);
		}
		// prepare outputs for simulation: create observers and attach them to outputs
		for (Output* out:outputs){
			std::string name = out->get_name();
			std::string port_name = name.substr(name.find('/') + 1);
			std::size_t offset = adapter->get_offset(port_name);
			out->remove_monitors();
			OutputObserver* observer= new OutputObserver(offset, &pattern_block);
			out->add_monitor(observer);
		}
	}


	void LeveledGraph::sim(ds_pattern::SimPatternBlock *pb){
		pattern_block = pb;
		//propagates events from in level order from inputs to outputs
		for (auto it=nodes.begin();it!=nodes.end();it++){
			LGNode *n = *it;
			n->propagate(false);
			n->mark();
		}
		//inject hooks
		for (ds_faults::SimulationHook* h: hooks){
			LGNode *node = h->get_hook_node(this);
			push_node(node);
		}
		sim_intermediate();
	}

	void LeveledGraph::sim_intermediate(){
		std::set<LGNode*> set;
		for (int i=0;i<num_levels;i++){
			lg_node_container* level = simulation[i];
			iteration++;
			for (auto it=level->begin();it!=level->end();it++){
				LGNode *n = *it;
				if (n->propagate(true)){
					for (LGNode *o : n->outputs){
						auto s = set.find(o);
						if (s==set.end()){
							push_node(o);
							set.insert(o);
						}
					}
				}
			}
		}
		for (int i=0;i<num_levels;i++){
			lg_node_container* level = simulation[i];
			for (auto it=level->begin();it!=level->end();it++){
				LGNode *n = *it;
				n->rollback();
			}
			level->clear();
		}

	}

	ds_common::int64 LeveledGraph::propagate_to_check_point(ds_faults::SimulationHook *h){
		LGNode *n = h->get_hook_node(this);
		n->add_hook(h);

		LGNode* node = get_check_point(n);
		lg_v64 ff = node->get_mark();

		std::vector<LGNode*> path;
		path.push_back(n);
		bool p = n->propagate(true);
		n->remove_hook(h);

		if (p)
			while (n!=node){
				n = n->outputs[0];
				path.push_back(n);
				if (!n->propagate(true))
					break;
			}

		lg_v64 faulty = n->peek();
		for (LGNode *p:path){
			p->rollback();
		}

		if (n!=node)
			return 0;

		ds_common::int64 result = ~ff.x & ~faulty.x & (ff.v ^ faulty.v);

		return result;
	}

	LGNode* LeveledGraph::get_check_point(LGNode* node){
		LGNode *fo = node;
		if (fo->outputs.size()!=0){
			while (fo->outputs.size()==1){
				fo = fo->outputs[0];
			}
		}
		return fo;
	}

	void LeveledGraph::add_hook(ds_faults::SimulationHook* hook){
		hooks.push_back(hook);
		LGNode *node = hook->get_hook_node(this);
		node->add_hook(hook);
		push_node(node);
	};


	void LeveledGraph::clear_hooks(){
		for (auto it=hooks.begin();it!=hooks.end();it++){
			ds_faults::SimulationHook *hook = *it;
			LGNode *node = hook->get_hook_node(this);
			node->remove_hooks();
		}
		hooks.clear();
	}

	void LeveledGraph::clear_hook(ds_faults::SimulationHook* hook){
		hooks.remove(hook);
		LGNode *node = hook->get_hook_node(this);
		node->remove_hook(hook);
	};
}
