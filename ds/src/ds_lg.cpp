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

	void LogicNode::hook_inputs(){
		for (ds_lg::SimulationHook<LogicNode> *h : hooks){
			lg_v64** input_address = get_input(h->get_hook_port());
			if (input_address!=0){
				lg_v64* p_port = *input_address;
				if (p_port!=0){
					int64 activation = h->hook(resolver);
					p_port->v ^= activation & ~p_port->x;
				}
			}
		}
	}

	void LogicNode::hook_outputs(){
		for (ds_lg::SimulationHook<LogicNode> *h : hooks){
			lg_v64* p_port = get_output(h->get_hook_port());
			if (p_port!=0){
				int64 activation = h->hook(resolver);
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

	lg_v64** LogicState::get_input(const std::string& name) {
		if (name == "d")return d.get_input("a");
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
		for (ds_lg::SimulationHook<LogicNode> *h : hooks){
			o.v ^=  h->hook(resolver) & ~o.x;
		}
	}

	void Input::hook() {
		sim();
		for (ds_lg::SimulationHook<LogicNode> *h : hooks){
			o.v ^=  h->hook(resolver) & ~o.x;
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

	void LogicState::hook() {

		lg_v64 *sd = *d.get_input("a");
		lg_v64 vd(**d.get_input("a"));
		*d.get_input("a") = &vd;
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
		*d.get_input("a") = sd;
		rst = srst;
		rst_n = srst_n;
		load = sload;
		load_n = sload_n;
	}

	driver_v64** TNode1I::get_input(const std::string& name) {
		if (name == "a")return &a;
		return 0;
	}

	driver_v64** TNode2I::get_input(const std::string& name) {
		if (name == "a")return &a;
		if (name == "b")return &b;
		return 0;
	}

	driver_v64** TNode3I::get_input(const std::string& name) {
		if (name == "a")return &a;
		if (name == "b")return &b;
		if (name == "c")return &c;
		return 0;
	}

	driver_v64** TState::get_input(const std::string& name) {
		if (name == "d")return d.get_input("a");
		if (name == "cd")return &cd;
		if (name == "rst")return &rst;
		if (name == "rst_n")return &rst_n;
		if (name == "load")return &load;
		if (name == "load_n")return &load_n;
		if (name == "si")return &si;
		if (name == "se")return &se;
		return 0;
	}

	void TOutput::hook() {
		sim();
		for (ds_lg::SimulationHook<TNode> *h : hooks){
			o.value.v ^=  h->hook(resolver) & ~o.value.x;
		}
	}

	void TInput::hook() {
		sim();
		for (ds_lg::SimulationHook<TNode> *h : hooks){
			o.value.v ^=  h->hook(resolver) & ~o.value.x;
		}
	}

	void TNode1I::hook() {
		driver_v64 *sa = a;
		driver_v64 va(*a);
		a = &va;
		hook_inputs();
		sim();
		hook_outputs();
		a = sa;
	}

	void TNode2I::hook() {
		driver_v64 *sa = a;
		driver_v64 va(*a);
		a = &va;
		driver_v64 *sb = b;
		driver_v64 vb(*b);
		b = &vb;
		hook_inputs();
		sim();
		hook_outputs();
		a = sa;
		b = sb;
	}

	void TNode3I::hook() {
		driver_v64 *sa = a;
		driver_v64 va(*a);
		a = &va;
		driver_v64 *sb = b;
		driver_v64 vb(*b);
		b = &vb;
		driver_v64 *sc = c;
		driver_v64 vc(*c);
		c = &vc;
		hook_inputs();
		sim();
		hook_outputs();
		a = sa;
		b = sb;
		c = sc;
	}

	void TState::hook() {
		driver_v64 *sd = *d.get_input("a");
		driver_v64 vd(**d.get_input("a"));
		*d.get_input("a") = &vd;
		driver_v64 *srst = rst;
		driver_v64 vrst(*rst);
		rst = &vrst;
		driver_v64 *srst_n = rst_n;
		driver_v64 vrst_n(*rst_n);
		rst_n = &vrst_n;
		driver_v64 *sload = load;
		driver_v64 vload = *load;
		load = &vload;
		driver_v64 *sload_n = load_n;
		driver_v64 vload_n(*load_n);
		load_n = &vload_n;
		hook_inputs();
		sim();
		hook_outputs();
		*d.get_input("a") = sd;
		rst = srst;
		rst_n = srst_n;
		load = sload;
		load_n = sload_n;
	}

	void TState::hook_outputs(){
		for (ds_lg::SimulationHook<TNode> *h : hooks){
			driver_v64* p_port = get_output(h->get_hook_port());
			if (p_port!=0){
				int64 activation = h->hook(resolver);
				p_port->value.v ^= activation;// & ~p_port->value.x;
			}
		}
	}

	void TNode::hook_inputs(){
		for (ds_lg::SimulationHook<TNode> *h : hooks){
			std::string hport = h->get_hook_port();
			driver_v64** input_address = get_input(hport);
			if (input_address!=0){
				driver_v64* p_port = *input_address;
				if (p_port!=0){
					int64 activation = h->hook(resolver);
					p_port->value.v ^= activation & ~p_port->value.x;
				}
			}
		}
	}

	void TNode::hook_outputs(){
		for (ds_lg::SimulationHook<TNode> *h : hooks){
			driver_v64* p_port = get_output(h->get_hook_port());
			if (p_port!=0){
				int64 activation = h->hook(resolver);
				p_port->value.v ^= activation;// & ~p_port->value.x;
			}
		}
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

	void ds_lg::LeveledGraph::attach_output_observers(std::map<ds_lg::LogicNode*, ds_lg::ErrorObserver*>& output_map,
					ds_common::int64 *detected, ds_common::int64 *possibly_detected){

		for (auto it=outputs.begin(); it!=outputs.end();it++){
			LogicNode *o = *it;
			ds_lg::ErrorObserver *observer = new ds_lg::ErrorObserver(detected, possibly_detected);
			output_map[o] = observer;
			o->add_monitor(observer);
		}
	}

	void ds_lg::LeveledGraph::attach_output_observers(ds_common::int64 *detected, ds_common::int64 *possibly_detected) {

		for (auto it=outputs.begin(); it!=outputs.end();it++){
			LogicNode *o = *it;
			ds_lg::ErrorObserver *observer = new ds_lg::ErrorObserver(detected, possibly_detected);
			o->add_monitor(observer);
		}
	}

	void ds_lg::LeveledGraph::remove_output_observers(){

		for (auto it=outputs.begin(); it!=outputs.end();it++){
			LogicNode *o = *it;
			o->delete_monitors();
		}
	}


	void TLeveledGraph::adapt(const ds_pattern::SequentialPatternAdapter* adapter){
		// prepare input for simulation: set pattern block and offset
		for (TInput* in:inputs){
			in->set_pattern_block(&pattern_block);
			std::string name = in->get_name();
			std::string port_name = name.substr(name.find('/') + 1);
			std::size_t offset = adapter->get_port_offset(port_name);
			in->set_offset(offset);
			in->set_vector_offset(&vector_offset);
		}
		// prepare outputs for simulation: create observers and attach them to outputs
		for (TOutput* out:outputs){
			std::string name = out->get_name();
			std::string port_name = name.substr(name.find('/') + 1);
			std::size_t offset = adapter->get_port_offset(port_name);
			out->remove_monitors();
			TOutputObserver* observer= new TOutputObserver(offset, &pattern_block, &vector_offset);
			out->add_monitor(observer);
		}
		std::size_t scan_offset = adapter->get_scan_offset();
		for (TState* reg:registers){
			std::string name = reg->get_name();
			std::string cell_name = name.substr(name.find('/') + 1);
			std::size_t offset = adapter->get_scan_offset(cell_name);
			reg->set_pattern_block(&pattern_block);
			reg->set_offset(scan_offset + offset);
		}
	}

	void TLeveledGraph::setup(){
		std::vector<TNode*> temp;
		for (auto it=nodes.begin();it!=nodes.end();it++){
			TNode *n = *it;
			if (!n->has_state()){
				temp.push_back(n);
			}
		}
		nodes.clear();
		nodes.insert(nodes.begin(), temp.begin(), temp.end());
		nodes.insert(nodes.begin(), registers.begin(), registers.end());
	}

	void TErrorObserver::observe(const ds_lg::driver_v64& v) {
		ds_common::int64 detected = (~spec.x & ~v.value.x & spec.v & ~v.value.v) | (~spec.x & ~v.value.x & ~spec.v & v.value.v);
		*ds |= detected;

		ds_common::int64 possibly_detected = ~spec.x & v.value.x;
		*np |= (possibly_detected & ~detected);
	}

	void ds_lg::TLeveledGraph::attach_output_observers(std::map<ds_lg::TNode*, ds_lg::TErrorObserver*>& output_map,
		ds_common::int64 *detected, ds_common::int64 *possibly_detected){

		for (auto it=outputs.begin(); it!=outputs.end();it++){
			TNode *o = *it;
			ds_lg::TErrorObserver *observer = new ds_lg::TErrorObserver(detected, possibly_detected);
			output_map[o] = observer;
			o->add_monitor(observer);
		}
	}

	void ds_lg::TLeveledGraph::attach_register_observers(std::map<ds_lg::TNode*, ds_lg::TErrorObserver*>& register_map,
		ds_common::int64 *detected, ds_common::int64 *possibly_detected){

		for (auto it=registers.begin(); it!=registers.end();it++){
			ds_lg::TState *r = *it;
			TNode *d = r->get_sink();
			ds_lg::TErrorObserver *observer = new ds_lg::TErrorObserver(detected, possibly_detected);
			observer->tag = d->get_name();
			register_map[d] = observer;
			d->add_monitor(observer);
		}

	}
}

