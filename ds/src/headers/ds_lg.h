/*
 * lg.h
 *
 *  Created on: Aug 26, 2012
 *      Author: cookao
 */

#ifndef DS_LG_H_
#define DS_LG_H_

#include <boost/bind.hpp>
#include <vector>
#include "ds_common.h"
#include "ds_structural.h"
#include "ds_pattern.h"

namespace ds_lg {

	using ds_common::int64;

	struct lg_v64 {
		ds_common::int64 v;
		ds_common::int64 x;

		lg_v64():v(0),x(-1){}
		lg_v64(const ds_common::int64 vv, const ds_common::int64 xx):v(vv),x(xx){}
		lg_v64(const lg_v64& v):v(v.v),x(v.x){}

		lg_v64 operator~() const{
			lg_v64 o;
			o.v = ~v;
			o.x = x;
			return o;
		}

		lg_v64& operator&=(const lg_v64& rhs){
			v &= rhs.v;
			x = (x & ~rhs.x & ~rhs.v) | (rhs.x & ~v & ~x);
			return *this;
		}

		lg_v64& operator|=(const lg_v64& rhs){
			v |= rhs.v;
			x = (x & ~rhs.x & rhs.v) | (rhs.x & v &x);
			return *this;
		}

		lg_v64& operator^=(const lg_v64& rhs){
			v ^= rhs.v;
			x = rhs.x | x;
			return *this;
		}


	};

	inline lg_v64 operator&(lg_v64 lhs, const lg_v64& rhs){
		lhs &= rhs;
		return lhs;
	}

	inline lg_v64 operator|(lg_v64 lhs, const lg_v64& rhs){
		lhs |= rhs;
		return lhs;
	}

	inline lg_v64 operator^(lg_v64 lhs, const lg_v64& rhs){
		lhs ^= rhs;
		return lhs;
	}



	typedef const lg_v64* const val64_cpc;
	typedef lg_v64 (*bi_eval)(val64_cpc, val64_cpc);

	inline lg_v64 f_buf(val64_cpc a) {return *a;}
	inline lg_v64 f_not(val64_cpc a) {return ~(*a);}

	inline lg_v64 f_and2(val64_cpc a, val64_cpc b){return *a & *b;}
	inline lg_v64 f_or2(val64_cpc a, val64_cpc b){return *a | *b;}
	inline lg_v64 f_nand2(val64_cpc a, val64_cpc b){return ~(*a & *b);}
	inline lg_v64 f_nor2(val64_cpc a, val64_cpc b){return ~(*a | *b);}
	inline lg_v64 f_xnor2(val64_cpc a, val64_cpc b){return ~(*a ^ *b);}
	inline lg_v64 f_xor2(val64_cpc a, val64_cpc b){return *a ^ *b;}

	inline lg_v64 f_and3(val64_cpc a, val64_cpc b, val64_cpc c){ return *a & *b & *c;}
	inline lg_v64 f_or3(val64_cpc a, val64_cpc b, val64_cpc c){ return *a | *b | *c;}
	inline lg_v64 f_nand3(val64_cpc a, val64_cpc b, val64_cpc c){ return ~(*a & *b & *c);}
	inline lg_v64 f_nor3(val64_cpc a, val64_cpc b, val64_cpc c){ return ~(*a | *b | *c);}
	inline lg_v64 f_xnor3(val64_cpc a, val64_cpc b, val64_cpc c){ return ~(*a ^ *b ^ *c);}
	inline lg_v64 f_xor3(val64_cpc a, val64_cpc b, val64_cpc c){ return *a ^ *b ^ *c;}

	inline lg_v64 f_and4(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d){ return *a & *b & *c & *d;}
	inline lg_v64 f_or4(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d){ return *a | *b | *c | *d;}
	inline lg_v64 f_nand4(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d){ return ~(*a & *b & *c & *d);}
	inline lg_v64 f_nor4(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d){ return ~(*a | *b | *c | *d);}
	inline lg_v64 f_xnor4(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d){ return ~(*a ^ *b ^ *c ^ *d);}
	inline lg_v64 f_xor4(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d){ return *a ^ *b ^ *c ^ *d;}

	inline lg_v64 f_and5(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e){ return *a & *b & *c & *d & *e;}
	inline lg_v64 f_or5(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e){ return *a | *b | *c | *d | *e;}
	inline lg_v64 f_nand5(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e){ return ~(*a & *b & *c & *d & *e);}
	inline lg_v64 f_nor5(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e){ return ~(*a | *b | *c | *d | *e);}
	inline lg_v64 f_xnor5(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e){ return ~(*a ^ *b ^ *c ^ *d ^ *e);}
	inline lg_v64 f_xor5(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e){ return *a ^ *b ^ *c ^ *d ^ *e;}

	inline lg_v64 f_and6(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e, val64_cpc f){ return *a & *b & *c & *d & *e & *f;}
	inline lg_v64 f_or6(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e, val64_cpc f){ return *a | *b | *c | *d | *e | *f;}
	inline lg_v64 f_nand6(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e, val64_cpc f){ return ~(*a & *b & *c & *d & *e & *f);}
	inline lg_v64 f_nor6(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e, val64_cpc f){ return ~(*a | *b | *c | *d | *e | *f);}
	inline lg_v64 f_xnor6(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e, val64_cpc f){ return ~(*a ^ *b ^ *c ^ *d ^ *e ^ *f);}
	inline lg_v64 f_xor6(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e, val64_cpc f){ return *a ^ *b ^ *c ^ *d ^ *e ^ *f;}

	inline lg_v64 f_and7(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e, val64_cpc f, val64_cpc g){ return *a & *b & *c & *d & *e & *f & *g;}
	inline lg_v64 f_or7(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e, val64_cpc f, val64_cpc g){ return *a | *b | *c | *d | *e | *f | *g;}
	inline lg_v64 f_nand7(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e, val64_cpc f, val64_cpc g){ return ~(*a & *b & *c & *d & *e & *f & *g);}
	inline lg_v64 f_nor7(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e, val64_cpc f, val64_cpc g){ return ~(*a | *b | *c | *d | *e | *f | *g);}
	inline lg_v64 f_xnor7(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e, val64_cpc f, val64_cpc g){ return ~(*a ^ *b ^ *c ^ *d ^ *e ^ *f ^ *g);}
	inline lg_v64 f_xor7(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e, val64_cpc f, val64_cpc g){ return *a ^ *b ^ *c ^ *d ^ *e ^ *f ^ *g;}

	inline lg_v64 f_and8(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e, val64_cpc f, val64_cpc g, val64_cpc h){ return *a & *b & *c & *d & *e & *f & *g & *h ;}
	inline lg_v64 f_or8(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e, val64_cpc f, val64_cpc g, val64_cpc h){ return *a | *b | *c | *d | *e | *f | *g | *h;}
	inline lg_v64 f_nand8(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e, val64_cpc f, val64_cpc g, val64_cpc h){ return ~(*a & *b & *c & *d & *e & *f & *g & *h );}
	inline lg_v64 f_nor8(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e, val64_cpc f, val64_cpc g, val64_cpc h){ return ~(*a | *b | *c | *d | *e | *f | *g | *h );}
	inline lg_v64 f_xnor8(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e, val64_cpc f, val64_cpc g, val64_cpc h){ return ~(*a ^ *b ^ *c ^ *d ^ *e ^ *f ^ *g ^ *h );}
	inline lg_v64 f_xor8(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e, val64_cpc f, val64_cpc g, val64_cpc h){ return *a ^ *b ^ *c ^ *d ^ *e ^ *f ^ *g ^ *h;}

	inline lg_v64 f_mux2(val64_cpc a, val64_cpc b, val64_cpc c){ return (*a & *c) | (*b & ~(*c));}

	class Monitor {
	public:
		virtual void observe(const lg_v64& v) = 0;
	};

	class OutputObserver : public Monitor {
	public:
		std::size_t offset;
		ds_pattern::SimPatternBlock** pb;
		OutputObserver(std::size_t output_offset, ds_pattern::SimPatternBlock** pattern_block):offset(output_offset){
			pb = pattern_block;
		}
		virtual void observe(const lg_v64& v) {
			(*pb)->values[offset].v = v.v;
			(*pb)->values[offset].x = v.x;
		}
	};

	typedef std::vector<Monitor*> monitor_container;

	class LGNode {
	public:
		int level;
		ds_structural::Gate *gate;
		std::vector<LGNode*> outputs;
		std::vector<LGNode*> inputs;
		LGNode(const std::string &t):level(-1),gate(0),endpoint(false),type(t){}
		void set_gate(ds_structural::Gate* g) {gate = g;}
		ds_structural::Gate* get_gate() const{return gate;}
		virtual void sim()=0;
		virtual bool propagate() {
			bo = o;
			sim();
			for (monitor_container::iterator it = monitors.begin();it!=monitors.end();it++){
				Monitor *m = *it;
				m->observe(o);
			}
			if (!endpoint)
				return (bo.v!=o.v || bo.x!=o.x);
			return false;
		}
		virtual lg_v64** get_binding(const std::string& name)=0;
		virtual lg_v64* get_output(const std::string& name)=0;
		virtual LGNode* clone()=0;
		virtual ~LGNode(){};
		std::string get_type() const {return type;}
		virtual bool has_state() {return false;}
		bool is_endpoint()const {return endpoint;}
		void add_monitor(Monitor* m){monitors.push_back(m);}
		unsigned int num_monitors() const {return monitors.size();}
		virtual std::string get_name() const {
			std::string name = gate !=0 ? gate->get_instance_name() : "";
			return name;
		}
		lg_v64 peek() const {return o;}
	protected:
		lg_v64 o;
		lg_v64 bo;
		bool endpoint;
		std::string type;
		monitor_container monitors;
	};

	class Output : public LGNode {
	protected:
		lg_v64 *a;
		std::string name;
	public:
		void set_name(const std::string& n){name = n;}
		virtual std::string get_name() const {return name;}
		virtual void sim() {
			o = *a;
		}
		Output():LGNode("output"){ endpoint = true;};
		virtual LGNode* clone() { return new Output();}
		virtual lg_v64* get_output(const std::string& name) {if (name=="o") return &o; return 0;}
		virtual lg_v64** get_binding(const std::string& name){
			if (name == "a") return &a;
			return 0;
		}
	};

	class Input : public LGNode {
	protected:
		std::string name;
		std::size_t offset;
		ds_pattern::SimPatternBlock** pb;
	public:
		void set_name(const std::string& n){name = n;}
		virtual std::string get_name() const {return name;}
		virtual void sim() {
			o.v = (*pb)->values[offset].v;
			o.x = (*pb)->values[offset].x;
		}
		Input():LGNode("input"){};
		virtual LGNode* clone() { return new Input();}
		virtual lg_v64* get_output(const std::string& name) {if (name=="o") return &o; return 0;}
		virtual lg_v64** get_binding(const std::string& name){
			return 0;
		}
		void set_pattern_block(ds_pattern::SimPatternBlock** pattern_block){pb = pattern_block;}
		void set_offset(const std::size_t input_offset){offset = input_offset;}
		std::size_t get_offset() const {return offset;}
	};

	class LGNode1I : public LGNode {
	protected:
		lg_v64 *a;
		lg_v64 (*A)(val64_cpc a);
	public:
		virtual void sim() {o = (*A)(a);}
		LGNode1I(const std::string& t, lg_v64 (*AA)(val64_cpc)):LGNode(t), A(AA){};
		virtual LGNode* clone() { return new LGNode1I(type, A);}
		virtual lg_v64* get_output(const std::string& name) {if (name=="o")return &o; return 0;}
		virtual lg_v64** get_binding(const std::string& name);
	};

	class LGNode2I : public LGNode {
	protected:
		lg_v64 * a;
		lg_v64 * b;
		lg_v64 (*A)(val64_cpc a, val64_cpc b);
	public:
		virtual void sim() {o = (*A)(a,b);}
		LGNode2I(const std::string& t, lg_v64 (*AA)(val64_cpc, val64_cpc)):LGNode(t), A(AA){};
		virtual LGNode* clone() { return new LGNode2I(type, A); }
		virtual lg_v64* get_output(const std::string& name) {if (name=="o")return &o; return 0;}
		virtual lg_v64** get_binding(const std::string& name);
	};

	class LGNode3I : public LGNode {
	protected:
		lg_v64 * a;
		lg_v64 * b;
		lg_v64 * c;
		lg_v64 (*A)(val64_cpc a, val64_cpc b, val64_cpc c);
	public:
		virtual void sim() {o = (*A)(a,b,c);}
		LGNode3I(const std::string& t, lg_v64 (*AA)(val64_cpc, val64_cpc, val64_cpc c)):LGNode(t), A(AA){};
		virtual LGNode* clone() { return new LGNode3I(type, A); }
		virtual lg_v64** get_binding(const std::string& name);
		virtual lg_v64* get_output(const std::string& name) {if (name=="o")return &o; return 0;}
	};

	class LGNode4I : public LGNode {
	protected:
		lg_v64 * a;
		lg_v64 * b;
		lg_v64 * c;
		lg_v64 * d;
		lg_v64 shadow;
		lg_v64 (*A)(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d);
	public:
		virtual void sim() {o = (*A)(a,b,c,d);}
		LGNode4I(const std::string& t, lg_v64 (*AA)(val64_cpc, val64_cpc, val64_cpc, val64_cpc)):LGNode(t), A(AA){};
		virtual LGNode* clone() { return new LGNode4I(type, A); }
		virtual lg_v64* get_output(const std::string& name) {if (name=="o")return &o; return 0;}
		virtual lg_v64** get_binding(const std::string& name);
	};

	class LGNode5I : public LGNode {
	protected:
		lg_v64 * a;
		lg_v64 * b;
		lg_v64 * c;
		lg_v64 * d;
		lg_v64 * e;
		lg_v64 (*A)(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e);
	public:
		virtual void sim() { o = (*A)(a,b,c,d,e);}
		LGNode5I(const std::string& t, lg_v64 (*AA)(val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc)):LGNode(t), A(AA){};
		virtual LGNode* clone() { return new LGNode5I(type, A); }
		virtual lg_v64* get_output(const std::string& name) {if (name=="o")return &o; return 0;}
		virtual lg_v64** get_binding(const std::string& name);
	};

	class LGNode6I : public LGNode {
	protected:
		lg_v64 * a;
		lg_v64 * b;
		lg_v64 * c;
		lg_v64 * d;
		lg_v64 * e;
		lg_v64 * f;
		lg_v64 shadow;
		lg_v64 (*A)(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e, val64_cpc f);
	public:
		virtual void sim() {o = (*A)(a,b,c,d,e,f);}
		LGNode6I(const std::string& t, lg_v64 (*AA)(val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc)):LGNode(t), A(AA){};
		virtual LGNode* clone() { return new LGNode6I(type, A); }
		virtual lg_v64* get_output(const std::string& name) {if (name=="o")return &o; return 0;}
		virtual lg_v64** get_binding(const std::string& name);
	};

	class LGNode7I : public LGNode {
	protected:
		lg_v64 * a;
		lg_v64 * b;
		lg_v64 * c;
		lg_v64 * d;
		lg_v64 * e;
		lg_v64 * f;
		lg_v64 * g;
		lg_v64 (*A)(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e, val64_cpc f, val64_cpc g);
	public:
		virtual void sim() {o = (*A)(a,b,c,d,e,f,g);}
		LGNode7I(const std::string& t, lg_v64 (*AA)(val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc)):LGNode(t), A(AA){};
		virtual LGNode* clone() { return new LGNode7I(type, A); }
		virtual lg_v64* get_output(const std::string& name) {if (name=="o")return &o; return 0;}
		virtual lg_v64** get_binding(const std::string& name);
	};

	class LGNode8I : public LGNode {
	protected:
		lg_v64 * a;
		lg_v64 * b;
		lg_v64 * c;
		lg_v64 * d;
		lg_v64 * e;
		lg_v64 * f;
		lg_v64 * g;
		lg_v64 * h;
		lg_v64 (*A)(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e, val64_cpc f, val64_cpc g, val64_cpc h);
	public:
		virtual void sim() {o = (*A)(a,b,c,d,e,f,g,h);}
		LGNode8I(const std::string& t, lg_v64 (*AA)(val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc)):LGNode(t), A(AA){};
		virtual LGNode* clone() { return new LGNode8I(type, A); }
		virtual lg_v64* get_output(const std::string& name) {if (name=="o")return &o; return 0;}
		virtual lg_v64** get_binding(const std::string& name);
	};

	class LGNodeArr : public LGNode {
	protected:
		lg_v64 o;
		int inputSize;
		lg_v64 (*A)(val64_cpc a, val64_cpc b);
		lg_v64 shadow;
		lg_v64 ** input_array;
		bool invert;
	public:
		virtual void sim() {
			shadow = o;
			o = **input_array;
			for (int i=1;i<inputSize;i++){
				o = (*A)(&o, input_array[i]);
			}
			if(invert)
				o = ~o;
		}
		LGNodeArr(const std::string& t, const int& inputs, lg_v64 (*AA)(val64_cpc, val64_cpc), bool iv):LGNode(t), inputSize(inputs), A(AA), invert(iv){
			input_array = new lg_v64*[inputSize];
		};
		virtual LGNode* clone() { return new LGNodeArr(type, inputSize, A, invert); }
		virtual lg_v64* get_output(const std::string& name) {if (name=="z")return &o; return 0;}
		virtual lg_v64** get_binding(const std::string& name);
		virtual ~LGNodeArr(){
			delete[] input_array;
		};
	};

	class LGState : public LGNode {
		protected:
			lg_v64 * d;
			lg_v64 * cd;
			lg_v64 q;
			lg_v64 qn;
		public:
			virtual void sim() {}
			LGState(const std::string& t):LGNode(t){endpoint = true;};
			virtual LGNode* clone() { return new LGState(type); }
			virtual lg_v64* get_output(const std::string& name) {
				if (name=="Q")return &q;
				if (name=="QN")return &qn;
				return 0;}
			virtual lg_v64** get_binding(const std::string& name);
			virtual bool has_state() {return true;}
			virtual ~LGState(){};
		};

	typedef std::vector<LGNode*> lg_node_container;
	typedef lg_node_container::iterator lg_node_iterator;
	typedef std::vector<Input*> lg_input_container;
	typedef std::vector<Output*> lg_output_container;

	class LeveledGraph {

		friend class ds_structural::NetList;

	public:

		lg_input_container inputs;
		lg_output_container outputs;
		lg_node_container nodes;
		lg_node_container registers;

		bool sanity_check();
		void add_input(Input* in){inputs.push_back(in);}
		void add_output(Output* out){outputs.push_back(out);}
		lg_input_container::iterator get_inputs_begin(){return inputs.begin();}
		lg_input_container::iterator get_inputs_end(){return inputs.end();}
		lg_output_container::iterator get_outputs_begin(){return outputs.begin();}
		lg_output_container::iterator get_outputs_end(){return outputs.end();}
		void adapt(const ds_pattern::CombinationalPatternAdapter* adapter);
		void sim(ds_pattern::SimPatternBlock * pb);
		Output* get_output(const std::string name){

			lg_output_container::iterator n = std::find_if(outputs.begin(), outputs.end(), [&](Output* o) {
				return name == o->get_name();
			});
			if (n == outputs.end())
				return 0;
			return *n;
		}
		LGNode* get_node(const std::string name){
			lg_node_iterator n = std::find_if(nodes.begin(), nodes.end(), [&](LGNode* p) {
					//std::cout << name << " " << p->get_name() << std::endl;
					return name == p->get_name();
			});
			if (n == nodes.end())
				return 0;
			return *n;
		}

	private:
		ds_pattern::SimPatternBlock *pattern_block;
		std::vector<lg_node_iterator> levels;
		std::vector<unsigned int> level_width;
		int num_levels;
		lg_v64 constant_0 = lg_v64(0L,0L);
		lg_v64 constant_1 = lg_v64(-1L,0L);
		lg_v64 constant_X = lg_v64(0L,-1L);
	};
}

#endif /* LG_H_ */
