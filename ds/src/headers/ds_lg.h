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

namespace ds_lg {

	using ds_common::int64;
	typedef const ds_common::int64* const int64_cpc;
	typedef int64 (*bi_eval)(int64_cpc, int64_cpc);

	inline int64 f_buf(int64_cpc a) { return *a;}
	inline int64 f_not(int64_cpc a) { return ~(*a);}

	inline int64 f_and2(int64_cpc a, int64_cpc b){ return *a & *b;}
	inline int64 f_or2(int64_cpc a, int64_cpc b){ return *a | *b;}
	inline int64 f_nand2(int64_cpc a, int64_cpc b){ return ~(*a & *b);}
	inline int64 f_nor2(int64_cpc a, int64_cpc b){ return ~(*a | *b);}
	inline int64 f_xnor2(int64_cpc a, int64_cpc b){ return ~(*a ^ *b);}
	inline int64 f_xor2(int64_cpc a, int64_cpc b){ return *a ^ *b;}

	inline int64 f_and3(int64_cpc a, int64_cpc b, int64_cpc c){ return *a & *b & *c;}
	inline int64 f_or3(int64_cpc a, int64_cpc b, int64_cpc c){ return *a | *b | *c;}
	inline int64 f_nand3(int64_cpc a, int64_cpc b, int64_cpc c){ return ~(*a & *b & *c);}
	inline int64 f_nor3(int64_cpc a, int64_cpc b, int64_cpc c){ return ~(*a | *b | *c);}
	inline int64 f_xnor3(int64_cpc a, int64_cpc b, int64_cpc c){ return ~(*a ^ *b ^ *c);}
	inline int64 f_xor3(int64_cpc a, int64_cpc b, int64_cpc c){ return *a ^ *b ^ *c;}

	inline int64 f_and4(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d){ return *a & *b & *c & *d;}
	inline int64 f_or4(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d){ return *a | *b | *c | *d;}
	inline int64 f_nand4(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d){ return ~(*a & *b & *c & *d);}
	inline int64 f_nor4(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d){ return ~(*a | *b | *c | *d);}
	inline int64 f_xnor4(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d){ return ~(*a ^ *b ^ *c ^ *d);}
	inline int64 f_xor4(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d){ return *a ^ *b ^ *c ^ *d;}

	inline int64 f_and5(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e){ return *a & *b & *c & *d & *e;}
	inline int64 f_or5(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e){ return *a | *b | *c | *d | *e;}
	inline int64 f_nand5(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e){ return ~(*a & *b & *c & *d & *e);}
	inline int64 f_nor5(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e){ return ~(*a | *b | *c | *d | *e);}
	inline int64 f_xnor5(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e){ return ~(*a ^ *b ^ *c ^ *d ^ *e);}
	inline int64 f_xor5(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e){ return *a ^ *b ^ *c ^ *d ^ *e;}

	inline int64 f_and6(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e, int64_cpc f){ return *a & *b & *c & *d & *e & *f;}
	inline int64 f_or6(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e, int64_cpc f){ return *a | *b | *c | *d | *e | *f;}
	inline int64 f_nand6(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e, int64_cpc f){ return ~(*a & *b & *c & *d & *e & *f);}
	inline int64 f_nor6(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e, int64_cpc f){ return ~(*a | *b | *c | *d | *e | *f);}
	inline int64 f_xnor6(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e, int64_cpc f){ return ~(*a ^ *b ^ *c ^ *d ^ *e ^ *f);}
	inline int64 f_xor6(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e, int64_cpc f){ return *a ^ *b ^ *c ^ *d ^ *e ^ *f;}

	inline int64 f_and7(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e, int64_cpc f, int64_cpc g){ return *a & *b & *c & *d & *e & *f & *g;}
	inline int64 f_or7(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e, int64_cpc f, int64_cpc g){ return *a | *b | *c | *d | *e | *f | *g;}
	inline int64 f_nand7(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e, int64_cpc f, int64_cpc g){ return ~(*a & *b & *c & *d & *e & *f & *g);}
	inline int64 f_nor7(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e, int64_cpc f, int64_cpc g){ return ~(*a | *b | *c | *d | *e | *f | *g);}
	inline int64 f_xnor7(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e, int64_cpc f, int64_cpc g){ return ~(*a ^ *b ^ *c ^ *d ^ *e ^ *f ^ *g);}
	inline int64 f_xor7(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e, int64_cpc f, int64_cpc g){ return *a ^ *b ^ *c ^ *d ^ *e ^ *f ^ *g;}

	inline int64 f_and8(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e, int64_cpc f, int64_cpc g, int64_cpc h){ return *a & *b & *c & *d & *e & *f & *g & *h ;}
	inline int64 f_or8(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e, int64_cpc f, int64_cpc g, int64_cpc h){ return *a | *b | *c | *d | *e | *f | *g | *h;}
	inline int64 f_nand8(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e, int64_cpc f, int64_cpc g, int64_cpc h){ return ~(*a & *b & *c & *d & *e & *f & *g & *h );}
	inline int64 f_nor8(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e, int64_cpc f, int64_cpc g, int64_cpc h){ return ~(*a | *b | *c | *d | *e | *f | *g | *h );}
	inline int64 f_xnor8(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e, int64_cpc f, int64_cpc g, int64_cpc h){ return ~(*a ^ *b ^ *c ^ *d ^ *e ^ *f ^ *g ^ *h );}
	inline int64 f_xor8(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e, int64_cpc f, int64_cpc g, int64_cpc h){ return *a ^ *b ^ *c ^ *d ^ *e ^ *f ^ *g ^ *h;}

	inline int64 f_mux2(int64_cpc a, int64_cpc b, int64_cpc c){ return (*a & ~(*c)) | (*b & *c);}

	class Monitor {
	public:
		virtual void log(const int64& v);
	};

	class NodeObserver : public Monitor {
		int64 val;
		virtual void log(const int64& v) {
			val = v;
		}
		int64 get_value() const {return val;}
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
		virtual int64 sim()=0;
		virtual bool propagate() {
			int64 l = o;
			int64 v = sim();
			for (monitor_container::iterator it = monitors.begin();it!=monitors.end();it++){
				Monitor *m = *it;
				m->log(o);
			}
			if (!endpoint)
				return l!=v;
			return false;
		}
		virtual int64** get_binding(const char& name)=0;
		virtual int64* get_output(const char& name)=0;
		virtual LGNode* clone()=0;
		virtual ~LGNode(){};
		std::string get_type() const {return type;}
		virtual bool has_state() {return false;}
	protected:
		int64 o;
		bool endpoint;
		std::string type;
		monitor_container monitors;
	};

	class Input : public LGNode {
	protected:
		int64 *a;
		int64 shadow;
		std::string name;
	public:
		void set_name(const std::string& n){name = n;}
		std::string get_name() const {return name;}
		virtual int64 sim() { shadow = o; o = *a; return o;}
		Input():LGNode("input"){};
		virtual LGNode* clone() { return new Input();}
		virtual int64* get_output(const char& name) {if (name=='o')return &o;return 0;}
		virtual int64** get_binding(const char& name){
			switch(name){
			case 'a': return &a;
			default: return 0;
			}
		}
	};

	class Output : public Input {
	public:
		Output():Input(){ type = "output"; endpoint = true;};
		virtual LGNode* clone() { return new Output();}
	};

	class LGNode1I : public LGNode {
	protected:
		int64 *a;
		int64 shadow;
		int64 (*A)(int64_cpc a);
	public:
		virtual int64 sim() { shadow = o; ; o = (*A)(a); return o;}
		LGNode1I(const std::string& t, int64 (*AA)(int64_cpc)):LGNode(t), A(AA){};
		virtual LGNode* clone() { return new LGNode1I(type, A);}
		virtual int64* get_output(const char& name) {if (name=='o')return &o; return 0;}
		virtual int64** get_binding(const char& name);
	};

	class LGNode2I : public LGNode {
	protected:
		int64 * a;
		int64 * b;
		int64 shadow;
		int64 (*A)(int64_cpc a, int64_cpc b);
	public:
		virtual int64 sim() { shadow = o; o = (*A)(a,b); return o;}
		LGNode2I(const std::string& t, int64 (*AA)(int64_cpc, int64_cpc)):LGNode(t), A(AA){};
		virtual LGNode* clone() { return new LGNode2I(type, A); }
		virtual int64* get_output(const char& name) {if (name=='o')return &o; return 0;}
		virtual int64** get_binding(const char& name);
	};

	class LGNode3I : public LGNode {
	protected:
		int64 * a;
		int64 * b;
		int64 * c;
		int64 shadow;
		int64 (*A)(int64_cpc a, int64_cpc b, int64_cpc c);
	public:
		virtual int64 sim() { shadow = o; o = (*A)(a,b,c); return o;}
		LGNode3I(const std::string& t, int64 (*AA)(int64_cpc, int64_cpc, int64_cpc c)):LGNode(t), A(AA){};
		virtual LGNode* clone() { return new LGNode3I(type, A); }
		virtual int64** get_binding(const char& name);
		virtual int64* get_output(const char& name) {if (name=='o')return &o; return 0;}
	};

	class LGNode4I : public LGNode {
	protected:
		int64 * a;
		int64 * b;
		int64 * c;
		int64 * d;
		int64 shadow;
		int64 (*A)(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d);
	public:
		virtual int64 sim() { shadow = o; o = (*A)(a,b,c,d); return o;}
		LGNode4I(const std::string& t, int64 (*AA)(int64_cpc, int64_cpc, int64_cpc, int64_cpc)):LGNode(t), A(AA){};
		virtual LGNode* clone() { return new LGNode4I(type, A); }
		virtual int64* get_output(const char& name) {if (name=='o')return &o; return 0;}
		virtual int64** get_binding(const char& name);
	};

	class LGNode5I : public LGNode {
	protected:
		int64 * a;
		int64 * b;
		int64 * c;
		int64 * d;
		int64 * e;
		int64 shadow;
		int64 (*A)(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e);
	public:
		virtual int64 sim() { shadow = o; o = (*A)(a,b,c,d,e); return o;}
		LGNode5I(const std::string& t, int64 (*AA)(int64_cpc, int64_cpc, int64_cpc, int64_cpc, int64_cpc)):LGNode(t), A(AA){};
		virtual LGNode* clone() { return new LGNode5I(type, A); }
		virtual int64* get_output(const char& name) {if (name=='o')return &o; return 0;}
		virtual int64** get_binding(const char& name);
	};

	class LGNode6I : public LGNode {
	protected:
		int64 * a;
		int64 * b;
		int64 * c;
		int64 * d;
		int64 * e;
		int64 * f;
		int64 shadow;
		int64 (*A)(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e, int64_cpc f);
	public:
		virtual int64 sim() { shadow = o; o = (*A)(a,b,c,d,e,f);return o;}
		LGNode6I(const std::string& t, int64 (*AA)(int64_cpc, int64_cpc, int64_cpc, int64_cpc, int64_cpc, int64_cpc)):LGNode(t), A(AA){};
		virtual LGNode* clone() { return new LGNode6I(type, A); }
		virtual int64* get_output(const char& name) {if (name=='o')return &o; return 0;}
		virtual int64** get_binding(const char& name);
	};

	class LGNode7I : public LGNode {
	protected:
		int64 * a;
		int64 * b;
		int64 * c;
		int64 * d;
		int64 * e;
		int64 * f;
		int64 * g;
		int64 shadow;
		int64 (*A)(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e, int64_cpc f, int64_cpc g);
	public:
		virtual int64 sim() { shadow = o; o = (*A)(a,b,c,d,e,f,g); return o;}
		LGNode7I(const std::string& t, int64 (*AA)(int64_cpc, int64_cpc, int64_cpc, int64_cpc, int64_cpc, int64_cpc, int64_cpc)):LGNode(t), A(AA){};
		virtual LGNode* clone() { return new LGNode7I(type, A); }
		virtual int64* get_output(const char& name) {if (name=='o')return &o; return 0;}
		virtual int64** get_binding(const char& name);
	};

	class LGNode8I : public LGNode {
	protected:
		int64 * a;
		int64 * b;
		int64 * c;
		int64 * d;
		int64 * e;
		int64 * f;
		int64 * g;
		int64 * h;
		int64 shadow;
		int64 (*A)(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e, int64_cpc f, int64_cpc g, int64_cpc h);
	public:
		virtual int64 sim() { shadow = o; o = (*A)(a,b,c,d,e,f,g,h);return o;}
		LGNode8I(const std::string& t, int64 (*AA)(int64_cpc, int64_cpc, int64_cpc, int64_cpc, int64_cpc, int64_cpc, int64_cpc, int64_cpc)):LGNode(t), A(AA){};
		virtual LGNode* clone() { return new LGNode8I(type, A); }
		virtual int64* get_output(const char& name) {if (name=='o')return &o; return 0;}
		virtual int64** get_binding(const char& name);
	};

	class LGNodeArr : public LGNode {
	protected:
		int64 o;
		int inputSize;
		int64 (*A)(int64_cpc a, int64_cpc b);
		int64_cpc op;
		int64 ** input_array;
	public:
		virtual int64 sim() {
			o = **input_array;
			for (int i=0;i<inputSize;i++){
				o = (*A)(op, input_array[i]);
			}
			return o;
		}
		LGNodeArr(const std::string& t, const int& inputs, int64 (*AA)(int64_cpc, int64_cpc)):LGNode(t), inputSize(inputs), A(AA), op(&o){
			input_array = new int64*[inputSize];
		};
		virtual LGNode* clone() { return new LGNodeArr(type, inputSize, A); }
		virtual int64* get_output(const char& name) {if (name=='o')return &o; return 0;}
		virtual int64** get_binding(const char& name);
		virtual ~LGNodeArr(){
			delete[] input_array;
		};
	};

	typedef std::vector<LGNode*> lg_container;
	typedef lg_container::iterator lg_iterator;

	class LeveledGraph {

	public:

		lg_container inputs;
		lg_container outputs;
		lg_container nodes;
		int max_level;
		std::vector<lg_iterator> levels;

		bool sanity_check();

		void add_input(LGNode* in){
			inputs.push_back(in);
		}

		void add_output(LGNode* out){
			outputs.push_back(out);
		}

		lg_container::iterator get_inputs_begin(){return inputs.begin();}
		lg_container::iterator get_inputs_end(){return inputs.end();}

		lg_container::iterator get_outputs_begin(){return outputs.begin();}
		lg_container::iterator get_outputs_end(){return outputs.end();}
	};

}

#endif /* LG_H_ */
