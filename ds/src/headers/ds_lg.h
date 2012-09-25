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

	class LGNode {
	public:
		ds_structural::Gate *gate;
		std::vector<LGNode*> outputs;
		std::vector<LGNode*> inputs;
		LGNode(const std::string &t):gate(0),outputs(0),inputs(0),endpoint(false),type(t){}
		virtual void sim()=0;
		virtual void propagate()=0;
		virtual int64* getBinding(const char& name) const {return 0;}
		virtual LGNode* clone()=0;
		virtual ~LGNode(){};
		std::string getType() const {return type;}

	protected:
		bool endpoint;
		std::string type;
	};

	class Input : public LGNode {
	protected:
		int64 *a;
		int64 o;
		int64 shadow;
	public:
		virtual void sim() { o = *a;}
		virtual void propagate() { sim(); }
		Input():LGNode("input"){};
		virtual LGNode* clone() { return new Input();}
		virtual int64* getBinding(const char& name){
			switch(name){
			case 'a': return a;
			case 'o': return &o;
			default: return 0;
			}
		}
	};

	class Output : public Input {
		Output():Input(){ type = "output"; endpoint = true;};
		virtual LGNode* clone() { return new Output();}
	};

	class LGNode1I : public LGNode {
	protected:
		int64 *a;
		int64 o;
		int64 shadow;
		int64 (*A)(int64_cpc a);
	public:
		virtual void sim() { o = (*A)(a);}
		virtual void propagate() { sim(); }
		LGNode1I(const std::string& t, int64 (*AA)(int64_cpc)):LGNode(t), A(AA){};
		virtual LGNode* clone() { return new LGNode1I(type, A);}
		virtual int64* getBinding(const char& name);
	};

	class LGNode2I : public LGNode {
	protected:
		int64 * a;
		int64 * b;
		int64 o;
		int64 shadow;
		int64 (*A)(int64_cpc a, int64_cpc b);
	public:
		virtual void sim() { o = (*A)(a,b);}
		virtual void propagate() { sim(); }
		LGNode2I(const std::string& t, int64 (*AA)(int64_cpc, int64_cpc)):LGNode(t), A(AA){};
		virtual LGNode* clone() { return new LGNode2I(type, A); }
		virtual int64* getBinding(const char& name);
	};

	class LGNode3I : public LGNode {
	protected:
		int64 * a;
		int64 * b;
		int64 * c;
		int64 o;
		int64 shadow;
		int64 (*A)(int64_cpc a, int64_cpc b, int64_cpc c);
	public:
		virtual void sim() { o = (*A)(a,b,c);}
		virtual void propagate() { sim(); }
		LGNode3I(const std::string& t, int64 (*AA)(int64_cpc, int64_cpc, int64_cpc c)):LGNode(t), A(AA){};
		virtual LGNode* clone() { return new LGNode3I(type, A); }
		virtual int64* getBinding(const char& name);
	};

	class LGNode4I : public LGNode {
	protected:
		int64 * a;
		int64 * b;
		int64 * c;
		int64 * d;
		int64 o;
		int64 shadow;
		int64 (*A)(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d);
	public:
		virtual void sim() { o = (*A)(a,b,c,d);}
		virtual void propagate() { sim(); }
		LGNode4I(const std::string& t, int64 (*AA)(int64_cpc, int64_cpc, int64_cpc, int64_cpc)):LGNode(t), A(AA){};
		virtual LGNode* clone() { return new LGNode4I(type, A); }
		virtual int64* getBinding(const char& name);
	};

	class LGNode5I : public LGNode {
	protected:
		int64 * a;
		int64 * b;
		int64 * c;
		int64 * d;
		int64 * e;
		int64 o;
		int64 shadow;
		int64 (*A)(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e);
	public:
		virtual void sim() { o = (*A)(a,b,c,d,e);}
		virtual void propagate() { sim(); }
		LGNode5I(const std::string& t, int64 (*AA)(int64_cpc, int64_cpc, int64_cpc, int64_cpc, int64_cpc)):LGNode(t), A(AA){};
		virtual LGNode* clone() { return new LGNode5I(type, A); }
		virtual int64* getBinding(const char& name);
	};

	class LGNode6I : public LGNode {
	protected:
		int64 * a;
		int64 * b;
		int64 * c;
		int64 * d;
		int64 * e;
		int64 * f;
		int64 o;
		int64 shadow;
		int64 (*A)(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e, int64_cpc f);
	public:
		virtual void sim() { o = (*A)(a,b,c,d,e,f);}
		virtual void propagate() { sim(); }
		LGNode6I(const std::string& t, int64 (*AA)(int64_cpc, int64_cpc, int64_cpc, int64_cpc, int64_cpc, int64_cpc)):LGNode(t), A(AA){};
		virtual LGNode* clone() { return new LGNode6I(type, A); }
		virtual int64* getBinding(const char& name);
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
		int64 o;
		int64 shadow;
		int64 (*A)(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e, int64_cpc f, int64_cpc g);
	public:
		virtual void sim() { o = (*A)(a,b,c,d,e,f,g);}
		virtual void propagate() { sim(); }
		LGNode7I(const std::string& t, int64 (*AA)(int64_cpc, int64_cpc, int64_cpc, int64_cpc, int64_cpc, int64_cpc, int64_cpc)):LGNode(t), A(AA){};
		virtual LGNode* clone() { return new LGNode7I(type, A); }
		virtual int64* getBinding(const char& name);
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
		int64 o;
		int64 shadow;
		int64 (*A)(int64_cpc a, int64_cpc b, int64_cpc c, int64_cpc d, int64_cpc e, int64_cpc f, int64_cpc g, int64_cpc h);
	public:
		virtual void sim() { o = (*A)(a,b,c,d,e,f,g,h);}
		virtual void propagate() { sim(); }
		LGNode8I(const std::string& t, int64 (*AA)(int64_cpc, int64_cpc, int64_cpc, int64_cpc, int64_cpc, int64_cpc, int64_cpc, int64_cpc)):LGNode(t), A(AA){};
		virtual LGNode* clone() { return new LGNode8I(type, A); }
		virtual int64* getBinding(const char& name);
	};

	class LGNodeArr : public LGNode {
	protected:
		int64 o;
		int inputSize;
		int64 (*A)(int64_cpc a, int64_cpc b);
		int64_cpc op;
		int64 ** input_array;
	public:
		virtual void sim() {
			o = **input_array;
			for (int i=0;i<inputSize;i++){
				o = (*A)(op, input_array[i]);
			}
		}
		virtual void propagate() { sim(); }
		LGNodeArr(const std::string& t, const int& inputs, int64 (*AA)(int64_cpc, int64_cpc)):LGNode(t), inputSize(inputs), A(AA), op(&o){
			input_array = new int64*[inputSize];
		};
		virtual LGNode* clone() { return new LGNodeArr(type, inputSize, A); }
		virtual int64* getBinding(const char& name);
		virtual ~LGNodeArr(){
			delete[] input_array;
		};
	};
}

#endif /* LG_H_ */
