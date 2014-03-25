/*
 * lg.h
 *
 *  Created on: Aug 26, 2012
 *      Author: cookao
 */

#ifndef DS_LG_H_
#define DS_LG_H_

#include "ds_common.h"
#include "ds_structural.h"
#include "ds_pattern.h"
#include <vector>
#include <unordered_map>
#include <boost/log/trivial.hpp>
#include <boost/array.hpp>


namespace ds_faults {
	struct SAFaultDescriptor;
}

namespace ds_lg {

	using ds_common::int64;
	using ds_common::lg_v64;

	typedef const lg_v64* const val64_cpc;
	typedef lg_v64 (*bi_eval)(val64_cpc, val64_cpc);

	/*
	 * logic primitive functions
	 */
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

	inline lg_v64 f_mux2(val64_cpc a, val64_cpc b, val64_cpc c){ return (*b & *c) | (*a & ~(*c));}

	/*!
	 * interface to observe any node simulation event.
	 */
	template<class T>
	class Monitor {
	public:
		/*!
		 * derived classes extend this method to log any interesting simulation event
		 * @param v current simulation value
		 */
		virtual void observe(const T& v) = 0;
	};

	/*!
	 * concrete class to capture the circuit's outputs in a pattern block
	 */
	class OutputObserver : public Monitor<lg_v64> {
	public:
		std::size_t offset;					//!< output offset in the pattern block
		ds_pattern::SimPatternBlock** pb;	//!< simulation pattern block
		/*!
		 * initializes public members
		 * @param output_offset offset
		 * @param pattern_block target pattern block
		 */
		OutputObserver(std::size_t output_offset, ds_pattern::SimPatternBlock** pattern_block):offset(output_offset){
			pb = pattern_block;
		}
		/*!
		 * write the simulation value at the output's offset in the pattern block
		 * @param v simulation value
		 */
		virtual void observe(const lg_v64& v) {
			(*pb)->values[offset].v = v.v;
			(*pb)->values[offset].x = v.x;
		}
	};
	template<class T>
	class Resolver {
	public:
		virtual T* get_node(const std::string& name) const=0;
		virtual std::string get_port_name(const std::string& node_name, const std::string& port_name) const=0;
	};
	/*!
	 * primitive node for logic simulation. It holds references to its input and output nodes.
	 * It holds the level assigned to the node and a reference to its equivalent gate in the netlist.
	 * A node may be an 'endpoint' if simulation events are not propagated to its output nodes
	 */
	template<class T>
	class SimulationHook{
	public:
		/*!
		 * activation condition for this fault. Derived classes implement this method to set the behavior of a fault
		 * @return 1 in bit position i if fault is active in slot i
		 */
	virtual ds_lg::int64 hook(ds_lg::Resolver<T> *res) const=0;
	/*!
	 * returns the port name in the leveled graph node where the fault is inserted
	 * @return
	 */
	virtual std::string get_hook_port() const=0;
	/*!
	 * returns the graph node that owns this hook. This is the node whose behavior is affected by this hook
	 * @return
	 */
	virtual T* get_hook_node(ds_lg::Resolver<T> *res) const=0;
	};

	template <class T, class V>
	class LGNode {
	public:
		int level;											//!< level in the graph
		ds_structural::Gate *gate;							//!< equivalent gate in netlist
		std::vector<V*> outputs;							//!< output nodes
		std::vector<V*> inputs;								//!< input nodes
		std::list<SimulationHook<V>*> hooks;
		/*!
		 * initializes public members. By default the node is not an endpoint.
		 * @param t node type
		 */
		LGNode(const std::string &t):level(-1),gate(0),endpoint(false),type(t),p(&o){}
		/*!
		 * sets the equivalent gate in the netlist
		 * @param g equivalent gate in the netlist
		 */
		void set_gate(ds_structural::Gate* g) {gate = g;}
		/*!
		 * return the equivalent gate in the netlist
		 */
		ds_structural::Gate* get_gate() const{return gate;}
		/*!
		 * Calculation of simulation values. Derived classes override this method to calculate the value of this node's outputs
		 * by evaluating this node's inputs
		 */
		virtual void sim()=0;
		/*!
		 * Calculation of simulation values with fault injection. Derived classes override this method to calculate the value of this node's outputs
		 * by evaluating this node's inputs and faults
		 */
		virtual void hook()=0;
		/*!
		 * apply all registered observers
		 */
		void observe(){

			for (auto it = monitors.begin();it!=monitors.end();it++){
				Monitor<T> *m = *it;
				m->observe(*p);		// log any simulation events
			}
		}
		/*!
		 * gets the primitive value address of an input port. If the provided name cannot be found null is returned
		 * @param name name of the port value
		 * @return address of the primitive value
		 */
		virtual T** get_input(const std::string& name)=0;
		/*!
		 * returns an output primitive value
		 * @param name output port name
		 * @return pointer to this node's output primitive value
		 */
		virtual T* get_output(const std::string& name){if (name=="o")return &o; return 0;};
		/*!
		 * Virtual destructor
		 */
		virtual ~LGNode(){};
		/*!
		 * returns node type
		 * @return any string may be returned
		 */
		std::string get_type() const {return type;}
		/*!
		 * true if the node's is a sequential element
		 * @return
		 */
		virtual bool has_state() {return false;}
		/*!
		 * true if node is a primary output
		 * @return
		 */
		bool is_endpoint() const {return endpoint;}
		/*!
		 * registers a monitor that is called on each simulation event
		 * @param m monitor to observe this node
		 */
		void add_monitor(Monitor<T>* m){monitors.push_back(m);}
		/*!
		 * removes the provided monitor from the monitor container
		 */
		void remove_monitor(Monitor<T>* m){
			auto m_it =std::find(monitors.begin(), monitors.end(), m);
			monitors.erase(m_it);
		}
		/*!
		 * removes all monitors in this node
		 */
		void remove_monitors(){
			monitors.clear();
		}
		/*!
		 * returns the current number of monitors this node has
		 * @return
		 */
		unsigned int num_monitors() const {return monitors.size();}
		/*!
		 * gets the node name. It returns the instance name of the corresponding gate in the netlist.
		 * If this gate does not exist it returns the empty string
		 * @return
		 */
		virtual std::string get_name() const {
			std::string name = gate !=0 ? gate->get_instance_name() : "";
			return name;
		}
		/*!
		 * reports the primitive value of this node's output
		 * @return a copy of the value of the node's output
		 */
		virtual T peek() const {return o;}
		/*!
		 * sets an internal pointer to the encompasing leveled graph
		 * @param graph
		 */
		void set_resolver(Resolver<V>* res) {
			resolver = res;
		}
		/*!
		 * adds a hook to be executed during the propagate method.
		 * This hook may or may not be registered in the encompassing leveled graph
		 * @param h
		 */
		void add_hook(SimulationHook<V>* h){
			hooks.push_back(h);
		}
		void remove_hook(SimulationHook<V>* h){
			auto it = std::find(hooks.begin(), hooks.end(), h);
			if (it!=hooks.end())
				hooks.remove(h);
		}
		/*!
		 * remove all hooks in this node
		 */
		void remove_hooks(){
			hooks.clear();
		}
	public:
		typedef typename std::vector<Monitor<T>*> monitor_container;
		T o; 						//!< node output
		bool endpoint;					//!< true if this node is an endpoint
		std::string type;				//!< node type
		T* p;
		monitor_container monitors;		//!< monitor container
		Resolver<V>* resolver;
		/*!
		 * Processes all hooks at the input ports. Commodity function
		 */
		void hook_inputs();
		/*!
		 * processes all hooks at the output ports. Commodity function
		 */
		void hook_outputs();
	};

	class LogicNode : public LGNode<lg_v64, LogicNode>{
	public:
	/*!
	 * initializes public members. By default the node is not an endpoint.
	 * @param t node type
	 */
	LogicNode(const std::string &t):LGNode<lg_v64,LogicNode>(t){}
	/*!
	 * revert output to previous value
	 */
	virtual void rollback(){o = bo;}
	/*!
	 * save the output value of the base simulation
	 */
	void mark() {bo = o;}
	/*!
	 * queries the output value of the base simulation
	 * @return
	 */
	lg_v64 get_mark() const {return bo;}
	/*!
	 * set output value to the complement of the simulation value
	 */
	virtual void flip(){o = ~bo;};
	/*!
	 * set output value to the complement of the simulation value and apply any attached observer
	 */
	void flip_and_observe(){
		flip();
		observe();
	}
	/*!
	 * Simulation primitives are created according to the prototype design pattern.
	 * Derived classes implement this method to replicate the node's structure and behavior
	 * @return a newly allocated node instance
	 */
	virtual LogicNode* clone() const=0;
	/*!
	* simulation procedure. The current output values is first stored, then new values are calculated.
	* Any attached observed is activated at this point. An event may propagated if any node output changed its value
	* @param intermediate intermediate simulation. Hooks are activated
	* @return true if an event is propagated to the output nodes
	*/
	virtual bool propagate(bool intermediate) {
		if (intermediate){
			if (hooks.size()!=0){
				hook();		// handle hooks and calculate new values
			}else{
				sim();
			}
		} else{
			sim();		// calculate new values
		}
		observe();
		if (!endpoint){
			ds_common::int64 result = ~o.x & (o.v ^ bo.v);
			return  result != 0;
		}
		return false;
	}
	protected:
		lg_v64 bo;						//!< backup node output
		/*!
		 * Processes all hooks at the input ports. Commodity function
		 */
		virtual void hook_inputs();
		/*!
		 * processes all hooks at the output ports. Commodity function
		 */
		virtual void hook_outputs();
	};

	/*!
	 * Simulation primitive for circuit outputs
	 */
	class Output : public LogicNode {
	protected:
		lg_v64 *a;				//!< single input
		std::string name;		//!< output name
	public:
		/*!
		 * sets the name of this output node, which is usually the port name in the netlist
		 * @param n output name
		 */
		void set_name(const std::string& n){name = n;}
		/*!
		 * returns the given output name
		 * @return
		 */
		virtual std::string get_name() const {return name;}
		/*!
		 * simulation value is forwarded form input to output
		 */
		virtual void sim() {
			o = *a;
		}
		/*!
		 * simulation value is forwarded form input to output. Faults are injected
		 */
		virtual void hook();
		/*!
		 * default values: this node is an endpoint and its type is initialized to "output"
		 */
		Output():LogicNode("output"){ endpoint = true;};
		/*!
		 * allocate new Output instance according to the prototype pattern
		 * @return
		 */
		virtual Output* clone() const { return new Output();}
		/*!
		 * only one input port available
		 * @param name output port name
		 * @return pointer to this node's input primitive value
		 */
		virtual lg_v64** get_input(const std::string& name){
			if (name == "a") return &a;
			return 0;
		}
	};

	/*!
	 * Simulation primitive for circuit inputs
	 */
	class Input : public LogicNode {
	protected:
		std::string name;					//!< input name
		std::size_t offset;					//!< offset position of this node in the pattern block
		ds_pattern::SimPatternBlock** pb;	//!< points current simulation pattern block
	public:
		/*!
		 * sets the name of this output node, which is usually the port name in the netlist
		 * @param n output name
		 */
		void set_name(const std::string& n){name = n;}
		/*!
		 * returns the given output name
		 * @return
		 */
		virtual std::string get_name() const {return name;}
		/*!
		 * copies the simulation values from the pattern block to this input
		 */
		virtual void sim() {
			o.v = (*pb)->values[offset].v;
			o.x = (*pb)->values[offset].x;
		}
		/*!
		 * simulation value is forwarded form input to output. Faults are injected
		 */
		virtual void hook();
		/*!
		 * default values: this node is an endpoint and its type is initialized to "input"
		 */
		Input():LogicNode("input"){};
		/*!
		 * allocate new Input instance according to the prototype pattern
		 * @return
		 */
		virtual Input* clone() const { return new Input();}
		/*!
		 * no inputs in this node
		 * @param name
		 * @return null
		 */
		virtual lg_v64** get_input(const std::string& name){
			return 0;
		}
		/*!
		 * sets the address of a pattern block out of which this input is evaluated
		 * @param pattern_block address of current pattern block to simulate
		 */
		void set_pattern_block(ds_pattern::SimPatternBlock** pattern_block){pb = pattern_block;}
		/*!
		 * sets the offset in the current pattern block corresponding to this input
		 * @param input_offset offset in pattern block
		 */
		void set_offset(const std::size_t input_offset){offset = input_offset;}
		/*!
		 * return the offset in the current pattern block corresponding to thie input
		 * @return
		 */
		std::size_t get_offset() const {return offset;}
	};

	/*!
	 * Simulation primitive for 1-input gates
	 */
	class LGNode1I : public LogicNode {
	protected:
		lg_v64 *a;					//!< single input
		lg_v64 (*A)(val64_cpc a);	//!< pointer to a function expecting 1 simulation value and returning 1 simulation value
	public:
		/*!
		 * Simulation primitive for single-input gates
		 */
		virtual void sim() {o = (*A)(a);}
		/*!
		 * calculate output value by evaluating simulation function with 1 input. Faults are injected
		 */
		virtual void hook();
		/*!
		 * By default this node is not an endpoint. The constructor initializes the node type and the function to execute during simulation
		 * @param t gate type
		 * @param AA simulation function
		 */
		LGNode1I(const std::string& t, lg_v64 (*AA)(val64_cpc)):LogicNode(t), A(AA){};
		/*!
		 * allocate new LGNode1I instance according to the prototype pattern
		 * @return
		 */
		virtual LogicNode* clone() const { return new LGNode1I(type, A);}
		/*!
		 * only one input port available
		 * @param name port name
		 * @return pointer to this node's input value. Null is name does not match.
		 */
		virtual lg_v64** get_input(const std::string& name);
	};
	/*!
	 * Simulation primitive for 2-input gates
	 */
	class LGNode2I : public LogicNode {
	protected:
		lg_v64 * a;								// two input nodes
		lg_v64 * b;								//
		lg_v64 (*A)(val64_cpc a, val64_cpc b);	//!< pointer to a function expecting 2 simulation value and returning 1 simulation value
	public:
		/*!
		 * calculate output value by evaluating simulation function with 2 inputs
		 */
		virtual void sim() {
			o = (*A)(a,b);
			return;
		}
		/*!
		 * calculate output value by evaluating simulation function with 2 inputs. Faults may be injected
		 */
		virtual void hook();
		/*!
		 * By default this node is not an endpoint. The constructor initializes the node type and the function to execute during simulation
		 * @param t gate type
		 * @param AA simulation function
		 */
		LGNode2I(const std::string& t, lg_v64 (*AA)(val64_cpc, val64_cpc)):LogicNode(t), A(AA){};
		/*!
		 * allocate new LGNode2I instance according to the prototype pattern
		 * @return
		 */
		virtual LogicNode* clone() const { return new LGNode2I(type, A); }
		/*!
		 * two input ports available
		 * @param name port name
		 * @return pointer to one of this node's input values. Null is name is unknown.
		 */
		virtual lg_v64** get_input(const std::string& name);
	};
	/*!
	 * Simulation primitive for 3-input gates
	 */
	class LGNode3I : public LogicNode {
	protected:
		lg_v64 * a;
		lg_v64 * b;
		lg_v64 * c;
		// pointer to a function expecting 3 simulation value and returning 1 simulation value
		lg_v64 (*A)(val64_cpc a, val64_cpc b, val64_cpc c);
	public:
		/*!
		 * calculate output value by evaluating simulation function with 3 inputs
		 */
		virtual void sim() {
			o = (*A)(a,b,c);
		}
		/*!
		 * calculate output value by evaluating simulation function with 3 inputs. Faults may be are injected
		 */
		virtual void hook();
		/*!
		 * By default this node is not an endpoint. The constructor initializes the node type and the function to execute during simulation
		 * @param t gate type
		 * @param AA simulation function
		 */
		LGNode3I(const std::string& t, lg_v64 (*AA)(val64_cpc, val64_cpc, val64_cpc c)):LogicNode(t), A(AA){};
		/*!
		 * allocate new LGNode3I instance according to the prototype pattern
		 * @return
		 */
		virtual LogicNode* clone() const { return new LGNode3I(type, A); }
		/*!
		 * three input ports available
		 * @param name port name
		 * @return pointer to one of this node's input values. Null is name is unknown.
		 */
		virtual lg_v64** get_input(const std::string& name);
	};
	/*!
	 * Simulation primitive for 4-input gates
	 */
	class LGNode4I : public LogicNode {
	protected:
		lg_v64 * a;
		lg_v64 * b;
		lg_v64 * c;
		lg_v64 * d;
		// pointer to a function expecting 4 simulation value and returning 1 simulation value
		lg_v64 (*A)(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d);
	public:
		/*!
		 * calculate output value by evaluating simulation function with 4 inputs
		 */
		virtual void sim() {o = (*A)(a,b,c,d);}
		/*!
		 * Simulation primitive for 4-input gates. Faults are injected
		 */
		virtual void hook();
		/*!
		 * By default this node is not an endpoint. The constructor initializes the node type and the function to execute during simulation
		 * @param t gate type
		 * @param AA simulation function
		 */
		LGNode4I(const std::string& t, lg_v64 (*AA)(val64_cpc, val64_cpc, val64_cpc, val64_cpc)):LogicNode(t), A(AA){};
		/*!
		 * allocate new LGNode4I instance according to the prototype pattern
		 * @return
		 */
		virtual LogicNode* clone() const { return new LGNode4I(type, A); }
		/*!
		 * four input ports available
		 * @param name port name
		 * @return pointer to one of this node's input values. Null is name is unknown.
		 */
		virtual lg_v64** get_input(const std::string& name);
	};
	/*!
	 * Simulation primitive for 5-input gates
	 */
	class LGNode5I : public LogicNode {
	protected:
		lg_v64 * a;
		lg_v64 * b;
		lg_v64 * c;
		lg_v64 * d;
		lg_v64 * e;
		// pointer to a function expecting 5 simulation value and returning 1 simulation value
		lg_v64 (*A)(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e);
	public:
		/*!
		 * calculate output value by evaluating simulation function with 5 inputs
		 */
		virtual void sim() { o = (*A)(a,b,c,d,e);}
		/*!
		 * Simulation primitive for 5-input gates. Faults are injected
		 */
		virtual void hook();
		/*!
		 * By default this node is not an endpoint. The constructor initializes the node type and the function to execute during simulation
		 * @param t gate type
		 * @param AA simulation function
		 */
		LGNode5I(const std::string& t, lg_v64 (*AA)(val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc)):LogicNode(t), A(AA){};
		/*!
		 * allocate new LGNode5I instance according to the prototype pattern
		 * @return
		 */
		virtual LogicNode* clone() const { return new LGNode5I(type, A); }
		/*!
		 * five input ports available
		 * @param name port name
		 * @return pointer to one of this node's input values. Null is name is unknown.
		 */
		virtual lg_v64** get_input(const std::string& name);
	private:
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & boost::serialization::base_object<LogicNode>(*this);
			ar & a;
			ar & b;
			ar & c;
			ar & d;
			ar & e;
			ar & A;
		}
	};
	/*!
	 * Simulation primitive for 6-input gates
	 */
	class LGNode6I : public LogicNode {
	protected:
		lg_v64 * a;
		lg_v64 * b;
		lg_v64 * c;
		lg_v64 * d;
		lg_v64 * e;
		lg_v64 * f;
		// pointer to a function expecting 6 simulation value and returning 1 simulation value
		lg_v64 (*A)(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e, val64_cpc f);
	public:
		/*!
		 * calculate output value by evaluating simulation function with 6 inputs
		 */
		virtual void sim() {o = (*A)(a,b,c,d,e,f);}
		/*!
		 * Simulation primitive for 6-input gates. Faults are injected
		 */
		virtual void hook();
		/*!
		 * By default this node is not an endpoint. The constructor initializes the node type and the function to execute during simulation
		 * @param t gate type
		 * @param AA simulation function
		 */
		LGNode6I(const std::string& t, lg_v64 (*AA)(val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc)):LogicNode(t), A(AA){};
		/*!
		 * allocate new LGNode6I instance according to the prototype pattern
		 * @return
		 */
		virtual LogicNode* clone() const { return new LGNode6I(type, A); }
		/*!
		 * six input ports available
		 * @param name port name
		 * @return pointer to one of this node's input values. Null is name is unknown.
		 */
		virtual lg_v64** get_input(const std::string& name);
	};
	/*!
	 * Simulation primitive for 7-input gates
	 */
	class LGNode7I : public LogicNode {
	protected:
		lg_v64 * a;
		lg_v64 * b;
		lg_v64 * c;
		lg_v64 * d;
		lg_v64 * e;
		lg_v64 * f;
		lg_v64 * g;
		// pointer to a function expecting 7 simulation value and returning 1 simulation value
		lg_v64 (*A)(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e, val64_cpc f, val64_cpc g);
	public:
		/*!
		 * calculate output value by evaluating simulation function with 7 inputs
		 */
		virtual void sim() {o = (*A)(a,b,c,d,e,f,g);}
		/*!
		 * Simulation primitive for 7-input gates. Faults are injected
		 */
		virtual void hook();
		/*!
		 * By default this node is not an endpoint. The constructor initializes the node type and the function to execute during simulation
		 * @param t gate type
		 * @param AA simulation function
		 */
		LGNode7I(const std::string& t, lg_v64 (*AA)(val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc)):LogicNode(t), A(AA){};
		/*!
		 * allocate new LGNode7I instance according to the prototype pattern
		 * @return
		 */
		virtual LogicNode* clone() const { return new LGNode7I(type, A); }
		/*!
		 * seven input ports available
		 * @param name port name
		 * @return pointer to one of this node's input values. Null is name is unknown.
		 */
		virtual lg_v64** get_input(const std::string& name);
	};
	/*!
	 * Simulation primitive for 8-input gates
	 */
	class LGNode8I : public LogicNode {
	protected:
		lg_v64 * a;
		lg_v64 * b;
		lg_v64 * c;
		lg_v64 * d;
		lg_v64 * e;
		lg_v64 * f;
		lg_v64 * g;
		lg_v64 * h;
		// pointer to a function expecting 8 simulation value and returning 1 simulation value
		lg_v64 (*A)(val64_cpc a, val64_cpc b, val64_cpc c, val64_cpc d, val64_cpc e, val64_cpc f, val64_cpc g, val64_cpc h);
	public:
		/*!
		 * calculate output value by evaluating simulation function with 8 inputs
		 */
		virtual void sim() {o = (*A)(a,b,c,d,e,f,g,h);}
		/*!
		 * Simulation primitive for 8-input gates. Faults are injected
		 */
		virtual void hook();
		/*!
		 * By default this node is not an endpoint. The constructor initializes the node type and the function to execute during simulation
		 * @param t gate type
		 * @param AA simulation function
		 */
		LGNode8I(const std::string& t, lg_v64 (*AA)(val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc)):LogicNode(t), A(AA){};
		/*!
		 * allocate new LGNode8I instance according to the prototype pattern
		 * @return
		 */
		virtual LogicNode* clone() const { return new LGNode8I(type, A); }
		/*!
		 * eight input ports available
		 * @param name port name
		 * @return pointer to one of this node's input values. Null is name is unknown.
		 */
		virtual lg_v64** get_input(const std::string& name);
	};
	/*!
	 * Simulation primitive for variable-input gates
	 */
	class LGNodeArr : public LogicNode {
	protected:
		int input_size;								//!< number of inputs
		lg_v64 (*A)(val64_cpc a, val64_cpc b);		//!< pointer to a function expecting 2 simulation value and returning 1 simulation value
		lg_v64 ** input_array;						//!< input array
		bool invert;								//!< true if simulation result is inverted
	public:
		/*!
		 * calculate output value by evaluating an input array
		 */
		virtual void sim() {
			o = **input_array;
			for (int i=1;i<input_size;i++){
				o = (*A)(&o, input_array[i]);		//!< aggregate results by evaluating binary simulation function
			}
			if(invert)
				o = ~o;
		}
		/*!
		 * Simulation primitive for variable-input gates. Faults are injected
		 */
		virtual void hook();
		/*!
		 * by default this node is not an endpoint. The constructor initializes the node type, the simulation function, the number of inputs and the inversion flag.
		 * It also allocates space for the input array
		 * @param t node type
		 * @param inputs number of inputs
		 * @param AA binary simulation function
		 * @param iv inversion flag
		 */
		LGNodeArr(const std::string& t, const int& inputs, lg_v64 (*AA)(val64_cpc, val64_cpc), bool iv):LogicNode(t), input_size(inputs), A(AA), invert(iv){
			input_array = new lg_v64*[input_size];
		};
		/*!
		 * allocate new LGNodeArr instance according to the prototype pattern
		 * @return
		 */
		virtual LogicNode* clone() const { return new LGNodeArr(type, input_size, A, invert); }
		/*!
		 * returns an output primitive value. Variable-input gates are usually instantiated implicitly. To allow the most inputs without ambiguity, the output name is 'z'
		 * @param name output port name
		 * @return pointer to this node's output primitive value
		 */
		virtual lg_v64* get_output(const std::string& name) {if (name=="z")return &o; return 0;}
		/*!
		 * up to 24 input ports available
		 * @param name port name (from 'a' to 'y')
		 * @return pointer to one of this node's input values. Null is name is unknown.
		 */
		virtual lg_v64** get_input(const std::string& name);
		/*!
		 * deallocates input array
		 */
		virtual ~LGNodeArr(){
			delete[] input_array;
		};
	};
	/*!
	 * Simulation primitive for generic sequential gates
	 */
	class LogicState : public LogicNode {
	protected:
		lg_v64 * d;		//!< data input
		lg_v64 * cd;	//!< clk input
		lg_v64 on;		//!< not Q output
		lg_v64 * rst;
		lg_v64 * rst_n;
		lg_v64 * load;
		lg_v64 * load_n;
		lg_v64 one;
		lg_v64 zero;
	public:

		LogicState(const std::string& t):LogicNode(t), one(-1L,0), zero(0,0){
			endpoint = true;
			rst = &zero;
			rst_n = &one;
			load = &zero;
			load_n = &one;
		}
		/*!
		 * update outputs when clk event is active
		 */
		virtual void sim() {
			o = (o & ~*rst & *rst_n) | (*load | ~*load_n);
			on = ~o;
		}
		virtual void tick(){
			o = *d;
			//o = (o & ~*rst & *rst_n) | (*load | ~*load_n);
			on = ~o;
		}
		/*!
		 * Simulation primitive state gates. Faults are injected
		 */
		virtual void hook();
		/*!
		 * revert output to previous value
		 */
		virtual void rollback(){o = bo; on = ~bo;}
		virtual void flip(){o = ~o; on = o;};
		/*!
		 * allocate new LGState instance according to the prototype pattern
		 * @return
		 */
		virtual LogicNode* clone() const { return new LogicState(type); }
		virtual LogicState* clone_register() const { return new LogicState(type); }
		/*!
		 * returns an output primitive value, either Q or QN
		 * @param name output port name
		 * @return pointer to the node's requested output primitive value
		 */
		virtual lg_v64* get_output(const std::string& name) {
			if (name=="Q")return &o;
			if (name=="QN")return &on;
			return 0;}
		/*!
		 * two input ports available
		 * @param name port name
		 * @return pointer to one of this node's input values (D or CD). Null is name is unknown.
		 */
		virtual lg_v64** get_input(const std::string& name);
		/*!
		 * by definition this node has state
		 * @return
		 */
		virtual bool has_state() {return true;}
		/*!
		 * Virtual destructor
		 */
		virtual ~LogicState(){};
	};

	template<class R>
	class ClockPublisher {
		std::string clk_name;
		std::vector<R*> subscribers;
	public:
		ClockPublisher(const std::string& name):clk_name(name){}
		void add_subscriber(R* s){subscribers.push_back(s);}
		void tick(){
			for (R* s: subscribers){
				s->tick();
			}
		}
	};

	/*!
	 *
	 */
	template<class N, class R, class I, class O, class V>
	class GenericLeveledGraph : public Resolver<N>{

		typedef typename std::vector<N*> lg_node_container;
		typedef typename lg_node_container::iterator lg_node_iterator;
		typedef typename std::vector<I*> lg_input_container;
		typedef typename std::vector<I*>::iterator lg_input_iterator;
		typedef typename std::vector<O*> lg_output_container;
		typedef typename std::vector<O*>::iterator lg_output_iterator;
		typedef typename std::vector<ClockPublisher<R> > lg_clock_container;
		typedef typename std::vector<R*> lg_register_container;


	public:

		lg_input_container inputs;		//!< input ports
		lg_output_container outputs;	//!< output ports
		lg_node_container nodes;		//!< all nodes
		lg_register_container registers;	//!< all sequential elements
		lg_clock_container clocks;
		std::list<SimulationHook<N>*> hooks;			//!< all active hooks

		virtual void set_netlist(ds_structural::NetList *netlist) {
			nl = netlist;
		}
		virtual void set_levels(const int& l){
			num_levels = l;
		}
		virtual void create_simulation_level(const std::size_t& levels){
			for (std::size_t l=0;l<levels;l++){
				simulation.push_back(new lg_node_container());
			}
		}
		virtual lg_node_iterator get_nodes_begin(){
			return nodes.begin();
		}
		virtual lg_node_iterator get_nodes_end(){
			return nodes.end();
		}
		virtual void push_level_iterator(lg_node_iterator it){
			levels.push_back(it);
		}
		virtual lg_node_iterator get_level_iterator(const std::size_t& level){
			return levels[level];
		}
		virtual void push_level_width(const std::size_t& width){
			level_width.push_back(width);
		}
		virtual V* get_constant_0(){
			return &constant_0;
		}
		virtual V* get_constant_1(){
			return &constant_1;
		}
		virtual void add_register(R* r){
			registers.push_back(r);
			registry[r->get_name()] = r;
			r->set_resolver(this);
		}
		virtual void add_node(N* n){
			nodes.push_back(n);
			registry[n->get_name()] = n;
			n->set_resolver(this);
		}
		/*!
		 * verify the internal structure of the graph
		 * @return true if check is successful
		 */
		bool sanity_check(){
			bool c = true;
			if (nodes.size() <= 0){
				c = false;
			}
			for (N *n: nodes)
			{
				if (n->level >= num_levels){
					c = false;
					BOOST_LOG_TRIVIAL(error) << "level " << n->level << " > " << num_levels;
				}

				for (N *o: n->outputs)
				{
					if (!o->has_state())
						if (n->level >= o->level)
							c = false;
				}
			}

			for (int i=0;i<num_levels;i++){
				unsigned int cnt = 0;
				std::for_each(nodes.begin(), nodes.end(),
					[&] (N* n) { if (n->level == i) cnt++;}
				);
				if (cnt != level_width[i]){
					c = false;
					BOOST_LOG_TRIVIAL(error) << "Inconsistent level (" << i <<") size: "<< cnt << "!=" << level_width[i];
				}
			}
			return c;
		}
		/*!
		 * adds an input port
		 * @param in input port
		 */
		void add_input(I* in){
			inputs.push_back(in);
			registry[in->get_name()] = in;
		}
		/*!
		 * adds an output port
		 * @param out output port
		 */
		void add_output(O* out){
			outputs.push_back(out);
			registry[out->get_name()] = out;
		}
		/*
		 * various iterators for input and output traversal
		 */
		lg_input_iterator get_inputs_begin(){return inputs.begin();}
		lg_input_iterator get_inputs_end(){return inputs.end();}
		lg_output_iterator get_outputs_begin(){return outputs.begin();}
		lg_output_iterator get_outputs_end(){return outputs.end();}
		/*!
		 * adds a new hook to be executed during logic simulation
		 * @param hook
		 */
		void add_hook(SimulationHook<N>* hook){
			hooks.push_back(hook);
			N *node = hook->get_hook_node(this);
			node->add_hook(hook);
			push_node(node);
		}

		/*!
		 * removes all active hooks
		 */
		void clear_hooks(){
			for (auto it=hooks.begin();it!=hooks.end();it++){
				SimulationHook<N> *hook = *it;
				N *node = hook->get_hook_node(this);
				node->remove_hooks();
			}
			hooks.clear();
		}

		void clear_hook(SimulationHook<N>* hook){
			hooks.remove(hook);
			N *node = hook->get_hook_node(this);
			node->remove_hook(hook);
		}
		/*!
		 * push node into event simulation node list
		 * @param node node to include
		 */
		void push_node(N *node){
			simulation[node->level]->push_back(node);
		}
		/*!
		 * returns the check point of any given node:
		 * It returns the argument node pointer is an input, output or fan out node, otherwise
		 * the argument's fan out node is returned
		 * @param n node whose check point is requested
		 * @return
		 */
		N* get_check_point(N* n){
			N *fo = n;
			if (fo->outputs.size()!=0){
				while (fo->outputs.size()==1 && !fo->is_endpoint()){
					fo = fo->outputs[0];
				}
			}
			return fo;
		}
		/*!
		 * returns a pointer to the netlist associated with this graph
		 * @return
		 */
		ds_structural::NetList* get_netlist() const {return nl;}

		virtual N* get_node(const std::string& name) const{
			auto it = registry.find(name);
			if(it!=registry.end())
				return it->second;
			return 0;
		}

		virtual std::string get_port_name(const std::string& node_name, const std::string& port_name) const{
			ds_structural::Gate *g = nl->find_gate(node_name);
			return g->get_mapping(port_name);
		}

		template<class IT>
		void add_publisher(const std::string& name, IT begin, IT end){
			ClockPublisher<R> p(name);
			for (IT it=begin; it!=end; it++){
				R *s = *it;
				p.add_subscriber(s);
			}
			clocks.push_back(p);
		}

		/*!
		 * @param h hook to propagate
		 * @return for each pattern / fault,
		 * returns 1 in position i if the hook can be propagated to its check point in the ith slot
		 */
		ds_common::int64 propagate_to_check_point(SimulationHook<N> *h){
			N *n = h->get_hook_node(this);
			N* node = get_check_point(n);
			n->add_hook(h);
			V ff = node->get_mark();
			std::vector<N*> path;
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
			V faulty = n->peek();


			for (N *p:path){
				p->rollback();
			}
			if (n!=node){
				//std::cout << "stopping at: " << n->get_name() << " " << node->get_name() << std::endl;
				return 0;
			}

			ds_common::int64 result = resolve(ff, faulty);
			return result;
		}

		ds_common::int64 activate(SimulationHook<N> *h){
			ds_common::int64 activation = 0;
			N *n = h->get_hook_node(this);
			V* p_port = n->get_output(h->get_hook_port());
			if (p_port!=0){
				activation = h->hook(this);
			}
			n->remove_hook(h);
			return activation;
		}

		virtual void sim(ds_pattern::SimPatternBlock *pb){
			pattern_block = pb;
			//propagates events from inputs to outputs
			for (auto it=nodes.begin();it!=nodes.end();it++){
				N *n = *it;
				n->propagate(false);
				n->mark();
			}
			//inject hooks
			for (SimulationHook<N>* h: hooks){
				N *node = h->get_hook_node(this);
				push_node(node);
			}
			sim_intermediate();
		}

		virtual void sim_intermediate(){
			std::set<N*> set;
			std::vector<N*> regs;
			for (int i=0;i<num_levels;i++){
				lg_node_container* level = simulation[i];
				iteration++;
				for (auto it=level->begin();it!=level->end();it++){
					N *n = *it;
					bool p = n->propagate(true);
					if (n->has_state()){
						regs.push_back(n);
					}
					if (p){
						for (N *o : n->outputs){
							auto s = set.find(o);
							if (s==set.end()){
								push_node(o);
								set.insert(o);
							}
						}
					}
				}
			}

			for (auto it=regs.begin();it!=regs.end();it++){
				N *n = *it;
				n->propagate(true);
			}

			for (int i=0;i<num_levels;i++){
				lg_node_container* level = simulation[i];
				for (auto it=level->begin();it!=level->end();it++){
					N *n = *it;
					n->rollback();
				}
				level->clear();
			}
		}


	protected:
		int num_levels;										//!< number of levels in the graph
		ds_pattern::SimPatternBlock *pattern_block; 		//!< current pattern block for simulation
		std::vector<lg_node_iterator> levels;				//!< level iterators. Two iterators define the nodes in a level
		std::vector<lg_node_container*> simulation;			//!< nodes to evaluate during intermediate simulation
		std::vector<unsigned int> level_width;				//!< number of nodes per level
		V constant_0;
		V constant_1;
		V constant_X;
		ds_structural::NetList *nl;							//!< parent netlist
		std::unordered_map<std::string, N*> registry;			//!< node map indexed by node instance name
		double iteration;
		virtual ds_common::int64 resolve(const V& ff, const V& faulty) const = 0;
	};

	class LeveledGraph : public GenericLeveledGraph<LogicNode,LogicState,Input,Output,lg_v64>{
		typedef std::vector<LogicNode*> lg_node_container;
	public:

		LeveledGraph(){
			constant_0 = lg_v64(0L,0L);
			constant_1 = lg_v64(-1L,0L);
			constant_X = lg_v64(0L,-1L);
		}

		void adapt(const ds_pattern::CombinationalPatternAdapter* adapter);

	protected:

		ds_common::int64 resolve(const lg_v64& ff, const lg_v64& faulty) const {
			return ~ff.x & ~faulty.x & (ff.v ^ faulty.v);
		}

	};

	class TNode;

	struct driver_v64 {
		lg_v64 value;
		TNode* driver;
		short port_id;

		driver_v64():value(0L,-1L),driver(0),port_id(-1){}

		driver_v64(const lg_v64& v):value(v),driver(0), port_id(-1){}

		driver_v64(const lg_v64& v, TNode* node, const short id):value(v),driver(node),port_id(id){}

		driver_v64(const ds_common::int64 v, const ds_common::int64 x):value(v,x), driver(0), port_id(-1){}

		driver_v64(const ds_common::int64 v, const ds_common::int64 x, TNode* node, const short id):value(v,x), driver(node), port_id(id){}

		driver_v64(const driver_v64& val): value(val.value), driver(val.driver), port_id(val.port_id){}

		driver_v64 operator~() const{
			lg_v64 o;
			o.v = ~value.v & ~value.x;
			o.x = value.x;
			return o;
		}

		driver_v64& operator=(const driver_v64 &rhs) {
			if (this != &rhs) {
				value.v = rhs.value.v;
				value.x = rhs.value.x;
			}

			return *this;
		}

		driver_v64& operator&=(const driver_v64& rhs){
			value.v &= rhs.value.v;
			value.x = (value.x & ~rhs.value.x & rhs.value.v) | (rhs.value.x &  ~value.x & value.v);
			return *this;
		}
		/*!
		 * unary 'or'
		 * @param rhs 'or' operand
		 * @return logic 'or' between this and provided operand
		 */
		driver_v64& operator|=(const driver_v64& rhs){
			value.v |= rhs.value.v;
			value.x = (value.x & ~rhs.value.x & ~rhs.value.v) | (rhs.value.x & value.x & ~value.v );
			return *this;
		}
		/*!
		 * unary 'xor'
		 * @param rhs 'xor' operand
		 * @return logic 'xor' between this and provided operand
		 */
		driver_v64& operator^=(const driver_v64& rhs){
			value.v ^= rhs.value.v;
			value.x = rhs.value.x | value.x;
			return *this;
		}
		private:
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version){
			ar & value.v;
			ar & value.x;
			ar & driver;
		}

	};

	inline driver_v64 operator&(driver_v64 lhs, const driver_v64& rhs){
		lhs &= rhs;
		return lhs;
	}

	inline driver_v64 operator|(driver_v64 lhs, const driver_v64& rhs){
		lhs |= rhs;
		return lhs;
	}

	inline driver_v64 operator^(driver_v64 lhs, const driver_v64& rhs){
		lhs ^= rhs;
		return lhs;
	}

	class TOutputObserver : public Monitor<driver_v64> {
	public:
		std::size_t offset;					//!< output offset in the pattern block
		ds_pattern::SimPatternBlock** pb;	//!< simulation pattern block
		std::size_t *voffset;
		/*!
		 * initializes public members
		 * @param output_offset offset
		 * @param pattern_block target pattern block
		 */
		TOutputObserver(std::size_t output_offset, ds_pattern::SimPatternBlock** pattern_block, std::size_t *vector_offset):offset(output_offset), pb(pattern_block), voffset(vector_offset){}
		/*!
		 * write the simulation value at the output's offset in the pattern block
		 * @param v simulation value
		 */
		virtual void observe(const driver_v64& v) {
			int o = *voffset + offset;
			(*pb)->values[o].v = v.value.v;
			(*pb)->values[o].x = v.value.x;
		}
	};
	class TNode : public LGNode<driver_v64, TNode>{
	public:
	/*!
	 * initializes public members. By default the node is not an endpoint.
	 * @param t node type
	 */
	TNode(const std::string &t):LGNode<driver_v64,TNode>(t){
		o.value = lg_v64(0L,-1L);
		o.driver = this;
		o.port_id = 0;
	}
	/*!
	 * revert output to previous value
	 */
	virtual void rollback(){
		o.value = bo;
	}
	/*!
	 * save the output value of the base simulation
	 */
	void mark() {bo = o.value;}
	/*!
	 * queries the output value of the base simulation
	 * @return
	 */
	lg_v64 get_mark() const {return bo;}
	/*!
	 * set output value to the complement of the simulation value
	 */
	virtual void flip(){o.value = ~bo;};
	/*!
	 * set output value to the complement of the simulation value and apply any attached observer
	 */
	void flip_and_observe(){
		flip();
		observe();
	}
	void virtual mark_clock_cycle(){
		previous = o.value;
	}
	/*!
	 * Simulation primitives are created according to the prototype design pattern.
	 * Derived classes implement this method to replicate the node's structure and behavior
	 * @return a newly allocated node instance
	 */
	virtual TNode* clone() const=0;
	/*!
	* simulation procedure. The current output values is first stored, then new values are calculated.
	* Any attached observed is activated at this point. An event may propagated if any node output changed its value
	* @param intermediate intermediate simulation. Hooks are activated
	* @return true if an event is propagated to the output nodes
	*/
	virtual bool propagate(bool intermediate) {
		if (intermediate){
			if (hooks.size()!=0){
				hook();		// handle hooks and calculate new values
			}else{
				sim();
			}
		} else{
			sim();		// calculate new values
		}
		observe();
		if (!endpoint){
			ds_common::int64 result = ~o.value.x & (o.value.v ^ bo.v);
			return  result != 0;
		}
		return false;
	}

	virtual const lg_v64* get_previous_value(const short id) const {
		if (id == 0){
			return &previous;
		}
		return 0;
	}

	protected:
		lg_v64 bo;						//!< backup node output
		lg_v64 previous;
		/*!
		 * Processes all hooks at the input ports. Commodity function
		 */
		virtual void hook_inputs();
		/*!
		 * processes all hooks at the output ports. Commodity function
		 */
		virtual void hook_outputs();
	};

	/*!
	 * Simulation primitive for circuit outputs
	 */
	class TOutput : public TNode {
	protected:
		driver_v64 *a;				//!< single input
		std::string name;		//!< output name
	public:
		/*!
		 * sets the name of this output node, which is usually the port name in the netlist
		 * @param n output name
		 */
		void set_name(const std::string& n){name = n;}
		/*!
		 * returns the given output name
		 * @return
		 */
		virtual std::string get_name() const {return name;}
		/*!
		 * simulation value is forwarded form input to output
		 */
		virtual void sim() {
			o.value = a->value;
		}
		/*!
		 * simulation value is forwarded form input to output. Faults are injected
		 */
		virtual void hook();
		/*!
		 * default values: this node is an endpoint and its type is initialized to "output"
		 */
		TOutput():TNode("output"){ endpoint = true;};
		/*!
		 * allocate new Output instance according to the prototype pattern
		 * @return
		 */
		virtual TOutput* clone() const { return new TOutput();}
		/*!
		 * only one input port available
		 * @param name output port name
		 * @return pointer to this node's input primitive value
		 */
		virtual driver_v64** get_input(const std::string& name){
			if (name == "a") return &a;
			return 0;
		}
	};

	/*!
	 * Simulation primitive for circuit inputs
	 */
	class TInput : public TNode {
	protected:
		std::string name;					//!< input name
		std::size_t offset;					//!< offset position of this node in the pattern block
		std::size_t *vector_offset;
		ds_pattern::SimPatternBlock** pb;	//!< points current simulation pattern block
	public:
		/*!
		 * sets the name of this output node, which is usually the port name in the netlist
		 * @param n output name
		 */
		void set_name(const std::string& n){name = n;}
		/*!
		 * returns the given output name
		 * @return
		 */
		virtual std::string get_name() const {return name;}
		/*!
		 * copies the simulation values from the pattern block to this input
		 */
		virtual void sim() {
			std::size_t idx = offset + *vector_offset;
			o.value.v = (*pb)->values[idx].v;
			o.value.x = (*pb)->values[idx].x;
		}
		/*!
		 * simulation value is forwarded form input to output. Faults are injected
		 */
		virtual void hook();
		/*!
		 * default values: this node is an endpoint and its type is initialized to "input"
		 */
		TInput():TNode("input"),vector_offset(0){};
		/*!
		 * allocate new Input instance according to the prototype pattern
		 * @return
		 */
		virtual TInput* clone() const { return new TInput();}
		/*!
		 * no inputs in this node
		 * @param name
		 * @return null
		 */
		virtual driver_v64** get_input(const std::string& name){
			return 0;
		}
		/*!
		 * sets the address of a pattern block out of which this input is evaluated
		 * @param pattern_block address of current pattern block to simulate
		 */
		void set_pattern_block(ds_pattern::SimPatternBlock** pattern_block){pb = pattern_block;}
		/*!
		 * sets the offset in the current pattern block corresponding to this input
		 * @param input_offset offset in pattern block
		 */
		void set_offset(const std::size_t input_offset){offset = input_offset;}
		/*!
		 * return the offset in the current pattern block corresponding to thie input
		 * @return
		 */
		std::size_t get_offset() const {return offset;}

		void set_vector_offset(std::size_t* vo){
			vector_offset = vo;
		}
	};

	/*!
	 * Simulation primitive for 1-input gates
	 */
	class TNode1I : public TNode {
	protected:
		driver_v64 *a;					//!< single input
		lg_v64 (*A)(val64_cpc a);	//!< pointer to a function expecting 1 simulation value and returning 1 simulation value
	public:
		/*!
		 * Simulation primitive for single-input gates
		 */
		virtual void sim() {
			o.value = (*A)(&a->value);
		}
		/*!
		 * calculate output value by evaluating simulation function with 1 input. Faults are injected
		 */
		virtual void hook();
		/*!
		 * By default this node is not an endpoint. The constructor initializes the node type and the function to execute during simulation
		 * @param t gate type
		 * @param AA simulation function
		 */
		TNode1I(const std::string& t, lg_v64 (*AA)(val64_cpc)):TNode(t), A(AA){};
		/*!
		 * allocate new LGNode1I instance according to the prototype pattern
		 * @return
		 */
		virtual TNode* clone() const { return new TNode1I(type, A);}
		/*!
		 * only one input port available
		 * @param name port name
		 * @return pointer to this node's input value. Null is name does not match.
		 */
		virtual driver_v64** get_input(const std::string& name);
	};
	/*!
	 * Simulation primitive for 2-input gates
	 */
	class TNode2I : public TNode {
	protected:
		driver_v64 * a;								// two input nodes
		driver_v64 * b;								//
		lg_v64 (*A)(val64_cpc a, val64_cpc b);	//!< pointer to a function expecting 2 simulation value and returning 1 simulation value
	public:
		/*!
		 * calculate output value by evaluating simulation function with 2 inputs
		 */
		virtual void sim() {
			o.value = (*A)(&a->value,&b->value);
			return;
		}
		/*!
		 * calculate output value by evaluating simulation function with 2 inputs. Faults may be injected
		 */
		virtual void hook();
		/*!
		 * By default this node is not an endpoint. The constructor initializes the node type and the function to execute during simulation
		 * @param t gate type
		 * @param AA simulation function
		 */
		TNode2I(const std::string& t, lg_v64 (*AA)(val64_cpc, val64_cpc)):TNode(t), A(AA){};
		/*!
		 * allocate new LGNode2I instance according to the prototype pattern
		 * @return
		 */
		virtual TNode* clone() const { return new TNode2I(type, A); }
		/*!
		 * two input ports available
		 * @param name port name
		 * @return pointer to one of this node's input values. Null is name is unknown.
		 */
		virtual driver_v64** get_input(const std::string& name);
	};
	/*!
	 * Simulation primitive for 3-input gates
	 */
	class TNode3I : public TNode {
	protected:
		driver_v64 * a;
		driver_v64 * b;
		driver_v64 * c;
		// pointer to a function expecting 3 simulation value and returning 1 simulation value
		lg_v64 (*A)(val64_cpc a, val64_cpc b, val64_cpc c);
	public:
		/*!
		 * calculate output value by evaluating simulation function with 3 inputs
		 */
		virtual void sim() {
			o.value = (*A)(&a->value,&b->value,&c->value);
		}
		/*!
		 * calculate output value by evaluating simulation function with 3 inputs. Faults may be are injected
		 */
		virtual void hook();
		/*!
		 * By default this node is not an endpoint. The constructor initializes the node type and the function to execute during simulation
		 * @param t gate type
		 * @param AA simulation function
		 */
		TNode3I(const std::string& t, lg_v64 (*AA)(val64_cpc, val64_cpc, val64_cpc c)):TNode(t), A(AA){};
		/*!
		 * allocate new LGNode3I instance according to the prototype pattern
		 * @return
		 */
		virtual TNode* clone() const { return new TNode3I(type, A); }
		/*!
		 * three input ports available
		 * @param name port name
		 * @return pointer to one of this node's input values. Null is name is unknown.
		 */
		virtual driver_v64** get_input(const std::string& name);
	};

	class TState : public TNode {
	public:
		driver_v64 * d;		//!< data input
		driver_v64 * cd;	//!< clk input
		driver_v64 o_n;		//!< not Q output
		driver_v64 o_int;	//!< intermediate value
		lg_v64 previous_n;
		driver_v64 * rst;
		driver_v64 * rst_n;
		driver_v64 * load;
		driver_v64 * load_n;
		driver_v64 * si;
		driver_v64 * se;
		driver_v64 one;
		driver_v64 zero;
		ds_pattern::SimPatternBlock** pb;
		std::size_t offset;
		boost::array<const lg_v64*,2> output_array;


	public:
		TState(const std::string& t):TNode(t), o_n(0,-1L,this,1), o_int(0,-1L,this,1), one(-1L,0,this,-1), zero(0,0,this,-1){
			rst = &zero;
			rst_n = &one;
			load = &zero;
			load_n = &one;
			output_array[0] = &previous;
			output_array[1] = &previous_n;
			endpoint = true;
			p = &o_int;
		}

		virtual driver_v64 peek() const {return o_int;}

		/*!
		 * update outputs when clk event is active
		 */
		virtual void sim() {
			o_int.value = d->value;
		}

		virtual void tick(){
			mark_clock_cycle();
			o.value = d->value;
			o_n.value = ~o.value;
			bo = o.value;
			std::cout<< get_name() << std::hex << "previous: " << previous.v << " current: " << o.value.v << std::endl;
		}
		/*!
		 * Simulation primitive state gates. Faults are injected
		 */
		virtual void hook();
		/*!
		 * revert output to previous value
		 */
		virtual void rollback(){
			o.value = bo;
			o_n.value = ~o.value;
			o_int.value = o.value;
		}
		virtual void flip(){
			o.value = ~o.value;
			o_n.value = ~o_n.value;
			//o_int.value = o.value;
		};

		/*!
		 * allocate new LGState instance according to the prototype pattern
		 * @return
		 */
		virtual TNode* clone() const { return new TState(type); }
		virtual TState* clone_register() const { return new TState(type); }
		/*!
		 * returns an output primitive value, either Q or QN
		 * @param name output port name
		 * @return pointer to the node's requested output primitive value
		 */
		virtual driver_v64* get_output(const std::string& name) {
			if (name=="q")return &o;
			if (name=="qn")return &o_n;
			return 0;}
		/*!
		 * two input ports available
		 * @param name port name
		 * @return pointer to one of this node's input values (D or CD). Null is name is unknown.
		 */
		virtual driver_v64** get_input(const std::string& name);
		/*!
		 * by definition this node has state
		 * @return
		 */
		virtual bool has_state() {return true;}
		void scan(){
			o.value.v = (*pb)->values[offset].v;
			o.value.x = (*pb)->values[offset].x;
		}
		void set_offset(const std::size_t reg_offset){
			offset = reg_offset;
		}
		void set_pattern_block(ds_pattern::SimPatternBlock** pattern_block){
			pb = pattern_block;
		}
		void virtual mark_clock_cycle(){
			previous = o.value;
			previous_n = o_n.value;
		}
		virtual const lg_v64* get_previous_value(const short id) const {
			return output_array[id];
		}
		virtual TNode* get_driver_node() const {
			return d->driver;
		}
		virtual void hook_outputs();
		/*!
		 * Virtual destructor
		 */
		virtual ~TState(){};
	};

	class TLeveledGraph : public GenericLeveledGraph<TNode,TState,TInput,TOutput,driver_v64>{
		typedef std::vector<TNode*> lg_node_container;

	public:

		TLeveledGraph():vector_offset(0){
			constant_0 = lg_v64(0L,0L);
			constant_1 = lg_v64(-1L,0L);
			constant_X = lg_v64(0L,-1L);
		}

	protected:

		std::size_t vector_offset;
		ds_common::int64 resolve(const driver_v64& ff, const driver_v64& faulty) const {
			ds_common::int64 diff = (ff.value.v ^ faulty.value.v);
			return ~ff.value.x & ~faulty.value.x & diff;
		}

	public:

		void setup();

		void adapt(const ds_pattern::SequentialPatternAdapter* adapter);

		void reset_offset(){
			vector_offset = 0;
		}
		void next_vector(){
			vector_offset += inputs.size() + outputs.size();
		}
		void scan(){
			for (auto it=registers.begin();it!=registers.end();it++){
				TState *n = *it;
				n->scan();
			}
		}

		virtual void sim(ds_pattern::SimPatternBlock *pb){
			pattern_block = pb;
			reset_offset();
			scan();

			//propagates events from inputs to outputs --> LAUNCH
			for (auto it=nodes.begin();it!=nodes.end();it++){
				TNode *n = *it;
				n->propagate(false);
				if (!n->has_state()){
					n->mark_clock_cycle();
				}
			}

			for (auto it=nodes.begin();it!=nodes.end();it++){
				TNode *n = *it;
				if (n->has_state()){
					n->propagate(false);
				}
			}

			TNode *n1 = get_node("U32432");
			TNode *ff = get_node("FF_15_17");

			std::cout << std::hex << "COMPARE " << n1->peek().value.v << "   " << ff->peek().value.v << std::endl;


			clocks[0].tick();

			next_vector();
			//propagates events from inputs to outputs --> CAPTURE
			for (auto it=nodes.begin();it!=nodes.end();it++){
				TNode *n = *it;
				n->propagate(false);
				if (!n->has_state()){
					n->mark();
				}
			}

			std::cout << std::hex << "COMPARE " << n1->peek().value.v << "   " << ff->peek().value.v << std::endl;

			for (auto it=nodes.begin();it!=nodes.end();it++){
				TNode *n = *it;
				if (n->has_state()){
					n->propagate(false);
				}
			}


		}
	};
}

#endif /* LG_H_ */
