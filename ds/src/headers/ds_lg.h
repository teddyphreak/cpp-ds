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
#include <boost/log/trivial.hpp>
#include <vector>


namespace ds_faults {
	class SimulationHook;
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

	inline lg_v64 f_mux2(val64_cpc a, val64_cpc b, val64_cpc c){ return (*a & *c) | (*b & ~(*c));}

	/*!
	 * interface to observe any node simulation event.
	 */
	class Monitor {
	public:
		/*!
		 * derived classes extend this method to log any interesting simulation event
		 * @param v current simulation value
		 */
		virtual void observe(const lg_v64& v) = 0;
	};

	/*!
	 * concrete class to capture the circuit's outputs in a pattern block
	 */
	class OutputObserver : public Monitor {
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

	typedef std::vector<Monitor*> monitor_container;

	/*!
	 * simulation primitive node. It holds references to its input and output nodes.
	 * It holds the level assigned to the node and a reference to its equivalent gate in the netlist.
	 * A node may be an 'endpoint' if simulation events are not propagated to its output nodes
	 */
	class LGNode {
	public:
		int level;						//!< level in the graph
		ds_structural::Gate *gate;				//!< equivalent gate in netlist
		std::vector<LGNode*> outputs;			//!< output nodes
		std::vector<LGNode*> inputs;			//!< input nodes
		std::list<ds_faults::SimulationHook*> hooks;
		/*!
		 * initializes public members. By default the node is not an endpoint.
		 * @param t node type
		 */
		LGNode(const std::string &t):level(-1),gate(0),endpoint(false),type(t),iteration(-1){}
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
		/*!
		 * apply all registered observers
		 */
		void observe(){
			for (monitor_container::iterator it = monitors.begin();it!=monitors.end();it++){
				Monitor *m = *it;
				m->observe(o);		// log any simulation events
			}
		}
		/*!
		 * gets the primitive value address of an input port. If the provided name cannot be found null is returned
		 * @param name name of the port value
		 * @return address of the primitive value
		 */
		virtual lg_v64** get_input(const std::string& name)=0;
		/*!
		 * returns an output primitive value
		 * @param name output port name
		 * @return pointer to this node's output primitive value
		 */
		virtual lg_v64* get_output(const std::string& name){if (name=="o")return &o; return 0;};
		/*!
		 * Simulation primitives are created according to the prototype design pattern.
		 * Derived classes implement this method to replicate the node's structure and behavior
		 * @return a newly allocated node instance
		 */
		virtual LGNode* clone()=0;
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
		void add_monitor(Monitor* m){monitors.push_back(m);}
		/*!
		 * removes the provided monitor from the monitor container
		 */
		void remove_monitor(Monitor* m){
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
		lg_v64 peek() const {return o;}
		/*!
		 * revert output to previous value
		 */
		virtual void rollback(){o = bo;}
		/*!
		 * save the output value of the base simulation
		 */
		void mark() {bo = o;}
		/*!
		 * queires the output value of the base simulation
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
		 * sets an internal pointer to the encompasing leveled graph
		 * @param graph
		 */
		void set_leveled_graph(LeveledGraph *graph) {
			lg = graph;
		}
		/*!
		 * adds a hook to be executed during the propagate method.
		 * This hook may or may not be registered in the encompassing leveled graph
		 * @param h
		 */
		void add_hook(ds_faults::SimulationHook* h){
			hooks.push_back(h);
		}
		void remove_hook(ds_faults::SimulationHook* h){
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
	protected:
		lg_v64 o; 						//!< node output
		lg_v64 bo;						//!< backup node output
		bool endpoint;					//!< true if this node is an endpoint
		std::string type;				//!< node type
		monitor_container monitors;		//!< monitor container
		LeveledGraph* lg;
		double iteration;
		/*!
		 * Processes all hooks at the input ports. Commodity function
		 */
		void hook_inputs();
		/*!
		 * processes all hooks at the output ports. Commodity function
		 */
		void hook_outputs();
	};

	/*!
	 * Simulation primitive for circuit outputs
	 */
	class Output : public LGNode {
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
		Output():LGNode("output"){ endpoint = true;};
		/*!
		 * allocate new Output instance according to the prototype pattern
		 * @return
		 */
		virtual LGNode* clone() { return new Output();}
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
	class Input : public LGNode {
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
		Input():LGNode("input"){};
		/*!
		 * allocate new Input instance according to the prototype pattern
		 * @return
		 */
		virtual LGNode* clone() { return new Input();}
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
	class LGNode1I : public LGNode {
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
		LGNode1I(const std::string& t, lg_v64 (*AA)(val64_cpc)):LGNode(t), A(AA){};
		/*!
		 * allocate new LGNode1I instance according to the prototype pattern
		 * @return
		 */
		virtual LGNode* clone() { return new LGNode1I(type, A);}
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
	class LGNode2I : public LGNode {
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
		LGNode2I(const std::string& t, lg_v64 (*AA)(val64_cpc, val64_cpc)):LGNode(t), A(AA){};
		/*!
		 * allocate new LGNode2I instance according to the prototype pattern
		 * @return
		 */
		virtual LGNode* clone() { return new LGNode2I(type, A); }
		/*!
		 * two input ports available
		 * @param name port name
		 * @return pointer to one of this node's input values. Null is name is unknown.
		 */
		virtual lg_v64** get_input(const std::string& name);
	private:
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & boost::serialization::base_object<LGNode>(*this);
			ar & a;
			ar & b;
			ar & A;
		}
	};
	/*!
	 * Simulation primitive for 3-input gates
	 */
	class LGNode3I : public LGNode {
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
		LGNode3I(const std::string& t, lg_v64 (*AA)(val64_cpc, val64_cpc, val64_cpc c)):LGNode(t), A(AA){};
		/*!
		 * allocate new LGNode3I instance according to the prototype pattern
		 * @return
		 */
		virtual LGNode* clone() { return new LGNode3I(type, A); }
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
	class LGNode4I : public LGNode {
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
		LGNode4I(const std::string& t, lg_v64 (*AA)(val64_cpc, val64_cpc, val64_cpc, val64_cpc)):LGNode(t), A(AA){};
		/*!
		 * allocate new LGNode4I instance according to the prototype pattern
		 * @return
		 */
		virtual LGNode* clone() { return new LGNode4I(type, A); }
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
	class LGNode5I : public LGNode {
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
		LGNode5I(const std::string& t, lg_v64 (*AA)(val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc)):LGNode(t), A(AA){};
		/*!
		 * allocate new LGNode5I instance according to the prototype pattern
		 * @return
		 */
		virtual LGNode* clone() { return new LGNode5I(type, A); }
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
			ar & boost::serialization::base_object<LGNode>(*this);
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
	class LGNode6I : public LGNode {
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
		LGNode6I(const std::string& t, lg_v64 (*AA)(val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc)):LGNode(t), A(AA){};
		/*!
		 * allocate new LGNode6I instance according to the prototype pattern
		 * @return
		 */
		virtual LGNode* clone() { return new LGNode6I(type, A); }
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
	class LGNode7I : public LGNode {
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
		LGNode7I(const std::string& t, lg_v64 (*AA)(val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc)):LGNode(t), A(AA){};
		/*!
		 * allocate new LGNode7I instance according to the prototype pattern
		 * @return
		 */
		virtual LGNode* clone() { return new LGNode7I(type, A); }
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
		LGNode8I(const std::string& t, lg_v64 (*AA)(val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc, val64_cpc)):LGNode(t), A(AA){};
		/*!
		 * allocate new LGNode8I instance according to the prototype pattern
		 * @return
		 */
		virtual LGNode* clone() { return new LGNode8I(type, A); }
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
	class LGNodeArr : public LGNode {
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
		LGNodeArr(const std::string& t, const int& inputs, lg_v64 (*AA)(val64_cpc, val64_cpc), bool iv):LGNode(t), input_size(inputs), A(AA), invert(iv){
			input_array = new lg_v64*[input_size];
		};
		/*!
		 * allocate new LGNodeArr instance according to the prototype pattern
		 * @return
		 */
		virtual LGNode* clone() { return new LGNodeArr(type, input_size, A, invert); }
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
	class LGState : public LGNode {
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
		LGState(const std::string& t):LGNode(t), one(-1L,0), zero(0,0){
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
		virtual LGNode* clone() { return new LGState(type); }
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
		virtual ~LGState(){};
	};
	typedef std::vector<LGNode*> lg_node_container;
	typedef lg_node_container::iterator lg_node_iterator;
	typedef std::vector<Input*> lg_input_container;
	typedef std::vector<Output*> lg_output_container;
	typedef std::list<ds_faults::SimulationHook*> hook_container;

	class LeveledGraph {

		friend class ds_structural::NetList;

	public:

		lg_input_container inputs;		//!< input ports
		lg_output_container outputs;	//!< output ports
		lg_node_container nodes;		//!< all nodes
		lg_node_container registers;	//!< all sequential elements
		hook_container hooks;			//!< all active hooks

		void add_node(LGNode* n){
			nodes.push_back(n);
			registry[n->get_name()] = n;
		}

		/*!
		 * verify the internal structure of the graph
		 * @return true if check is successful
		 */
		bool sanity_check();
		/*!
		 * adds an input port
		 * @param in input port
		 */
		void add_input(Input* in){
			inputs.push_back(in);
			registry[in->get_name()] = in;
		}
		/*!
		 * adds an output port
		 * @param out output port
		 */
		void add_output(Output* out){
			outputs.push_back(out);
			registry[out->get_name()] = out;
		}
		/*
		 * various iterators for input and output traversal
		 */
		lg_input_container::iterator get_inputs_begin(){return inputs.begin();}
		lg_input_container::iterator get_inputs_end(){return inputs.end();}
		lg_output_container::iterator get_outputs_begin(){return outputs.begin();}
		lg_output_container::iterator get_outputs_end(){return outputs.end();}
		/*!
		 * Sets the offset of input and output nodes to read values from and write values to a pattern block during logic simulation
		 * @param adapter describes port information in the pattern block
		 */
		void adapt(const ds_pattern::CombinationalPatternAdapter* adapter);
		/*!
		 * simulation command. It performs logic simulation on a 64-bit wide pattern block
		 * @param pb
		 */
		void sim(ds_pattern::SimPatternBlock * pb);

		/*!
		 * find a node by name
		 * @param name name of desired node
		 * @return pointer to desired node
		 */
		LGNode* get_node(const std::string name){
			auto it = registry.find(name);
			if(it!=registry.end())
				return it->second;
			return 0;
		}
		/*!
		 * adds a new hook to be executed during logic simulation
		 * @param hook
		 */
		void add_hook(ds_faults::SimulationHook* hook);
		/*!
		 * removes all active hooks
		 */
		void clear_hooks();

		void clear_hook(ds_faults::SimulationHook* hook);
		/*!
		 * evaluate nodes in simulation and manage simulation events
		 */
		void sim_intermediate();

		/*!
		 * push node into event simulation node list
		 * @param node node to include
		 */
		void push_node(LGNode *node){
			simulation[node->level]->push_back(node);
		}
		/*!
		 * returns the check point of any given node:
		 * It returns the argument node pointer is an input, output or fan out node, otherwise
		 * the argument's fan out node is returned
		 * @param n node whose check point is requested
		 * @return
		 */
		LGNode* get_check_point(LGNode* n);
		/*!
		 * returns a pointer to the netlist associated with this graph
		 * @return
		 */
		ds_structural::NetList* get_netlist() const {return nl;}
		/*!
		 *
		 * @param h hook to propagate
		 * @return for each pattern / fault,
		 * returns 1 in position i if the hook can be propagated to its check point in the ith slot
		 */
		ds_common::int64 propagate_to_check_point(ds_faults::SimulationHook *h);

	protected:
		int num_levels;										//!< number of levels in the graph
		ds_pattern::SimPatternBlock *pattern_block; 		//!< current pattern block for simulation
		std::vector<lg_node_iterator> levels;				//!< level iterators. Two iterators define the nodes in a level
		std::vector<lg_node_container*> simulation;			//!< nodes to evaluate during intermediate simulation
		std::vector<unsigned int> level_width;				//!< number of nodes per level
		lg_v64 constant_0 = lg_v64(0L,0L);
		lg_v64 constant_1 = lg_v64(-1L,0L);
		lg_v64 constant_X = lg_v64(0L,-1L);
		ds_structural::NetList *nl;
		std::map<std::string, LGNode*> registry;
		double iteration;
	};
}

#endif /* LG_H_ */
