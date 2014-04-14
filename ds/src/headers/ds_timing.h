/*
 * timing.h
 *
 *  Created on: 04.01.2014
 *      Author: cookao
 */

#ifndef TIMING_H_
#define TIMING_H_

#include "ds_common.h"
#include "ds_lg.h"
#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>
#include <boost/spirit/include/phoenix_object.hpp>
#include <boost/spirit/include/phoenix_fusion.hpp>
#include <boost/variant.hpp>

namespace ds_timing {

	using ds_common::int64;
	using ds_common::lg_v64;
	using ds_lg::val64_cpc;

	class TNode;

	struct v64_ts {
		lg_v64 value;
		TNode* driver;
		short port_id;
		double ts;

		v64_ts():value(0L,-1L),driver(0),port_id(-1),ts(0.0){}

		v64_ts(const lg_v64& v):value(v),driver(0), port_id(-1), ts(0.0){}

		v64_ts(const lg_v64& v, TNode* node, const short id):value(v),driver(node),port_id(id), ts(0.0){}

		v64_ts(const int64 v, const int64 x):value(v,x), driver(0), port_id(-1), ts(0.0){}

		v64_ts(const int64 v, const int64 x, TNode* node, const short id):value(v,x), driver(node), port_id(id), ts(0.0){}

		v64_ts(const v64_ts& val): value(val.value), driver(val.driver), port_id(val.port_id), ts(val.ts){}

		v64_ts operator~() const{
			v64_ts o;
			o.value.v = ~value.v & ~value.x;
			o.value.x = value.x;
			return o;
		}

		v64_ts& operator=(const v64_ts &rhs) {
			if (this != &rhs) {
				value.v = rhs.value.v;
				value.x = rhs.value.x;
				ts = rhs.ts;
			}

			return *this;
		}

		v64_ts& operator&=(const v64_ts& rhs){
			value.v &= rhs.value.v;
			value.x = (value.x & ~rhs.value.x & rhs.value.v) | (rhs.value.x &  ~value.x & value.v);
			return *this;
		}
		/*!
		 * unary 'or'
		 * @param rhs 'or' operand
		 * @return logic 'or' between this and provided operand
		 */
		v64_ts& operator|=(const v64_ts& rhs){
			value.v |= rhs.value.v;
			value.x = (value.x & ~rhs.value.x & ~rhs.value.v) | (rhs.value.x & value.x & ~value.v );
			return *this;
		}
		/*!
		 * unary 'xor'
		 * @param rhs 'xor' operand
		 * @return logic 'xor' between this and provided operand
		 */
		v64_ts& operator^=(const v64_ts& rhs){
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
			ar & ts;
		}

	};

	inline v64_ts operator&(v64_ts lhs, const v64_ts& rhs){
		lhs &= rhs;
		return lhs;
	}

	inline v64_ts operator|(v64_ts lhs, const v64_ts& rhs){
		lhs |= rhs;
		return lhs;
	}

	inline v64_ts operator^(v64_ts lhs, const v64_ts& rhs){
		lhs ^= rhs;
		return lhs;
	}

	class TNode : public ds_lg::LGNode<v64_ts, TNode>{
	public:
		/*!
		 * initializes public members. By default the node is not an endpoint.
		 * @param t node type
		 */
		TNode(const std::string &t):ds_lg::LGNode<v64_ts, TNode>(t){
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
		virtual void mark() {bo = o.value;}
		/*!
		 * queries the output value of the base simulation
		 * @return
		 */
		lg_v64 get_mark() const {return bo;}
		/*!
		 * Set output value to the complement of the fault-free simulation value
		 */
		virtual void flip(){o.value = ~bo;};
		/*!
		 * Set output value to the complement of the fault-free simulation value and apply any attached observer
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

		TNode* get_sink() {return this;}

		virtual lg_v64 resolve() const {
			return peek().value;
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
		v64_ts *a;				//!< single input
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
		virtual v64_ts** get_input(const std::string& name){
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
		virtual v64_ts** get_input(const std::string& name){
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
		v64_ts *a;					//!< single input
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
		virtual v64_ts** get_input(const std::string& name);
	};
	/*!
	 * Simulation primitive for 2-input gates
	 */
	class TNode2I : public TNode {
	protected:
		v64_ts * a;								// two input nodes
		v64_ts * b;								//
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
		virtual v64_ts** get_input(const std::string& name);
	};
	/*!
	 * Simulation primitive for 3-input gates
	 */
	class TNode3I : public TNode {
	protected:
		v64_ts * a;
		v64_ts * b;
		v64_ts * c;
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
		virtual v64_ts** get_input(const std::string& name);
	};

	class TState : public TNode {
	public:
		TOutput d;
		v64_ts * cd;	//!< clk input
		v64_ts o_n;		//!< not Q output
		lg_v64 previous_n;
		v64_ts * rst;
		v64_ts * rst_n;
		v64_ts * load;
		v64_ts * load_n;
		v64_ts * si;
		v64_ts * se;
		v64_ts one;
		v64_ts zero;
		ds_pattern::SimPatternBlock** pb;
		std::size_t offset;
		boost::array<const lg_v64*,2> output_array;


	public:
		TState(const std::string& t):TNode(t), o_n(0,-1L,this,1), one(-1L,0,this,-1), zero(0,0,this,-1){
			rst = &zero;
			rst_n = &one;
			load = &zero;
			load_n = &one;
			output_array[0] = &previous;
			output_array[1] = &previous_n;
			endpoint = true;
			p = &d.o;
		}

		/*!
		 * update outputs when clk event is active
		 */
		virtual void sim() {
			d.sim();
			d.observe();
		}

		virtual void tick(){
			o = d.peek();
			o_n.value = ~o.value;
		}
		/*!
		 * Simulation primitive state gates. Faults are injected
		 */
		virtual void hook();
		/*!
		 * Reverts output and internal sink to previous values
		 */
		virtual void rollback(){
			o.value = bo;
			d.rollback();
		}
		/*!
		 * Set output values to the complement of the fault-free simulation values
		 */
		virtual void flip(){
			o.value = ~bo;
			o_n.value = bo;
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
		virtual v64_ts* get_output(const std::string& name) {
			if (name=="q")return &o;
			if (name=="qn")return &o_n;
			return 0;}
		/*!
		 * two input ports available
		 * @param name port name
		 * @return pointer to one of this node's input values (D or CD). Null is name is unknown.
		 */
		virtual v64_ts** get_input(const std::string& name);
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
		virtual void mark() {
			bo = o.value;
			d.mark();
		}
		void virtual mark_clock_cycle(){
			previous = o.value;
			previous_n = o_n.value;
		}
		virtual const lg_v64* get_previous_value(const short id) const {
			return output_array[id];
		}
		virtual void hook_outputs();
		/*!
		 * Virtual destructor
		 */
		virtual ~TState(){};

		virtual v64_ts peek_input()const {
			return d.peek();
		}

		TNode* get_sink() {return &d;}

	};

	class TErrorObserver : public ds_lg::Monitor<v64_ts> {
	protected:
		ds_lg::lg_v64 spec;
		ds_common::int64 *ds;
		ds_common::int64 *np;
	public:

		std::string tag;

		TErrorObserver(ds_common::int64* d, ds_common::int64* n):ds(d), np(n){}

		virtual void observe(const ds_lg::driver_v64& v);

		void set_spec(const ds_lg::lg_v64& s){
			spec.v = s.v;
			spec.x = s.x;
		}
		virtual ~TErrorObserver(){}
	};

	class TLeveledGraph : public ds_lg::GenericLeveledGraph<TNode,TState,TInput,TOutput,v64_ts>{
		typedef std::vector<TNode*> lg_node_container;

	public:

		TLeveledGraph():vector_offset(0){
			constant_0 = lg_v64(0L,0L);
			constant_1 = lg_v64(-1L,0L);
			constant_X = lg_v64(0L,-1L);
		}

		virtual void initialize(){
			for (auto it=nodes.begin();it!=nodes.end();it++){
				TNode *n = *it;
				if (!n->has_state()){
					combinational.push_back(n);
				}
			}
		}

	protected:
		std::vector<TNode*> combinational;

		std::size_t vector_offset;

		lg_v64 resolve(const v64_ts& ff, const v64_ts& faulty) const {
			ds_common::int64 diff = (ff.value.v ^ faulty.value.v) & ~ff.value.x;
			ds_common::int64 x = (~ff.value.x & faulty.value.x);
			lg_v64 ret(diff,x);
			return ret;
		}

	public:

		void setup();

		void adapt(const ds_pattern::SequentialPatternAdapter* adapter);

		void reset_offset(){
			vector_offset = 0;
		}
		/*!
		 * Advances the vector offset in the pattern block. This enables the simulation of the next pattern
		 */
		void next_vector(){
			vector_offset += inputs.size() + outputs.size();
		}
		/*!
		 * Symbolic scan operation. Registers are loaded according to the scan values in the pattern block
		 */
		void scan(){
			for (auto it=registers.begin();it!=registers.end();it++){
				TState *n = *it;
				n->scan();
			}
		}

		void attach_output_observers(std::map<TNode*, TErrorObserver*>& output_map,
						ds_common::int64 *detected, ds_common::int64 *possibly_detected);

		void attach_register_observers(std::map<TNode*, TErrorObserver*>& register_map,
								ds_common::int64 *detected, ds_common::int64 *possibly_detected);

		/*!
		 * Simulation of two patterns for the simulation of transition delay faults (LOC)
		 * @param pb pattern block to simulate
		 */
		virtual void sim(ds_pattern::SimPatternBlock *pb){
			pattern_block = pb;
			reset_offset();
			scan();

			//propagates events from inputs to outputs --> LAUNCH
			for (auto it=combinational.begin();it!=combinational.end();it++){
				TNode *n = *it;
				n->propagate(false);
				n->mark_clock_cycle();
			}

			for (auto it=registers.begin();it!=registers.end();it++){
				TNode *n = *it;
				n->propagate(false);
				n->mark_clock_cycle();
			}

			clocks[0].tick();

			for (ds_lg::SimulationHook<TNode>* h: hooks){
				TNode *node = h->get_hook_node(this);
				node->add_hook(h);
			}

			next_vector();
			//propagates events from inputs to outputs --> CAPTURE
			for (auto it=combinational.begin();it!=combinational.end();it++){
				TNode *n = *it;
				n->propagate(true);
				n->mark();
			}

			for (auto it=registers.begin();it!=registers.end();it++){
				TNode *n = *it;
				n->propagate(true);
				n->mark();
			}

			for (ds_lg::SimulationHook<TNode>* h: hooks){
				TNode *node = h->get_hook_node(this);
				node->remove_hooks();
			}
		}
	};


	struct sdf_header {
		std::string sdf_version;
		std::string design_name;
		std::string date;
		std::string vendor;
		std::string program_name;
		std::string program_version;
		char hierarchy_divider;
		std::string voltage;
		std::string process;
		std::string temperature;
		std::string time_scale;
	};

	struct sdf_instance {
		std::string instance;
		bool wildcard;
	};

	struct del_val {
		double min;
		double mean;
		double max;
	};

	struct sdf_port_spec {
		std::string qual;
		std::string port;
	};

	struct bool_condition{
		std::string name;
		bool val;
	};

	struct sdf_iopath {
		sdf_port_spec input;
		std::string output;
		std::vector<del_val> vals;
	};

	struct sdf_interconnect {
		std::string input;
		std::string output;
		del_val val;
	};

	struct sdf_cond {
		std::vector<bool_condition> conditions;
		sdf_iopath def;
	};

	typedef boost::variant<sdf_interconnect, sdf_iopath, sdf_cond> del_def;

	struct absolute {
		std::vector<del_def> defs;
	};

	struct sdf_delay {
		std::vector<absolute> specs;
	};

	struct sdf_cell {
		std::string cell_type;
		sdf_instance instance;
		std::vector<sdf_delay> sdf_spec;

	};

	struct sdf_data {
		sdf_header header;
		std::vector<sdf_cell> cells;
	};

}

BOOST_FUSION_ADAPT_STRUCT(
	ds_timing::sdf_header,
	(std::string, sdf_version)
	(std::string, design_name)
	(std::string, date)
	(std::string, vendor)
	(std::string, program_name)
	(std::string, program_version)
	(char, hierarchy_divider)
	(std::string, voltage)
	(std::string, process)
	(std::string, temperature)
	(std::string, time_scale)
)

BOOST_FUSION_ADAPT_STRUCT(
	ds_timing::sdf_instance,
	(std::string, instance)
	(bool, wildcard)
)

BOOST_FUSION_ADAPT_STRUCT(
	ds_timing::del_val,
	(double, min)
	(double, mean)
	(double, max)
)

BOOST_FUSION_ADAPT_STRUCT(
	ds_timing::sdf_port_spec,
	(std::string, qual)
	(std::string, port)
)

BOOST_FUSION_ADAPT_STRUCT(
	ds_timing::bool_condition,
	(std::string, name)
	(bool, val)
)

BOOST_FUSION_ADAPT_STRUCT(
	ds_timing::sdf_iopath,
	(ds_timing::sdf_port_spec, input)
	(std::string, output)
	(std::vector<ds_timing::del_val>, vals)
)

BOOST_FUSION_ADAPT_STRUCT(
	ds_timing::sdf_interconnect,
	(std::string, input)
	(std::string, output)
	(ds_timing::del_val, val)
)

BOOST_FUSION_ADAPT_STRUCT(
	ds_timing::sdf_cond,
	(std::vector<ds_timing::bool_condition>, conditions)
	(ds_timing::sdf_iopath, def)
)

BOOST_FUSION_ADAPT_STRUCT(
	ds_timing::absolute,
	(std::vector<ds_timing::del_def>, defs)
)

BOOST_FUSION_ADAPT_STRUCT(
	ds_timing::sdf_delay,
	(std::vector<ds_timing::absolute>, specs)
)

BOOST_FUSION_ADAPT_STRUCT(
	ds_timing::sdf_cell,
	(std::string, cell_type)
	(ds_timing::sdf_instance, instance)
	(std::vector<ds_timing::sdf_delay>, sdf_spec)
)

BOOST_FUSION_ADAPT_STRUCT(
	ds_timing::sdf_data,
	(ds_timing::sdf_header, header)
	(std::vector<ds_timing::sdf_cell>, cells)
)

namespace ds_timing {

	namespace qi = boost::spirit::qi;
	namespace ascii = boost::spirit::ascii;
	namespace phoenix = boost::phoenix;

	template <typename Iterator>
	struct sdf_parser : qi::grammar<Iterator,sdf_data(), ascii::space_type>
	{

		sdf_parser() : sdf_parser::base_type(start)
		{

			using qi::eps;
			using qi::_1;
			using qi::int_;
			using qi::double_;
			using qi::char_;
			using qi::blank;
			using qi::lit;
			using boost::phoenix::at_c;
			using boost::spirit::_val;
			using boost::spirit::omit;
			using phoenix::push_back;

			start =	lit("(") >> lit("DELAYFILE")
					>> header[at_c<0>(_val)=_1]
					>> +(cell[push_back(at_c<1>(_val), _1)]) >> lit(")");

			bool_c = lit("(") >> name[at_c<0>(_val)=_1] >> lit("==") >> ( lit("1'b0")[at_c<1>(_val)=false] | lit("1'b1")[at_c<1>(_val)=true] ) >>  lit(")");

			qname =  lit("\"") >> +(char_ - '\"')>> lit("\"");
			name =  +(char_ - ' ' - ')');
			header =   -( lit("(") >> lit("SDFVERSION") >> qname[at_c<0>(_val)=_1] >> lit(")") )
					>> -( lit("(") >> lit("DESIGN")     >> qname[at_c<1>(_val)=_1] >> lit(")") )
					>> -( lit("(") >> lit("DATE")       >> qname[at_c<2>(_val)=_1] >> lit(")") )
					>> -( lit("(") >> lit("VENDOR")     >> qname[at_c<3>(_val)=_1] >> lit(")") )
					>> -( lit("(") >> lit("PROGRAM")    >> qname[at_c<4>(_val)=_1] >> lit(")") )
					>> -( lit("(") >> lit("VERSION")    >> qname[at_c<5>(_val)=_1] >> lit(")") )
					>> -( lit("(") >> lit("DIVIDER")    >> char_[at_c<6>(_val)=_1] >> lit(")") )
					>> -( lit("(") >> lit("VOLTAGE")    >> +(char_ - ')')[at_c<7>(_val)=_1] >> lit(")") )
					>> -( lit("(") >> lit("PROCESS")    >> qname[at_c<8>(_val)=_1] >> lit(")") )
					>> -( lit("(") >> lit("TEMPERATURE")>> +(char_ - ')')[at_c<9>(_val)=_1] >> lit(")") )
					>> -( lit("(") >> lit("TIMESCALE")  >> +(char_ - ')')[at_c<10>(_val)=_1] >> lit(")") );

			instance = eps[at_c<1>(_val)=false]
					>> lit("(") >> lit("INSTANCE")
					>> ( lit(")") | ( ( lit('*')[at_c<1>(_val)=true] | name[at_c<0>(_val)=_1]) ) >>  lit(")") );

			val %= lit("(") >> double_ >> lit(":") >> double_ >> lit(":") >> double_ >> lit(")");

			input_spec = ( lit("(") >> name[at_c<0>(_val)=_1] >>  name[at_c<1>(_val)=_1] >> lit(")") ) | name[at_c<1>(_val)=_1];

			cond = lit("(") >> lit("COND") >> bool_c[push_back(at_c<0>(_val), _1)] % "&&" >> iopath[at_c<1>(_val)=_1] >> lit(")");

			iopath %= lit("(") >> lit("IOPATH") >> input_spec >> name >> +val >>   lit(")");

			interconnect %= lit("(") >> lit("INTERCONNECT") >> name >> name >> val >>   lit(")");

			def = (interconnect | iopath | cond);

			abs = lit("(") >> lit("ABSOLUTE") >> +def >>   lit(")");

			delay = lit("(") >> lit("DELAY") >> +abs >> lit(")");

			cell = lit("(") >> lit("CELL")
				   >> lit("(") >> lit("CELLTYPE") >> qname[at_c<0>(_val)=_1] >> lit(")")
				   >> instance[at_c<1>(_val)=_1]
				   >> +delay[push_back(at_c<2>(_val), _1)] >>  lit(")");

			name.name("name");
			start.name("start");
			header.name("header");
			qname.name("qname");
			instance.name("instance");
			val.name("del_val");
			input_spec.name("input_spec");
			iopath.name("iopath");
			interconnect.name("interconnect");
			delay.name("delay");
			abs.name("absolute");
			cell.name("cell");
			bool_c.name("boolean");
			cond.name("cond");
//			debug(name);
//			debug(start);
//			debug(header);
//			debug(qname);
//			debug(instance);
//			debug(val);
//			debug(input_spec);
//			debug(iopath);
//			debug(interconnect);
//			debug(abs);
//			debug(delay);
//			debug(cell);
//			debug(bool_c);
//			debug(cond);
		}

		qi::rule<Iterator, std::string()> qname;
		qi::rule<Iterator, std::string()> name;
		qi::rule<Iterator, sdf_data(), ascii::space_type> start;
		qi::rule<Iterator, sdf_header(), ascii::space_type> header;
		qi::rule<Iterator, sdf_instance(), ascii::space_type> instance;
		qi::rule<Iterator, del_val(), ascii::space_type> val;
		qi::rule<Iterator, bool_condition(), ascii::space_type> bool_c;
		qi::rule<Iterator, sdf_port_spec(), ascii::space_type> input_spec;
		qi::rule<Iterator, sdf_iopath(), ascii::space_type> iopath;
		qi::rule<Iterator, sdf_interconnect(), ascii::space_type> interconnect;
		qi::rule<Iterator, sdf_cond(), ascii::space_type> cond;
		qi::rule<Iterator, sdf_delay(), ascii::space_type> delay;
		qi::rule<Iterator, absolute(), ascii::space_type> abs;
		qi::rule<Iterator, del_def(), ascii::space_type> def;
		qi::rule<Iterator, sdf_cell(), ascii::space_type> cell;

	};

	bool parse_sdf(const std::string& file, ds_timing::sdf_data& sdf);

}



#endif /* TIMING_H_ */
