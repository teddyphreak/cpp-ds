/*
 * ds_common.h
 *
 *  Created on: Jun 29, 2012
 *      Author: cookao
 */

#ifndef DS_STRUCTURAL_H_
#define DS_STRUCTURAL_H_

#include <iostream>
#include <string>
#include <set>
#include <algorithm>
#include <stack>
#include <fstream>
#include <sstream>
#include "ds_common.h"
#include <boost/checked_delete.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/spirit/include/karma.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>
#include <boost/spirit/include/phoenix_fusion.hpp>

namespace ds_lg {
	class LogicNode;

	template<class N, class R, class I, class O, class V>
	class GenericLeveledGraph;

	class LeveledGraph;
	class TLeveledGraph;
	class Input;
	class Output;
}

namespace ds_timing {
	class TLeveledGraph;
}

namespace ds_library {
	struct instance_visitor;
	class Library;
}

namespace ds_workspace {
	class Workspace;
}

namespace ds_structural {

	class Signal;
	class Gate;
	class PortBit;
	class NetList;

	typedef std::vector<PortBit*> port_container; 	//!< holds pointers to ports used in gates
	typedef port_container::iterator port_iterator;	//!< iterates over pointers to ports
	typedef std::map<std::string,std::string> function_map_t;	//!< holds a mapping between formal port names in a library and primitive port names
	typedef std::map<std::string, Gate*> gate_map_t; //!< gate container indexed by gate name
	typedef std::map<std::string, Signal*> signal_map_t; //!< signal container indexed by signal name
	typedef std::list<ds_structural::PortBit*> sp_container; //!< holds pointers to ports used in signals
	typedef std::map<std::string, std::string> assignment_map_t; //!< TBD
	typedef std::vector<Signal*> signal_container;

	/*!
	 * defines the port type of any port
	 */
	enum PortType {
		DIR_IN,  //!< input port
		DIR_OUT, //!< output port
		DIR_INOUT//!< bidirectional port
	};
	/*!
	 * this class represents a port in gate. A port has a name and a type \sa PortType.
	 * A port belongs to a single gate and may be connected to other ports by means of a (1) signal
	 */
	class PortBit {
	private:
		std::string name; 	//!< port name
		Signal* signal; 	//!< signal connected to this port
		Gate* gate;     	//!< gate to which this port belongs
		PortType type;		//!< port type
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			//Note that member gate is NOT serialized. This must be fixed when loading the instance
			ar & name;
		    ar & signal;
		    ar & type;
		}
	public:
		PortBit():signal(0),gate(0){}
		/*!
		 * port constructor. All members specified
		 * @param n port name
		 * @param s signal to be connected to this port
		 * @param g owner gate
		 * @param t gate type
		 */
		PortBit(const std::string n, Signal * const s, Gate * const g, const PortType& t):name(n),signal(s),gate(g),type(t){}
		/*!
		 * port constructor. No signal specified (set to null)
		 * @param n port name
		 * @param g owner gate
		 * @param t port type
		 */
		PortBit(const std::string n, Gate * const g, const PortType& t):name(n), signal(0),gate(g), type(t){}

		/*!
		 * port constructor. No owner gate specified (null) and no signal specified (null)
		 * @param n port name
		 * @param t port type
		 */
		PortBit(const std::string n, const PortType& t):name(n),signal(0),type(t){gate=0;}

		/*!
		 * sets the owner gate of this port
		 * @param g
		 */
		void set_gate(Gate * const g){gate = g;}

		/*!
		 * sets the signal of this port to null
		 */
		void disconnect(){signal = 0;}
		/*!
		 * returns a pointer the owner gate of this port
		 */
		Gate* get_gate() const{return gate;}
		/*!
		 * returns the name of the port without (gate) qualifiers
		 */
		std::string get_instance_name() const {return name;}
		/*!
		 * connects this port to other ports
		 * @param s pointer to the signal to which this port is connected
		 */
		void set_signal(Signal* const s){signal = s;}
		/*!
		 * returns the signal connected to this port
		 */
		Signal* get_signal() const {return signal;}
		/*!
		 * returns the txpe of this port @sa PortType
		 */
		PortType get_type() const {return type;}
		/*!
		 * returns the qualified name of the port: the gate hierarchy up to the owner gate with the port name appended at the end
		 */
		std::string get_qualified_name() const;
		~PortBit();
	};
	/*!
	 * represents connections between ports. It contains a signal name and a container of attached ports.
	 * A signal may also hold a predefined value for simulation
	 */
	class Signal{
		friend class ds_library::instance_visitor;
	private:
		std::string name; 			//!< signal name
		sp_container ports; 		//!< port container
		ds_common::Value val; 	//!< simulation value
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & name;
		    ar & ports;
		    ar & val;
		}
		Signal(){}
	public:
		/*!
		 * returns the signal name
		 */
		std::string get_instance_name() const {return name;}
		/*!
		 * sets the signal name
		 */
		void set_name(const std::string& n){name = n;}
		/*!
		 * signal constructor. Simulation value specified
		 * @param name signal name
		 * @param val predefined simulation value
		 */
		Signal(const std::string& name, ds_common::Value val):name(name),val(val) {}
		/*!
		 * signal constructor. Undefined simulation value
		 * @param name signal name
		 */
		Signal(const std::string& name):Signal(name, ds_common::BIT_UD) {}
		/*!
		 * connects this signal to a port. It adds a port to the internal port container and sets the port's signal accordingly
		 * @param pb pointer to the port to be connected
		 */
		void add_port(PortBit * const pb) {ports.push_back(pb); pb->set_signal(this);}
		/*!
		 * disconnects a port from this signal. It removes the port from the internal container and disconnects the port
		 * @param pb pointer to the port to be disconnected
		*/
		void remove_port(PortBit * const pb) {ports.remove(pb); pb->disconnect();}
		/*!
		 * returns an iterator to the first element in the port container
		 */
		sp_container::const_iterator port_begin() const {return ports.begin();}
		/*!
		 * returns an iterator to the last element in the port container
		 */
		sp_container::const_iterator port_end() const {return ports.end();}
		/*!
		 * returns the number of ports connected to this signal
		 */
		int count_ports(){return ports.size();}

		int count_output_ports() const {
			int outputs = 0;
			for (ds_structural::PortBit* p: ports){
				if (p->get_type() == DIR_OUT || p->get_type() == DIR_INOUT){
					outputs++;
				}
			}
			return outputs;
		}
		/*!
		 * true if this signal has an initial value for simulation different from "undefined" @sa ds_common::Value
		 */
		bool is_fixed() const {return val != ds_common::BIT_UD;}
		/*!
		 * returns the predefined value for this signal
		 */
		ds_common::Value get_fixed_value() const {return val;}
		/*!
		 * sets the predefined value for this signal
		 */
		void set_value(const ds_common::Value& v ) {val = v;}
		/*!
		 * disconnects all connected ports
		 */
		void detach(){
			for (ds_structural::PortBit *pb:ports){
				pb->disconnect();
			}
		}
	};
	/*!
	 * represents a gate. A gate has a name and a type. It contains input and output ports.
	 * A gate may have a parent gate (netlist) and an associated LeveledGraphNode @sa ds_lg::LGNode.
	 * Each gate holds information to bind its ports to the inputs and outputs of a ds_lg::LGNode
	 */
	class Gate {
		friend NetList* load_netlist(const std::string& file, ds_workspace::Workspace* wp);
	protected:
		std::string name; 			//!< gate name
		std::string type; 			//!< gate type
		port_container inputs;		//!< input ports
		port_container outputs;		//!< output ports
		port_container inouts;		//!< input ports
		function_map_t mappings;	//!< port mappings (Gate->LGNode)
		ds_lg::LogicNode* lgn;		//!< corresponding simulation LogicNode
		Gate* parent;				//!< parent gate or netlist

		/*!
		 * makes this gate a deep copy of the provided gate
		 * @param g reference gate
		 */
		void copy(Gate *g);
	private:
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & name;
		    ar & type;
		    ar & inputs;
		    ar & outputs;
		    ar & mappings;
		    ar & parent;
		}
	public:
		/*!
		 * gate constructor: produces and "empty" gate. All structural information must be provided
		 */
		Gate():name(),type(),parent(0){}
		/*!
		 * sets the parent gate
		 * @param p pointer to parent gate
		 */
		void set_parent(Gate* p){ parent = p;}
		/*!
		 * returns pointer to parent gate
		 */
		Gate* get_parent () const {return parent;}
		/*!
		 * sets the gate type
		 * @param n any string may be a gate type
		 */
		void set_type(const std::string& n){type = n;}
		/*!
		 * returns the gate type. Any string may be the gate type
		 */
		std::string get_type() const {return type;}
		/*!
		 * sets the name of the gate
		 * @param n any string may be the gate name
		 */
		void set_instance_name(const std::string& n){name = n;}
		/*!
		 * returns the name of the gate
		 * @return any strung may be the gate name
		 */
		std::string get_instance_name() const {return name;}
		/*!
		 * returns a pointer to a constant container of port inputs. Ports cannot be modified
		 */
		const port_container* get_inputs() const {return &inputs;}
		/*!
		 * returns a pointer to a constant container of port outputs. Ports cannot be modified
		 */
		const port_container* get_outputs() const {return &outputs;}
		/*!
		 * creates a mapping between the port library name and the primitive port name
		 * @param library name in the library
		 * @param primitive name in the primitive simulation node
		 */
		void add_mapping(const std::string& library, const std::string& primitive) {mappings[library] = primitive;}
		/*!
		 * retrieves the primitive port name given the port library name
		 * @param name port library name
		 * @return primitive name
		 */
		std::string get_mapping(const std::string& name) const{
			function_map_t::const_iterator it = mappings.find(name);
			if (it!=mappings.end()){
				return it->second;
			}
			return std::string("");
		}
		/*!
		 * returns the number of ports (inputs + outputs)
		 */
		std::size_t get_num_ports() const {return inputs.size() + outputs.size();}
		/*!
		 * allocates a new gate and copies the structure of this gate
		 * @return a pointer to a deep copy of this gate
		 */
		virtual Gate* clone();
		/*!
		 * adds a port to the structure of this gate. It holds the port according to its type
		 */
		void add_port(PortBit * const pb){
			if (pb->get_type() == DIR_IN)
				inputs.push_back(pb);
			else if (pb->get_type() == DIR_OUT)
				outputs.push_back(pb);
			else
				inouts.push_back(pb);
		}
		/*!
		* removes a port from the structure of this gate.
		*/
		void remove_port(PortBit * const pb){
			port_container* ports = &outputs;
			if (pb->get_type()==DIR_IN)
				ports = &inputs;
				port_container::iterator port = std::find_if(ports->begin(), ports->end(),
					[&](PortBit* p) {
				return pb->get_instance_name() == p->get_instance_name();
			});
			if (port != ports->end())
				ports->erase(port);
		}

		/*!
		 * searches for a gate given the gate's name
		 * @param n name of the gate (unqualified)
		 * @return pointer to the port whose name matches the provided name
		 */
		PortBit* find_port_by_name(const std::string& n){
			port_container::iterator port = std::find_if(inputs.begin(), inputs.end(),
					[&](PortBit* p) {
				return n == p->get_instance_name();
			});
			if (port != inputs.end())
				return *port;
			port = std::find_if(outputs.begin(), outputs.end(),
					[&](PortBit* p) {
				return n == p->get_instance_name();
			});
			if (port != outputs.end())
				return *port;
			return 0;
		}
		/*!
		 * gate destructor. It deletes inputs and outputs.
		 */
		virtual ~Gate();
	};

	std::string escape_name(const std::string& name);
	std::string get_implicit_instantiation(ds_structural::Gate* g);
	std::string get_explicit_instantiation(ds_structural::Gate* g);

	/*!
	 * a netlist is a gate which may contain another gates and holds signals connecting ports.
	 * A netlist constructs an equivalent leveled graph for simulation
	 */
	class NetList : public Gate{
		friend NetList* load_netlist(const std::string& file, ds_workspace::Workspace* wp);
		friend class ds_library::instance_visitor;
	private:
		const std::string DS_SIGNAL_PREFIX = "ds_"; //!< prefix for automatically generated signals
		gate_map_t gates;							//!< gates in the netlist
		signal_map_t signals;						//!< signals in the netlist
		int signal_counter;							//!< number of signals so far
		signal_map_t own_signals;					//!< auxiliary signals
		assignment_map_t assignment_map;
		signal_container clocks;
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & boost::serialization::base_object<Gate>(*this);
			ar & signals;
			ar & signal_counter;
			ar & own_signals;
			ar & gates;
		    ar & assignment_map;
		    ar & clocks;
		}
		/*!
		 * trace forward the receivers of a port and connect simulation primitives.
		 * @param pb port to trace
		 * @param node simulation node of the driving gate. The node outputs of this node are updated
		 * @param driver primitive holding the simulation value
		 * @param trace contains the gates traced so far (avoid infinite loops)
		 * @param todo gates to trace in next iteration
		 */
		template <class V, class T>
		void trace_lg_forward(const PortBit* pb, V *node, T *driver, std::map<std::string, Gate*> *trace, std::stack<Gate*> *todo,
				 std::map<Gate*,V*>& node_map);
		/*!
		 * traces an input port backwards to find unused gates and deletes unconnected signals
		 * @param pb input port to trace
		 * @param unused holds identified unused gates
		 */
		void find_unused_gates(const PortBit *pb, std::set<const Gate*> *unused);
	public:
		/*!
		 * leveled graph pointer is null
		 */
		NetList(){}
		/*!
		 * sets the name of this netlist
		 */
		void set_name(const std::string& n){name = n;}
		/*!
		 * returns the name of the netlist
		 */
		std::string get_instance_name() const {return name;}
		/*!
		 * adds a gate to the netlist
		 */
		void add_gate(Gate * const g){gates[g->get_instance_name()]=g;}
		/*!
		 * removes a gate from the netlist
		 */
		void remove_gate(const Gate* g){
			gates.erase(g->get_instance_name());
			dettach_gate(g);
		}

		void dettach_gate(const Gate* g){
			gates.erase(g->get_instance_name());
			std::vector<PortBit*> ports;
			ports.insert(ports.begin(), g->get_inputs()->begin(), g->get_inputs()->end());
			ports.insert(ports.begin(), g->get_outputs()->begin(), g->get_outputs()->end());
			for (auto it=ports.begin();it!=ports.end();it++){
				PortBit* pb = *it;
				Signal *s = pb->get_signal();
				if (s!=0)
					s->remove_port(pb);
			}
		}

		void change_signal_name(Signal *s, const std::string& name){
			signals.erase(s->get_instance_name());
			s->set_name(name);
			add_signal(s);
		}

		/*!
		 * creates a new netlist with the same structure as this netlist. Useful to instantiate hierarchical designs
		 * @return deep copy of this netlist
		 */
		NetList* clone();
		/*!
		 * searches for a gate having the provided name
		 * @param name instance name of the desired gate
		 * @return the desired gate. Null if gate cannot be found
		 */
		Gate* find_gate(const std::string& name){
			gate_map_t::iterator it;
			it = gates.find(name);
			if (it!=gates.end())
				return it->second;
			return 0;
		}
		/*!
		 * adds signal to the netlist
		 */
		bool add_signal(Signal * const s){
			auto it = signals.find(s->get_instance_name());
			if (it == signals.end()){
				signals[s->get_instance_name()]= s;
				return true;
			}
			return false;
		}
		/*!
		 * removes a signal from the netlist
		 */
		void remove_signal(Signal* s){
			signals.erase(s->get_instance_name());
			s->detach();
		}
		/*!
		* searches for a signal having the provided name
		* @param name instance name of the desired signal
		* @return the desired signal. Null if signal cannot be found
		*/
		Signal* find_signal(const std::string& name){
			signal_map_t::iterator it = signals.find(name);
			if (it != signals.end())
				return it->second;
			return 0;
		}
		/*!
		 * creates a new signal with an automatically generated name.
		 * The name is @sa DS_SIGNAL_PREFIX concatenated with the
		 * @return
		 */
		Signal* create_signal(){
			static int counter = 0;
			std::string name = DS_SIGNAL_PREFIX + std::to_string(counter++);
			Signal* s = new Signal(name);
			own_signals[name] = s;
			return s;
		}
		/*!
		 * constructs an equivalent leveled graph for zero-delay simulation
		 * @return
		 */
		ds_lg::LeveledGraph* get_sim_graph(ds_library::Library *lib);
		/*!
		 * constructs a leveled graph. It makes use of the ds_lg::LeveledGraphBuilder to build the graph incrementally.
		 * @param builder resulting leveled graph
		 * @param node_map Node map for easy lookup between gates and simulation nodes
		 * @param input_prototype simulation input nodes are cloned from this instance
		 * @param output_prototype simulation outputs nodes are cloned from this interface
		 */
		template<class N, class R, class I, class O, class V>
		void build_leveled_graph(ds_lg::GenericLeveledGraph<N,R,I,O,V>* builder, std::map<Gate*, N*>& node_map,
				std::map<Gate*, R*>& register_map, const I& input_prototype, const O& output_prototype);
		/*!
		 * adds an assignment to the circuit representation ('<=' in vhdl or 'assign' in verilog)
		 * @param lhs left hand side of assignment
		 * @param rhs right hand side of assignment
		 */
		void add_assignment(std::string lhs, std::string rhs) {
			assignment_map[lhs] = rhs;
		}
		/*!
		 * finds all netlist output port bits reachable from the provided port bit
		 * @param pb start of forward trace
		 * @param cone port bits in the output cone
		 */
		void get_output_cone(PortBit *pb, std::set<PortBit*> *cone);
		/*!
		 * stores all gate pointers in this netlist into the specified container
		 * @param container array where gates are stored
		 */
		template<typename Container>
		void get_gates(Container& container) const {
			for (auto it= gates.begin();it!=gates.end();it++){
				Gate* g = it->second;
				container.push_back(g);
			}
		}
		/*!
		 * stores all gate pointers in this netlist into the specified container
		 * @param container array where gates are stored
		 */
		void get_signals(std::vector<Signal*>& container) const {
			for (auto it= signals.begin();it!=signals.end();it++){
				Signal* s = it->second;
				container.push_back(s);
			}
		}
		bool define_clock(const std::string& clk_name){
			Signal* s = find_signal(clk_name);
			if (s!=0){
				clocks.push_back(s);
				return true;
			}
			return false;
		}
		void remove_clock(const std::string& clk_name){
			Signal* s = find_signal(name);
			if (s!=0){
				auto it = std::find(clocks.begin(), clocks.end(), s);
				if (it != clocks.end()){
					clocks.erase(it);
				}
			}
		}

		ds_timing::TLeveledGraph* get_ts_graph(ds_library::Library *lib);

		ds_lg::TLeveledGraph* get_loc_graph(ds_library::Library *lib);
		/*!
		 * netlist destructor. It deletes its internal signals and gates
		 */
		~NetList();
		/*!
		 * performs various sanity checks to verify the integrity of the netlist
		 * @return true if check is successful
		 */
		bool check_netlist();
		/*!
		 * removes signals without any port connections
		 */
		void remove_floating_signals();
		/*!
		 *
		 * @return true if any gate was removed
		 */
		bool remove_unused_gates();
		/*!
		 *
		 */
		template <typename OutputIterator>
		void dump_verilog_implicit(OutputIterator& sink){
			dump_verilog(sink, false);
		}
		template <typename OutputIterator>
		void dump_verilog_explicit(OutputIterator& sink){
			dump_verilog(sink, true);
		}
		template <typename OutputIterator>
		void dump_verilog(OutputIterator& sink, const bool& verilog_ex){
			ds_structural::port_container all_ports;
			all_ports.insert(all_ports.end(), inputs.begin(), inputs.end());
			all_ports.insert(all_ports.end(), outputs.begin(), outputs.end());
			sink << "module " << escape_name(get_instance_name()) << " (\n";
			auto it=all_ports.begin();
			if (it!=all_ports.end()){
				PortBit* first = *it++;
				sink << escape_name(first->get_instance_name());
				for (;it!=all_ports.end();it++){
					PortBit* pb = *it;
					sink << ",\n" << escape_name(pb->get_instance_name());
				}
			}
			sink << ");\n";
			for (auto it=inputs.begin();it!=inputs.end();it++){
				PortBit *pb = *it;
				sink << "input\t" << escape_name(pb->get_instance_name()) << ";\n";
			}
			for (auto it=outputs.begin();it!=outputs.end();it++){
				PortBit *pb = *it;
				sink << "output\t" << escape_name(pb->get_instance_name()) << ";\n";
			}
			for (auto it=signals.begin();it!=signals.end();it++){
				Signal *s = it->second;
				if (find_port_by_name(s->get_instance_name())==0){
					sink << "wire\t" << s->get_instance_name() << ";\n";
				}
			}
			for (auto it=gates.begin();it!=gates.end();it++){
				Gate *g = it->second;
				if (verilog_ex){
					sink << ds_structural::get_explicit_instantiation(g) << std::endl;
				} else {
					sink << ds_structural::get_implicit_instantiation(g) << std::endl;
				}
			}
			for (auto it=signals.begin();it!=signals.end();it++){
				Signal *s = it->second;
				if (s->get_fixed_value() != ds_common::BIT_UD){
					std::string v = "1'b0";
					if (s->get_fixed_value() != ds_common::BIT_1)
						v = "1'b1";
					sink << "assign " << s->get_instance_name() << " = " << v << ";\n";
				}
			}
			sink << "endmodule";
		}
	protected:
		/*!
		 * connects the simulation output with the simulation input equivalent to the provided port
		 * @param pb port input to drive
		 * @param driver simulation output
		 */
		template <class V, class T>
		void drive(PortBit*pb, T *driver, std::map<Gate*,V*>& node_map);
	};

	void save_netlist(const std::string& file, ds_structural::NetList *nl);

	NetList* load_netlist(const std::string& file, ds_workspace::Workspace* wp);

	struct ScanPort{
		std::string signal_name;
		std::string type;
		std::string chain_number;
		std::string ff_number;
	};

	class CombinationalScanMap {
		typedef std::vector<std::vector<std::string> > ScanConfiguration;
		ScanConfiguration inputs;
		ScanConfiguration outputs;
	public:
		unsigned int get_output_chains() const {return outputs.size();}
		unsigned int get_input_chains() const {return inputs.size();}
		unsigned int get_output_chain_length(const int& i) const {return outputs[i].size();}
		unsigned int get_input_chain_length(const int& i) const {return inputs[i].size();}
		std::string get_output_signal(const int& ch, const int& ff) const {return outputs[ch][ff];}
		std::string get_input_signal(const int& ch, const int& ff) const {return inputs[ch][ff];}
		std::vector<std::string>::const_iterator get_inputs_begin(const int &chain) const {return inputs[chain].begin();}
		std::vector<std::string>::const_iterator get_inputs_end(const int &chain) const {return inputs[chain].end();}
		std::vector<std::string>::const_iterator get_outputs_begin(const int &chain) const {return outputs[chain].begin();}
		std::vector<std::string>::const_iterator get_outputs_end(const int &chain) const {return outputs[chain].end();}

		template<typename T>
		CombinationalScanMap(T start, T end){
			std::map<int,std::vector<std::pair<int, std::string> >* > input_map;
			std::map<int,std::vector<std::pair<int, std::string> >* > output_map;
			for (T it = start;it!=end;it++){
				ScanPort p = *it;

				bool is_input = p.type == "FFINPUT";

				std::map<int, std::vector<std::pair<int, std::string> >* >* map = &output_map;
				if (is_input){
					map = &input_map;
				}

				std::stringstream cn(p.chain_number);
				int chain_number = 0;
				cn >> chain_number;

				std::stringstream fn(p.ff_number);
				int ff_number = 0;
				fn >> ff_number;

				auto chain_it = map->find(chain_number);
				std::vector<std::pair<int, std::string> >*chain = 0;
				std::pair<int, std::string> pair(ff_number, p.signal_name);
				if (chain_it == map->end()){
					chain = new std::vector<std::pair<int, std::string> >();
					std::pair<int, std::vector<std::pair<int, std::string> >* > new_pair(chain_number, chain);
					map->insert(new_pair);
				} else {
					chain = chain_it->second;
				}
				chain->push_back(pair);
			}
			for (auto it=input_map.begin();it!=input_map.end();it++){
				std::vector<std::pair<int, std::string> >*chain = it->second;
				std::sort(chain->begin(),chain->end(),
					[](const std::pair<int, std::string>& a, const std::pair<int, std::string>& b) -> bool {
				    	return b.first > a.first;
					}
				);
				std::vector<std::string> scan;
				for (auto it = chain->begin();it!=chain->end();it++){
					scan.push_back(it->second);
				}
				inputs.push_back(scan);
			}
			for (auto it=output_map.begin();it!=output_map.end();it++){
				std::vector<std::pair<int, std::string> >*chain = it->second;
				std::sort(chain->begin(),chain->end(),
					[](const std::pair<int, std::string>& a, const std::pair<int, std::string>& b) -> bool {
				    	return b.first > a.first;
					}
				);
				std::vector<std::string> scan;
				for (auto it = chain->begin();it!=chain->end();it++){
					scan.push_back(it->second);
				}
				outputs.push_back(scan);
			}
			for (auto it=input_map.begin();it!=input_map.end();it++){
				delete it->second;
			}
			for (auto it=output_map.begin();it!=output_map.end();it++){
				delete it->second;
			}

		}
	};

	CombinationalScanMap* get_combinational_scan_map(const std::string& file_name);

	bool parse_nxp_chain_info(const std::string& file, std::vector<ScanPort>& ports);

}

BOOST_FUSION_ADAPT_STRUCT(
    ds_structural::ScanPort,
    (std::string, signal_name)
    (std::string, type)
    (std::string, chain_number)
    (std::string, ff_number)
)

namespace ds_structural {
	/*!
	 * scan info qi grammar
	 */
	namespace qi = boost::spirit::qi;
	namespace ascii = boost::spirit::ascii;

	template <typename Iterator>
	struct nxp_chain_info_parser : qi::grammar<Iterator,std::vector<ds_structural::ScanPort>(), ascii::space_type>
	{

		nxp_chain_info_parser() : nxp_chain_info_parser::base_type(start, "nxp_chain_info")
		{
			using qi::int_;
			using qi::char_;
			using qi::lit;
			using qi::_1;
			using boost::spirit::_val;
			using boost::phoenix::push_back;

			start = *port[push_back(_val,_1)];

			port %=
					name >>
					lit(':') >>
					name >>
					chain_nr >>
					ff_nr;

			name = qi::char_("a-zA-Z_") >> *qi::char_("a-zA-Z_0-9");
			chain_nr = ",chainnr=" >> +qi::char_("0-9");
			ff_nr = ",FFnr=" >> +qi::char_("0-9");

//			name.name("name");
//			chain_nr.name("chain_nr");
//			ff_nr.name("ff_nr");
//			port.name("port");
//			start.name("start");
//			debug(name);
//			debug(chain_nr);
//			debug(ff_nr);
//			debug(port);
//			debug(start);
		}

		qi::rule<Iterator, std::string()> name;
		qi::rule<Iterator, std::string()> chain_nr;
		qi::rule<Iterator, std::string()> ff_nr;
		qi::rule<Iterator, ds_structural::ScanPort()> port;
		qi::rule<Iterator, std::vector<ds_structural::ScanPort>(), ascii::space_type> start;

	};
}


#endif
