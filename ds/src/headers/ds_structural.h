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
#include "ds_common.h"
#include <boost/checked_delete.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>

namespace ds_lg {
	class LGNode;
	class LeveledGraph;
}

namespace ds_library {
	struct instance_visitor;
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
		function_map_t mappings;	//!< port mappings (Gate->LGNode)
		ds_lg::LGNode* lgn;			//!< corresponding simulation LGNode
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
		Gate():name(),type(),lgn(0), parent(0){}
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
		 * associates this gate to a simulation primitive
		 */
		void set_lgn(ds_lg::LGNode* n){lgn = n;}
		/*!
		 * returns the associated simulation primitive
		 */
		ds_lg::LGNode* get_lgn()const{return lgn;}
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
			else
				outputs.push_back(pb);
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
		virtual ~Gate() {
			for (PortBit* in: inputs){
				delete(in);
			}
			for (PortBit* out: outputs){
				delete(out);
			}
		}
	};
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
		ds_lg::LeveledGraph* lg;						//!< corresponding leveled graph instance
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & boost::serialization::base_object<Gate>(*this);
			ar & signals;
			ar & signal_counter;
			ar & own_signals;
			ar & gates;
		    ar & assignment_map;
		}
		/*!
		 * trace forward the receivers of a port and connect simulation primitives.
		 * @param pb port to trace
		 * @param node simulation node of the driving gate. The node outputs of this node are updated
		 * @param driver primitive holding the simulation value
		 * @param trace contains the gates traced so far (avoid infinite loops)
		 * @param todo gates to trace in next iteration
		 */
		void trace_lg_forward(const PortBit* pb, ds_lg::LGNode *node, ds_common::lg_v64 *driver, std::map<std::string, Gate*> *trace, std::stack<Gate*> *todo);
		/*!
		 * traces an input port backwards to find unused gates and deletes unconnected signals
		 * @param pb input port to trace
		 * @param unused holds identified unused gates
		 */
		void find_unused_gates(const PortBit *pb, std::vector<const Gate*> *unused);
	public:
		/*!
		 * leveled graph pointer is null
		 */
		NetList():lg(0){}
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
			std::vector<PortBit*> ports;
			ports.insert(ports.begin(), g->get_inputs()->begin(), g->get_inputs()->end());
			ports.insert(ports.begin(), g->get_outputs()->begin(), g->get_outputs()->end());
			for (auto it=ports.begin();it!=ports.end();it++){
				PortBit* pb = *it;
				Signal *s = pb->get_signal();
				s->remove_port(pb);
			}
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
		void add_signal(Signal * const s){signals[s->get_instance_name()]= s;}
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
		 * constructs an equivalent leveled graph for simulation
		 * @return
		 */
		ds_lg::LeveledGraph* build_leveled_graph();
		/*!
		 * detaches the current leveled graph and allocates another leveles graph instance
		 * @return
		 */
		ds_lg::LeveledGraph* clone_leveled_graph();
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
		void get_gates(std::vector<Gate*>& container) const {
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
		/*!
		 * netlist destructor. It deletes its internal signals and gates
		 */
		~NetList(){

			for (signal_map_t::iterator it=signals.begin();it!=signals.end();it++){
				it->second->detach();
				delete it->second;
			}
			for (signal_map_t::iterator it=own_signals.begin();it!=own_signals.end();it++){
				delete it->second;
			}
			for (gate_map_t::iterator it=gates.begin();it!=gates.end();it++){
				delete it->second;
			}

		}
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
	protected:
		/*!
		 * connects the simulation output with the simulation input equivalent to the provided port
		 * @param pb port input to drive
		 * @param driver simulation output
		 */
		void drive(PortBit*pb, ds_common::lg_v64 *driver);
	};

	void save_netlist(const std::string& file, ds_structural::NetList *nl);

	NetList* load_netlist(const std::string& file, ds_workspace::Workspace* wp);
}

#endif
