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
#include <list>
#include <algorithm>
#include <vector>
#include <map>
#include <stack>
#include <boost/bind.hpp>
#include <boost/checked_delete.hpp>
#include "ds_common.h"
#include "ds_simulation.h"

namespace ds_lg {
	class LGNode;
	class LeveledGraph;
	struct lg_v64;

}

namespace ds_library {
	struct instance_visitor;
}

namespace ds_structural {

	class Signal;
	class Gate;
	class PortBit;

	typedef std::vector<PortBit*> port_container;
	typedef port_container::iterator port_iterator;
	typedef std::map<std::string,std::string> function_map_t;
	typedef std::map<std::string, Gate*> gate_map_t;
	typedef std::map<std::string, Signal*> signal_map_t;
	typedef std::list<ds_structural::PortBit*> sp_container;
	typedef std::map<std::string, std::string> assignment_map_t;

	enum PortType {
		DIR_IN,
		DIR_OUT,
		DIR_INOUT
	};

	class PortBit {
	private:
		std::string name;
		Signal* signal;
		Gate* gate;
		PortType type;
	public:
		PortBit(const std::string n, Signal * const s, Gate * const g, const PortType& t):name(n),signal(s),gate(g),type(t){}
		PortBit(const std::string n, Gate * const g, const PortType& t):name(n), signal(0),gate(g), type(t){}
		PortBit(const std::string n, const PortType& t):name(n),signal(0),type(t){gate=0;}

		void setGate(Gate * const g){gate = g;}
		void disconnect(){signal = 0;}
		Gate* get_gate() const{return gate;}
		std::string get_instance_name() const {return name;}
		void set_signal(Signal* const s){signal = s;}
		Signal* get_signal() const {return signal;}
		PortType get_type() const {return type;}
		std::string get_qualified_name() const;
		~PortBit();

	};

	class Signal{
		friend class ds_library::instance_visitor;
	private:
		std::string name;
		sp_container ports;
		ds_simulation::Value val;
	public:
		std::string get_instance_name() const {return name;}
		void set_name(const std::string& n){name = n;}
		Signal(const std::string& name, ds_simulation::Value val):name(name),val(val) {}
		Signal(const std::string& name):Signal(name, ds_simulation::BIT_UD) {}
		void add_port(PortBit * const pb) {ports.push_back(pb); pb->set_signal(this);}
		void remove_port(PortBit * const pb) {ports.remove(pb); pb->disconnect();}
		sp_container::const_iterator port_begin() const {return ports.begin();}
		sp_container::const_iterator port_end() const {return ports.end();}
		int count_ports(){return ports.size();}
		bool is_fixed() const {return val != ds_simulation::BIT_UD;}
		ds_simulation::Value get_fixed_value() const {return val;}
		void set_value(const ds_simulation::Value& v ) {val = v;}
		void detach(){
			for (ds_structural::PortBit *pb:ports){
				pb->disconnect();
			}
		}
	};

	class Gate {
	protected:
		std::string name;
		std::string type;
		port_container inputs;
		port_container outputs;
		function_map_t mappings;
		ds_lg::LGNode* lgn;
		Gate* parent;
		void copy(Gate *g);
	public:
		Gate():name(),type(),lgn(0), parent(0){}
		void set_parent(Gate* p){ parent = p;}
		Gate* get_parent () const {return parent;}
		void set_type(const std::string& n){type = n;}
		std::string get_type() const {return type;}
		void set_instance_name(const std::string& n){name = n;}
		std::string get_instance_name() const {return name;}
		const port_container* get_inputs() const {return &inputs;}
		const port_container* get_outputs() const {return &outputs;}
		void add_mapping(const std::string& formal, const std::string& n) {mappings[formal] = n;}
		std::string get_mapping(const std::string& name) const{
			function_map_t::const_iterator it = mappings.find(name);
			if (it!=mappings.end()){
				return it->second;
			}
			return std::string("");
		}
		void set_lgn(ds_lg::LGNode* n){lgn = n;}
		ds_lg::LGNode* get_lgn()const{return lgn;}
		std::size_t get_num_ports() const {return inputs.size() + outputs.size();}
		virtual Gate* clone();
		void add_port(PortBit * const pb){
			if (pb->get_type() == DIR_IN)
				inputs.push_back(pb);
			else
				outputs.push_back(pb);
		}
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
		virtual ~Gate() {
			for (PortBit* in: inputs){
				delete(in);
			}
			for (PortBit* out: outputs){
				delete(out);
			}
		}
	};

	class NetList : public Gate{
		friend class ds_library::instance_visitor;
	private:
		gate_map_t gates;
		signal_map_t signals;
		int signal_counter;
		signal_map_t own_signals;
		assignment_map_t assignment_map;
		void trace_lg_forward(const PortBit* pb, ds_lg::LGNode *node, ds_lg::lg_v64 *driver, std::map<std::string, Gate*> *trace, std::stack<Gate*> *todo);
		void find_unused_gates(const PortBit *pb, std::vector<const Gate*> *unused);
	public:
		void set_name(const std::string& n){name = n;}
		std::string get_instance_name() const {return name;}
		void add_gate(Gate * const g){gates[g->get_instance_name()]=g;}
		void remove_gate(const Gate* g){
			gates.erase(g->get_instance_name());
		}
		NetList* clone();
		Gate* find_gate(const std::string& name){
			gate_map_t::iterator it;
			it = gates.find(name);
			if (it!=gates.end())
				return it->second;
			return 0;
		}
		void add_signal(Signal * const s){signals[s->get_instance_name()]= s;}
		void remove_signal(Signal* s){
			signals.erase(s->get_instance_name());
			s->detach();
		}
		Signal* find_signal(const std::string& name){
			signal_map_t::iterator it = signals.find(name);
			if (it != signals.end())
				return it->second;
			return 0;
		}
		Signal* create_signal(){
			static int counter = 0;
			std::string name = "ds_" + counter++;
			Signal* s = new Signal(name);
			own_signals[name] = s;
			return s;
		}
		ds_lg::LeveledGraph* build_leveled_graph();

		void add_assignment(std::string lhs, std::string rhs) {
			assignment_map[lhs] = rhs;
		}

		void get_output_cone(PortBit *pb, std::set<PortBit*> *cone);

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

		bool check_netlist();
		void remove_floating_signals();
		bool remove_unused_gates();
	protected:
		void drive(PortBit*pb, ds_lg::lg_v64 *driver);

	};


}

#endif
