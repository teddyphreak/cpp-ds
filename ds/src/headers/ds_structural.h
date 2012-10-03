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
#include <boost/bind.hpp>
#include <boost/checked_delete.hpp>

namespace ds_lg {
	class LGNode;
}

namespace ds_library {
	struct instance_visitor;
}

namespace ds_structural {

	class Signal;
	class Gate;
	class PortBit;

	typedef std::vector<PortBit*> port_container;
	typedef std::map<std::string,std::string> function_map_t;
	typedef std::map<std::string, Gate*> gate_map_t;
	typedef std::map<std::string, Signal*> signal_map_t;

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
		PortBit(const std::string n, Gate * const g, const PortType& t):name(n),gate(g), type(t){}
		PortBit(const std::string n, const PortType& t):name(n),type(t){gate=0;}

		void setGate(Gate * const g){gate = g;}
		void disconnect(){signal = 0;}
		Gate* get_gate() const{return gate;}
		std::string get_instance_name() const {return name;}
		void set_signal(Signal* const s){signal = s;}
		Signal* get_signal() const {return signal;}
		PortType get_type() const {return type;}

	};

	class Signal{
		friend class ds_library::instance_visitor;
	private:
		std::string name;
		std::list<PortBit*> ports;
	public:
		std::string get_instance_name() const {return name;}
		void set_name(const std::string& n){name = n;}
		Signal(const std::string& name):name(name) {}
		void add_port(PortBit * const pb) {ports.push_back(pb); pb->set_signal(this);}
		void remove_port(PortBit * const pb) {ports.remove(pb); pb->disconnect();}
		const std::list<PortBit*>::const_iterator port_begin() const {return ports.begin();}
		const std::list<PortBit*>::const_iterator port_end() const {return ports.end();}

	};

	template <class T> bool inst_name (const T& a1, const T& a2){
		if (a1.get_instance_name() == a2.get_Instance_name())
			return true;
		return false;
	}

	template <class T> class name_eq_p : public std::unary_function<T,bool> {
		std::string arg2;
	public:
		explicit name_eq_p(const std::string s): arg2(s){};
		bool operator()(const T& x)  {
			return x->get_instance_name() == arg2;
		}
	};

	template <class T> class inst_eq_p : public std::unary_function<T,bool> {
		T arg2;
	public:
		explicit inst_eq_p(const T& a): arg2(a){};
		bool operator()(const T& x)  {
			return x->get_instance_name() == arg2->get_instance_name();
		}
	};

	template <class T> class name_eq : public std::unary_function<T,bool> {
		std::string arg2;
	public:
		explicit name_eq(const std::string s): arg2(s){};
		bool operator()(T& x)  {
			return x.get_instance_name() == arg2;
		}
	};

	template <class T> class inst_eq : public std::unary_function<T,bool> {
		T arg2;
	public:
		explicit inst_eq(const T& a): arg2(a){};
		bool operator()(T& x)  {
			return x.get_instance_name() == arg2.get_instance_name();
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
		void copy(Gate *g);
	public:
		Gate():name(),type(),lgn(0){}
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
			port_container::iterator port = std::find_if(ports->begin(), ports->end(), inst_eq_p<PortBit*>(pb));
			if (port != ports->end())
				ports->erase(port);
		}

		PortBit* find_port_by_name(const std::string& n){
			port_container::iterator port = std::find_if(inputs.begin(), inputs.end(), name_eq_p<PortBit*>(n));
			if (port != inputs.end())
				return *port;
			port = std::find_if(outputs.begin(), outputs.end(), name_eq_p<PortBit*>(n));
			if (port != outputs.end())
				return *port;
			return 0;

		}
		virtual ~Gate() {
			std::for_each(inputs.begin(), inputs.end(), boost::checked_deleter<PortBit>());
			std::for_each(outputs.begin(), outputs.end(), boost::checked_deleter<PortBit>());
		}
	};

	class NetList : public Gate{
		friend class ds_library::instance_visitor;
	private:
		gate_map_t gates;
		signal_map_t signals;
		int signal_counter;
		signal_map_t own_signals;
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

		~NetList(){
			for (gate_map_t::iterator it=gates.begin();it!=gates.end();it++){
				delete it->second;
			}
			for (signal_map_t::iterator it=signals.begin();it!=signals.end();it++){
				delete it->second;
			}
			for (signal_map_t::iterator it=own_signals.begin();it!=own_signals.end();it++){
				delete it->second;
			}
		}

		bool check_netlist();
	};


}

#endif
