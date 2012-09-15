/*
 * ds_common.h
 *
 *  Created on: Jun 29, 2012
 *      Author: cookao
 */

#ifndef DS_STRUCTURAL_H_
#define DS_STRUCTURAL_H_

#include <string>
#include <list>
#include <algorithm>
#include <vector>
#include <boost/bind.hpp>
#include <boost/checked_delete.hpp>

namespace ds_lg {
	class LGNode;
}

namespace ds_structural {

	class Signal;
	class Gate;

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
		std::string get_instance_name() const{return name;}
		void set_signal(Signal* const s){signal = s;}
		Signal* get_signal() const {return signal;}
		PortType get_type() const {return type;}

	};

	class Signal{
	private:
		std::string name;
		std::list<PortBit*> ports;
	public:
		std::string get_instance_name() const {return name;}
		Signal(const std::string& name):name(name) {}
		void add_receiver(PortBit * const pb) {ports.push_back(pb); pb->set_signal(this);}
		void remove_receiver(PortBit * const pb) {ports.remove(pb); pb->disconnect();}
		const std::list<PortBit*>& get_receivers() const {return ports;}

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
		bool operator()(T& x)  {
			return x->get_instance_name() == arg2;
		}
	};

	template <class T> class inst_eq_p : public std::unary_function<T,bool> {
		T arg2;
	public:
		explicit inst_eq_p(const T& a): arg2(a){};
		bool operator()(T& x)  {
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
		std::vector<PortBit*> inputs;
		std::vector<PortBit*> outputs;
		std::map<std::string, std::string> mappings;
		ds_lg::LGNode* lgn;
	public:
		Gate():name(),type(),lgn(0){}
		void set_type(const std::string& n){name = n;}
		std::string get_instance_name() const {return name;}
		const std::vector<PortBit*>* get_inputs()const {return &inputs;}
		const std::vector<PortBit*>* get_outputs()const {return &outputs;}
		void add_mapping(const std::string& formal, const std::string& n) {mappings[formal] = n;}
		void set_lgn(ds_lg::LGNode* n){lgn = n;}
		void add_port(PortBit * const pb){
			if (pb->get_type() == DIR_IN)
				inputs.push_back(pb);
			else
				outputs.push_back(pb);
		}
		void remove_port(PortBit * const pb){
			std::vector<PortBit*>* ports = &outputs;
			if (pb->get_type()==DIR_IN)
				ports = &inputs;
			typedef std::vector<PortBit*>::iterator IT;
			IT port = std::find_if(ports->begin(), ports->end(), inst_eq_p<PortBit*>(pb));
			ports->erase(port);
		}

		PortBit* find_port_by_name(const std::string& n){
			typedef std::vector<PortBit*>::iterator IT;
			IT port = std::find_if(inputs.begin(), inputs.end(), name_eq_p<PortBit*>(n));
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
	private:
		std::vector<Gate*> gates;
		std::vector<Signal*> signals;
		int signal_counter;
		std::list<Signal*> own_signals;
	public:
		void set_name(const std::string& n){name = n;}
		std::string get_instance_name() const {return name;}
		void add_gate(Gate * const g){gates.push_back(g);}
		void remove_gate(Gate * const g){
			std::remove_if(gates.begin(),gates.end(),inst_eq_p<Gate*>(g));
		}
		Gate* find_gate(const std::string& name){
			typedef std::vector<Gate*>::iterator IT;
			IT g = std::find_if(gates.begin(), gates.end(), name_eq_p<Gate*>(name));
			return *g;
		}
		void add_signal(Signal * const s){signals.push_back(s);}
		void remove_signal(Signal * const s){
			std::remove_if(signals.begin(),signals.end(),inst_eq_p<Signal*>(s));
		}
		Signal* find_signal(Signal * const name){
			typedef std::vector<Signal*>::iterator IT;
			IT s = std::find_if(signals.begin(), signals.end(), inst_eq_p<Signal*>(name));
			return *s;
		}
		Signal* create_signal(){
			static int counter = 0;
			std::string name = "ds_" + counter++;
			Signal* s = new Signal(name);
			own_signals.push_back(s);
			return s;
		}
		void join(Signal* a, const Signal& v){
			void (Signal::*fptr) (PortBit* pb) = &Signal::add_receiver;
			(a->*fptr)(0);
			std::for_each(v.get_receivers().begin(),
					v.get_receivers().end(),
					std::bind1st(std::mem_fun(&Signal::add_receiver), a));
		}
		~NetList(){
			std::for_each(gates.begin(), gates.end(), boost::checked_deleter<Gate>());
			std::for_each(signals.begin(), signals.end(), boost::checked_deleter<Signal>());
			std::for_each(own_signals.begin(), own_signals.end(), boost::checked_deleter<Signal>());
		}
	};

	class GateDef {

	};

}

#endif
