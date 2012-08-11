/*
 * ds_common.h
 *
 *  Created on: Jun 29, 2012
 *      Author: cookao
 */

#ifndef DS_COMMON_H_
#define DS_COMMON_H_

#include <string>
#include <list>
#include <algorithm>
#include <ds_library.h>

namespace ds_common {

	typedef long int64;

	class Signal{
	private:
		std::string name;
		std::list<PortBit*> ports;
	public:
		Signal(const std::string& name):name(name) {}
		void add_receiver(PortBit* pb) {ports.push_back(pb); pb->set_signal(*this);}
		void remove_receiver(PortBit* pb) {ports.remove(pb); pb->disconnect();}
		const std::list<PortBit>& get_receivers() const {return ports;}

	};

	class PortBit {
	private:
		std::string name;
		Signal* signal;
		Gate& gate;
		PortType type;
	public:
		PortBit(const std::string n, const Signal& s, const Gate& g, const PortType& t):name(n),signal(s),type(t){}
		PortBit(const std::string n, const Gate& g, const PortType& t):name(n),type(t){}
		PortBit(const std::string n, const PortType& t):name(n),type(t){}

		void setGate(const Gate& g){gate = g;}
		void disconnect(){signal = 0;}
		Gate& get_gate(){return gate;}
		std::string get_instance_name(){return name;}
		void set_signal(const Signal& s){signal = s;}
		Signal& get_signal(){return signal;}
		PortType get_type() const {return type;}

	};

	enum PortType {
		IN,
		OUT,
		INOUT
	};

	template <class T> bool inst_name (const T& a1, const T& a2){
		if (a1.get_instance_name() == a2.get_Instance_name())
			return true;
		return false;
	}

	template <class T> class name_eq : public std::unary_function<T,bool> {
		std::string arg2;
		explicit name_eq(const string& s): arg2(s){};
		bool operator()(const T& x)  {
			return x.get_instance_name() == x;
		}
	};

	template <class T> class inst_eq : public std::unary_function<T,bool> {
		T arg2;
		explicit inst_eq(const T& a): arg2(a){};
		bool operator()(const T& x)  {
			return x.get_instance_name() == arg2.get_instance_name();
		}
	};

	class Gate {
	private:
		std::string name;
		std::vector<PortBit> inputs;
		std::vector<PortBit> outputs;
	public:
		std::string get_instance_name(){return name;}
		const std::vector<PortBit>& get_inputs()const {return &inputs;}
		const std::vector<PortBit>& get_outputs()const {return &outputs;}
		void add_port(const PortBit& pb){
			if (pb.get_type() == IN)
				inputs.push_back(pb);
			else
				outputs.push_back(pb);
		}
		void remove_port(const PortBit& pb){
			std::vector<PortBit>& ports = outputs;
			if (pb.get_type()==IN)
				ports = inputs;
			typedef std::vector<PortBit>::iterator IT;
			IT port = std::find_if(ports.begin(), ports.end(), inst_eq<PortBit>(pb));
			ports.erase(port);
		}

		PortBit& find_port_by_name(std::string n){
			typedef std::vector<PortBit>::iterator IT;
			IT port = std::find_if(inputs.begin(), inputs.end(), name_eq<PortBit>(n));
			if (port != inputs.end())
				return port;
			port = std::find_if(outputs.begin(), outputs.end(), name_eq<PortBit>(n));
			if (port != outputs.end())
				return port;
			return 0;

		}
	};

	class NetList : public Gate{
	private:
		std::string name;
		std::vector<Gate> gates;
		std::vector<Signal> signals;
		int signal_counter;
		std::list<Signal> own_signals;

	public:
		std::string get_instance_name(){return name;}
		void add_gate(const Gate& g){gates.push_back(g);}
		void remove_gate(const Gate& g){
			std::remove_if(gates.begin(),gates.end(),inst_eq<Gate>(g));
		}
		Gate& find_gate(const std::string& name){
			typedef std::vector<Gate>::iterator IT;
			IT g = std::find_if(gates.begin(), gates.end(), name_eq<Gate>(name));
			return *g;
		}
		void add_signal(const Signal& s){signals.push_back(s);}
		void remove_signal(const Signal& s){
			std::remove_if(signals.begin(),signals.end(),name_eq<Signal>(s));
		}
		Gate& find_signal(const Signal& name){
			typedef std::vector<Signal>::iterator IT;
			IT s = std::find_if(signals.begin(), signals.end(), name_eq<Signal>(name));
			return *s;
		}
		Signal create_signal(){
			static int counter = 0;
			std::string name = "ds_" + counter++;
			Signal s = new Signal(name);
			own_signals.push_back(s);
			return s;
		}
		void join(Signal& a, const Signal& v){
			std::for_each(v.get_receivers().begin(), v.get_receivers().end(), a.add_receiver);
		}
		~Netlist();
	};

}

#endif /* DS_COMMON_H_ */
