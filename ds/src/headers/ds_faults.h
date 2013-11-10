/*
 * ds_faults.h
 *
 *  Created on: 30.10.2013
 *      Author: cookao
 */

#ifndef DS_FAULTS_H_
#define DS_FAULTS_H_

#include "ds_lg.h"

namespace ds_faults {

/*!
 * this interface identifies the a pin in a leveled graph
 */
class PinReference {
public:
	/*!
	 * returns the gate name of this pin
	 * @param lg working leveled graph
	 * @return
	 */
	virtual std::string get_gate_name() const = 0;
	/*!
	 * returns the netlist port name
	 * @param lg working leveled graph
	 * @return
	 */
	virtual std::string get_port_name() const = 0;
	/*!
	 * returns the qualified name of this pin (with gate hierarchy)
	 * @return
	 */
	virtual std::string get_qualified_name() const =0;
	/*!
	 * true if pin is an input
	 * @return
	 */
	virtual bool is_input() const = 0;
	/*!
	 * returns true if pin is an output
	 * @return
	 */
	virtual bool is_output() const = 0;
};

ds_lg::lg_v64* resolve(const PinReference *pr, ds_lg::LeveledGraph *lg);

class SimulationHook{
public:
	/*!
	 * activation condition for this fault. Derived classes implement this method to set the behavior of a fault
	 * @return 1 in bit position i if fault is active in slot i
	 */
	virtual ds_lg::int64 hook(ds_lg::LeveledGraph *lg) const = 0;
	/*!
	 * returns the port name in the leveled graph node where the fault is inserted
	 * @return
	 */
	virtual std::string get_hook_port() const;
	/*!
	 * returns the leveled graph node where the fault is iserted
	 * @return
	 */
	virtual ds_lg::LGNode* get_hook_node() const;
};

/*!
 * Static faults depend only the the current logic state of the circuit. They are well-suited for combinational simulation.
 * Static faults are active if the specified nodes in the leveled graph have certain logic values
 */
class StaticFault : public SimulationHook, public PinReference {
protected:

	std::string node_name;						//!< node name
	std::string node_port_name;					//!< node port name
	bool isInput;								//!< true if input port
	bool isOutput;								//!< true if output port
	std::vector<const PinReference*> aggressors;		//!< list of simulation primitives to observe
	std::vector<ds_lg::lg_v64> polarity;		//!< conditions for the simulation primitives
	ds_lg::int64 mask;							//!< fault is only is mask bit position is true

	std::string gate_name;
	std::string gate_port_name;
public:

	/*
	 * mask setter and getter
	 */
	ds_lg::int64 get_mask() const {
		return mask;
	}
	void set_mask (const ds_lg::int64& m){
		mask = m;
	}
	/*!
	 * constructs a static fault for simulation. This intermediate fault representation can be injected into any
	 * compatible leveled graph instance produced by the same netlist
	 * @param nl netlist for name resolution
	 * @param g gate name in netlist
	 * @param p port name in gate
	 */
	StaticFault(ds_structural::NetList* nl, std::string g, std::string p):isInput(false),isOutput(false),mask(-1L),gate_name(g),gate_port_name(p){

		if (g == nl->get_instance_name()){
			node_name = p;
			ds_structural::PortBit *pb = nl->find_port_by_name(p);
			if (pb->get_type()==ds_structural::DIR_IN){
				isInput = true;
			}
			if (pb->get_type()==ds_structural::DIR_OUT){
				isOutput = true;
			}
		} else {

			ds_structural::Gate *gate = nl->find_gate(gate_name);
			node_name = gate_name;
			//translate port name in the netlist to primitive node name
			std::string port_node_name = gate->get_mapping(gate_port_name);
			ds_structural::PortBit *pb = gate->find_port_by_name(gate_port_name);
			if (pb->get_type()==ds_structural::DIR_IN){
				isInput = true;
			}
			if (pb->get_type()==ds_structural::DIR_OUT){
				isOutput = true;
			}
		}
	}

	virtual ds_lg::LGNode* get_hook_node(ds_lg::LeveledGraph* lg) const {
		ds_lg::LGNode *n = lg->get_node(node_name);
		return n;
	}
	/*!
	 * A fault is active if all conditions for the observed nodes are true.
	 * If an 'X' value is specified in the condition, it evaluates to true if the observed values is also an 'X' and logic values are ignored.
	 * Otherwise, it is checked if the logical value in the simulation primitive matches that specified in the condition
	 * @return true if static fault is active
	 */
	virtual ds_lg::int64 hook(ds_lg::LeveledGraph* lg) const{
		ds_lg::int64 active = 0L;
		for(std::size_t i=0;i<aggressors.size();i++){
			ds_lg::lg_v64 *agg_value = ds_faults::resolve(aggressors[i], lg);
			if (polarity[i].x != 0){
				active &= agg_value->x ^ polarity[i].x;
			} else {
				active &= agg_value->v ^ polarity[i].v ;
			}
		}
		return active;
	}
	/*!
	 * adds a new node to observe, together with its required value for fault activation
	 * @param pin to observe
	 * @param v required simulation value
	 */
	void add_condition(const PinReference* pin, const ds_simulation::Value& v){
		if (v==ds_simulation::BIT_X){
			aggressors.push_back(pin);
			polarity.push_back(ds_lg::lg_v64(0L,-1L));
		} else if (v==ds_simulation::BIT_0){
			aggressors.push_back(pin);
			polarity.push_back(ds_lg::lg_v64(0L,0L));
		} else if (v==ds_simulation::BIT_1){
			aggressors.push_back(pin);
			polarity.push_back(ds_lg::lg_v64(-1L,0L));
		}
	}

	/*!
	 * queries the name of the equivalent gate in the netlist
	 * @return name of the equivalent gate in the netlist
	 */
	virtual std::string get_gate_name() const{
		return gate_name;
	}
	/*!
	 * queries the name of the equivalent port in the netlist
	 * @return name of the equivalent port in the netlist
	 */
	virtual std::string get_port_name() const{
		return gate_port_name;
	}
	virtual std::string get_qualified_name() const {
		return gate_name + "/" + gate_port_name;
	}
	virtual bool is_input(){return isInput;}
	virtual bool is_output(){return isOutput;}
};

/*!
 * realizes a stuck-at fault for simulation. The name, port and value of the fault are stored internally.
 */
class StuckAt : public StaticFault {


	ds_simulation::Value value;		//!< stuck-at value

	/*!
	 * Constructs a new stuck-at fault for simulation. It adds a single observed node (also the victim node)
	 * @param nl netlist where this fault is injected
	 * @param g gate name
	 * @param p port name
	 * @param v stuck-at value. Behavior is undefined if 'X' is provided
	 */
	StuckAt(ds_structural::NetList* nl, std::string g, std::string p, ds_simulation::Value v):StaticFault(nl,g,p),value(ds_simulation::BIT_X){
		if (v == ds_simulation::BIT_0){
			value = ds_simulation::BIT_1;
		} else if (v == ds_simulation::BIT_1)
			value = ds_simulation::BIT_0;
		add_condition(this,value);
	}
};

/*!
 * Descriptor of a stuck-at fault.
 * It holds the gate name, port name and stuck-at value
 */
struct SAFaultDescriptor {
	std::string gate_name;	//!< gate name in the netlist where the fault resides
	std::string port_name;	//!< netlist name of the port where the fault is injected
	ds_simulation::Value value;	//!< stuck-at fault value. Undefined behavior for 'X'
	SAFaultDescriptor(): gate_name(""), port_name(""), value(ds_simulation::BIT_X){}
	SAFaultDescriptor(std::string g, std::string p, ds_simulation::Value v): gate_name(g), port_name(p), value(v){}
	SAFaultDescriptor(const SAFaultDescriptor& d): gate_name(d.gate_name), port_name(d.port_name), value(d.value){}
	/*!
	 * returns a string representation of this descriptor
	 * @return
	 */
	std::string get_string() const {
		std::string v = value == ds_simulation::BIT_0 ? "0" : "1";
		return gate_name + "/" + port_name + ":" + v;
	}
	/*!
	 * defines a strict weak ordering for fault descriptors
	 * @param f
	 * @return true if this descriptor is less than the argument
	 */
	bool operator<(const SAFaultDescriptor& f) const {
		return get_string() < f.get_string();
	}
};

/*!
 * gets all equivalence fault classes of a gate
 * @param g gate to process
 * @param fault_class fault classes are inserted in this container. Each entry in the container has a list of equivalent faults
 */
void get_fault_classes(ds_structural::Gate* g, std::vector<std::list<SAFaultDescriptor> >& fault_class);

/*!
 * gets equivalent fault classes of a given gate. The function tyoe of the gate is queried in the workspace. The first definition in any
 * available library is used. Equivalent faults are identified and pushed into the same class.
 * @param g gate to process
 * @param c controlling value of gate
 * @param i inverting value of gate
 * @param fault_class fault classes are inserted in this container. Each entry in the container has a list of equivalent faults
 */
void get_gate_faults(ds_structural::Gate* g, ds_simulation::Value c, ds_simulation::Value i, std::vector<std::list<SAFaultDescriptor> >& fault_class);

/*!
 * generates one stuck-at-0 and one stuck-at-1 fault for each of the provided ports
 * @param begin iterator pointing at the first port to be processed
 * @param end iterator pointing at the last port to be processed
 * @param fault_classes fault classes are inserted in this container. Each entry in the container has a list of equivalent faults
 */
void get_port_faults(ds_structural::port_container::const_iterator begin, ds_structural::port_container::const_iterator end, std::vector<std::list<SAFaultDescriptor> >& fault_classes);

/*!
 * gets all fault classes of a netlist after fault collapsing
 * @param nl netlist to process
 * @param fault_class fault classes are inserted in this container. Each entry in the container has a list of equivalent faults
 */
void get_fault_classes(ds_structural::NetList* nl, std::map<SAFaultDescriptor, std::list<SAFaultDescriptor> >& fault_class);

}
#endif /* DS_FAULTS_H_ */
