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
	/*!
	 * returns the node name of this pin
	 * @param lg working leveled graph
	 * @return
	 */
	virtual std::string get_node_name(ds_lg::LeveledGraph* lg)=0;
	/*!
	 * returns the netlist port name
	 * @param lg working leveled graph
	 * @return
	 */
	virtual std::string get_port_name(ds_lg::LeveledGraph* lg)=0;
	/*!
	 * returns the qualified name of this pin (with gate hierarchy)
	 * @return
	 */
	virtual std::string get_qualified_name()=0;
	/*!
	 * true if pin is an input
	 * @return
	 */
	virtual bool is_input() = 0;
	/*!
	 * returns true if pin is an output
	 * @return
	 */
	virtual bool is_output() = 0;
};

/*!
 * This callback method is executed after the first simulation pass to activate a fault
 */
class SimulationHook{
public:
	virtual ds_lg::LGNode* hook(ds_lg::LeveledGraph* lg)=0;
};

/*!
 * Conditional fault. Only active if its activation condition evaluates to true.
 */
class ConditionalFault : public SimulationHook{
protected:
	ds_lg::lg_v64* victim;			//!< victim value in the leveled graph
	ds_lg::int64 mask;				//!< fault is only is mask bit position is true
public:

	/*!
	 * victim pointer and mask are initialized
	 * @param v victim primitive value
	 * @param mask mask
	 */
	ConditionalFault(ds_lg::lg_v64* v, ds_lg::int64 m): victim(v), mask(m){}
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
	 * activation condition for this fault. Derived classes implement this method to set the behavior of a fault
	 * @return true if fault is active
	 */
	virtual ds_lg::int64 activate()=0;
	/*!
	 * changes the simulation value in the leveled graph if the activation condition is met
	 * @param nl netlist
	 * @returns pointer to the binded node where the fault is activated. Null pointer of the fault is not activated
	 */
	virtual ds_lg::LGNode* hook(ds_lg::LeveledGraph* lg){
		ds_lg::LGNode* n = bind(lg);
		ds_lg::int64 a = activate();
		if (a!=0){
			victim->v = victim->v ^ (a & mask);
			return n;
		}
		return 0;
	}
	/**!
	 * finds victim primitive in the provided leveled graph
	 * @param lg leveled graph where the fault is injected
	 */
	virtual ds_lg::LGNode* bind(ds_lg::LeveledGraph* lg)=0;
};

/*!
 * Static faults depend only the the current logic state of the circuit. They are well-suited for combinational simulation.
 * Static faults are active if the specified nodes in the leveled graph have certain logic values
 */
class StaticFault : public ConditionalFault, PinReference {
protected:
	std::string node_name;							//!< node name
	std::string port_name;							//!< node port name
	bool isInput;								//!< true if input port
	bool isOutput;								//!< true if output port
	std::vector<ds_lg::lg_v64*> aggressors;		//!< list of simulation primitives to observe
	std::vector<ds_lg::lg_v64> polarity;		//!< conditions for the simulation primitives
public:

	/*!
	 * constructs a static fault for simulation. This intermediate fault representation can be injected into any
	 * compatible leveled graph instance produced by the same netlist
	 * @param nl netlist for name resolution
	 * @param g gate name in netlist
	 * @param p port name in gate
	 */
	StaticFault(ds_structural::NetList* nl, std::string g, std::string p):ConditionalFault(0,-1L),isInput(false),isOutput(false){

		using ds_structural::Gate;
		using ds_lg::LGNode;
		using ds_lg::lg_v64;

		Gate *component = nl->find_gate(g);
		LGNode *n = component->get_lgn();
		// save port name
		node_name = n->get_name();
		//translate port name in the netlist to primitive node name
		std::string port_name = component->get_mapping(p);
		// find an output
		if (n->get_output(port_name)==0){
			//no output found, try an input
			if (n->get_input(port_name)!=0){
				isInput = true;
			}
		} else {
			isOutput = true;
		}
	}

	virtual ds_lg::LGNode* bind(ds_lg::LeveledGraph* lg){
		using ds_structural::Gate;
		using ds_lg::LGNode;
		using ds_lg::lg_v64;
		LGNode *n = lg->get_node(node_name);
		// find an output
		victim = n->get_output(port_name);
		if (victim==0){
			//no output found, try an input
			victim = *n->get_input(port_name);
			return n;

		}
		return 0;
	}

	/*!
	 * A fault is active if all conditions for the observed nodes are true.
	 * If an 'X' value is specified in the condition, it evaluates to true if the observed values is also an 'X' and logic values are ignored.
	 * Otherwise, it is checked if the logical value in the simulation primitive matches that specified in the condition
	 * @return true if static fault is active
	 */
	virtual ds_lg::int64 activate(){
		ds_lg::int64 active = 0L;
		for(std::size_t i=0;i<aggressors.size();i++){
			if (polarity[i].x != 0){
				active &= aggressors[i]->x ^ polarity[i].x;
			} else {
				active &= (aggressors[i]->v ^ polarity[i].v) ;
			}
		}
		return active;
	}
	/*!
	 * adds a new node to observe, together with its required value for fault activation
	 * @param value simulation primitive to observe
	 * @param p required simulation value
	 */
	void add_condition(ds_lg::lg_v64* value, const ds_simulation::Value& p){
		if (p==ds_simulation::BIT_X){
			aggressors.push_back(value);
			polarity.push_back(ds_lg::lg_v64(0L,-1L));
		} else if (p==ds_simulation::BIT_0){
			aggressors.push_back(value);
			polarity.push_back(ds_lg::lg_v64(0L,0L));
		} else if (p==ds_simulation::BIT_1){
			aggressors.push_back(value);
			polarity.push_back(ds_lg::lg_v64(-1L,0L));
		}
	}

	/*!
	 * queries the name of the equivalent gate in the netlist
	 * @param lg working leveled graph
	 * @return name of the equivalent gate in the netlist
	 */
	virtual std::string get_node_name(ds_lg::LeveledGraph* lg){
		ds_lg::LGNode *n = lg->get_node(node_name);
		return n->get_name();
	}

	/*!
	 * queries the name of the equivalent port in the netlist
	 * @param lg working leveled graph
	 * @return name of the equivalent port in the netlist
	 */
	virtual std::string get_port_name(ds_lg::LeveledGraph* lg){
		ds_structural::PortBit *pb = get_port_bit(lg);
		if (pb!=0)
			return pb->get_instance_name();
		return "";
	}
	virtual std::string get_qualified_name(ds_lg::LeveledGraph* lg){
		ds_structural::PortBit *pb = get_port_bit(lg);
		if (pb!=0)
			return pb->get_qualified_name();
		return "";
	}
	virtual bool is_input(){return isInput;}
	virtual bool is_output(){return isOutput;}

protected:
	/*!
	 * finds the equivalent port bit in the netlist
	 * @param lg working leveled graph
	 * @return
	 */
	ds_structural::PortBit* get_port_bit(ds_lg::LeveledGraph* lg){
		ds_lg::LGNode *n = lg->get_node(node_name);
		// identify gate
		ds_structural::Gate *g = n->get_gate();

		// mappings are unidirectional, name of port has to be searched explicitly
		// search input ports
		const ds_structural::port_container *ports = g->get_inputs();
		auto port = std::find_if(ports->begin(), ports->end(),
				[&](ds_structural::PortBit* p) {
				return g->get_instance_name() == p->get_instance_name();
		});
		if (port != ports->end())
			return *port;
		// no matching port found: search input ports
		ports = g->get_outputs();
		port = std::find_if(ports->begin(), ports->end(),
				[&](ds_structural::PortBit* p) {
				return g->get_instance_name() == p->get_instance_name();
		});
		if (port != ports->end())
			return *port;
		return 0;
	}
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
	StuckAt(ds_structural::NetList* nl, std::string g, std::string p, ds_simulation::Value v):StaticFault(nl,g,p),value(v){
		if (victim!=0){
			add_condition(victim, value);
		}

	}
};

}


#endif /* DS_FAULTS_H_ */
