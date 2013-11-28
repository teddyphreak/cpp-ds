/*
 * ds_faults.h
 *
 *  Created on: 30.10.2013
 *      Author: cookao
 */

#ifndef DS_FAULTS_H_
#define DS_FAULTS_H_

#include "ds_lg.h"
#include <boost/log/trivial.hpp>

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
	virtual std::string get_gate_name() const=0;
	/*!
	 * returns the netlist port name
	 * @param lg working leveled graph
	 * @return
	 */
	virtual std::string get_port_name() const=0;
	/*!
	 * returns the qualified name of this pin (with gate hierarchy)
	 * @return
	 */
	virtual std::string get_qualified_name() const=0;
	/*!
	 * true if pin is an input
	 * @return
	 */
	virtual bool is_input() const=0;
	/*!
	 * returns true if pin is an output
	 * @return
	 */
	virtual bool is_output() const=0;
};

/*!
 * returns a pointer to the corresponding simulation primitive in the leveled graph
 * @param pr pin reference to resolve
 * @param lg leveled graph where the reference is resolved
 * @return
 */

ds_lg::lg_v64* resolve(const PinReference *pr, ds_lg::LeveledGraph *lg);

class SimulationHook{
public:
	/*!
	 * activation condition for this fault. Derived classes implement this method to set the behavior of a fault
	 * @return 1 in bit position i if fault is active in slot i
	 */
	virtual ds_lg::int64 hook(ds_lg::LeveledGraph *lg) const=0;
	/*!
	 * returns the port name in the leveled graph node where the fault is inserted
	 * @return
	 */
	virtual std::string get_hook_port() const=0;
	/*!
	 * returns the graph node that owns this hook. This is the node whose behavior is affected by this hook
	 * @return
	 */
	virtual ds_lg::LGNode* get_hook_node(ds_lg::LeveledGraph *lg) const=0;

};

/*!
 * Static faults depend only the the current logic state of the circuit. They are well-suited for combinational simulation.
 * Static faults are active if the specified nodes in the leveled graph have certain logic values
 */
class StaticFault : public SimulationHook, public PinReference {
protected:

	std::string node_name;							//!< node name
	std::string node_port_name;						//!< node port name
	bool isInput;									//!< true if input port
	bool isOutput;									//!< true if output port
	std::vector<const PinReference*> aggressors;	//!< list of simulation primitives to observe
	std::vector<ds_lg::lg_v64> polarity;			//!< conditions for the simulation primitives
	ds_lg::int64 mask;								//!< fault is only is mask bit position is true
	std::string gate_name;
	std::string gate_port_name;
public:

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

			node_port_name = gate->get_mapping(gate_port_name);
			ds_structural::PortBit *pb = gate->find_port_by_name(gate_port_name);
			if (pb->get_type()==ds_structural::DIR_IN){
				isInput = true;
			}
			if (pb->get_type()==ds_structural::DIR_OUT){
				isOutput = true;
			}
		}
	}

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
	 * returns the port name in the simulation primitive
	 * @return
	 */
	virtual std::string get_hook_port() const {
		return node_port_name;
	}

	/*!
	 * returns the node where the fault is injected
	 * @param lg leveled graph where the fault is injected
	 * @return
	 */
	virtual ds_lg::LGNode* get_hook_node(ds_lg::LeveledGraph *lg) const {
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
		ds_lg::int64 active = -1L;
		for(std::size_t i=0;i<aggressors.size();i++){
			ds_lg::lg_v64 *agg_value = ds_faults::resolve(aggressors[i], lg);
			ds_common::int64 v = agg_value->v;
			ds_common::int64 p = polarity[i].v;
			active &= ~(v ^ p) ;
		}
		return active;
	}
	/*!
	 * adds a new node to observe, together with its required value for fault activation
	 * @param pin to observe
	 * @param v required simulation value
	 */
	void add_condition(const PinReference* pin, const ds_common::Value& v){
		if (v==ds_common::BIT_X){
			aggressors.push_back(pin);
			polarity.push_back(ds_lg::lg_v64(0L,-1L));
		} else if (v==ds_common::BIT_0){
			aggressors.push_back(pin);
			polarity.push_back(ds_lg::lg_v64(0L,0L));
		} else if (v==ds_common::BIT_1){
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
	/*
	 * describes the port type of this fault
	 */
	virtual bool is_input() const {return isInput;}
	virtual bool is_output() const {return isOutput;}
	/*
	 * male destructor virtual
	 */
	virtual ~StaticFault(){}
};

/*!
 * realizes a stuck-at fault for simulation. The name, port and value of the fault are stored internally.
 */
class StuckAt : public StaticFault {
	ds_common::Value value;		//!< stuck-at value
public:
	/*!
	 * Constructs a new stuck-at fault for simulation. It adds a single observed node (also the victim node)
	 * @param nl netlist where this fault is injected
	 * @param g gate name
	 * @param p port name
	 * @param v stuck-at value. Behavior is undefined if 'X' is provided
	 */
	StuckAt(ds_structural::NetList* nl, std::string g, std::string p, ds_common::Value v):StaticFault(nl,g,p),value(ds_common::BIT_X){
		if (v == ds_common::BIT_0){
			value = ds_common::BIT_1;
		} else if (v == ds_common::BIT_1)
			value = ds_common::BIT_0;
		add_condition(this,value);
	}

	virtual ~StuckAt(){}
};

/*!
 * Descriptor of a stuck-at fault.
 * It holds the gate name, port name and stuck-at value
 */
struct SAFaultDescriptor {
	std::string gate_name;	//!< gate name in the netlist where the fault resides
	std::string port_name;	//!< netlist name of the port where the fault is injected
	ds_common::Value value;	//!< stuck-at fault value. Undefined behavior for 'X'
	SAFaultDescriptor(): gate_name(""), port_name(""), value(ds_common::BIT_X){}
	SAFaultDescriptor(std::string g, std::string p, ds_common::Value v): gate_name(g), port_name(p), value(v){}
	SAFaultDescriptor(const SAFaultDescriptor& d): gate_name(d.gate_name), port_name(d.port_name), value(d.value){}
	/*!
	 * returns a string representation of this descriptor
	 * @return
	 */
	std::string get_string() const {
		std::string v = value == ds_common::BIT_0 ? "0" : "1";
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
void get_fault_classes(ds_structural::Gate* g, std::vector<std::list<SAFaultDescriptor*>* >& fault_class);

/*!
 * gets equivalent fault classes of a given gate. The function type of the gate is queried in the workspace. The first definition in any
 * available library is used. Equivalent faults are identified and pushed into the same class.
 * @param g gate to process
 * @param c controlling value of gate
 * @param i inverting value of gate
 * @param fault_class fault classes are inserted in this container. Each entry in the container has a list of equivalent faults
 */
void get_gate_faults(ds_structural::Gate* g, ds_common::Value c, ds_common::Value i, std::vector<std::list<SAFaultDescriptor*>* >& fault_class);

/*!
 * generates one stuck-at-0 and one stuck-at-1 fault for each of the provided ports
 * @param begin iterator pointing at the first port to be processed
 * @param end iterator pointing at the last port to be processed
 * @param fault_classes fault classes are inserted in this container. Each entry in the container has a list of equivalent faults
 */
void get_port_faults(ds_structural::port_container::const_iterator begin, ds_structural::port_container::const_iterator end, std::vector<std::list<SAFaultDescriptor*>* >& fault_classes);

/*!
 * gets all fault classes of a netlist after fault collapsing
 * @param nl netlist to process
 * @param fault_class fault classes are inserted in this container. Each entry in the container has a list of equivalent faults
 */
std::map<SAFaultDescriptor*, std::list<SAFaultDescriptor*>* >* get_fault_universe(ds_structural::NetList* nl);

/*!
 * returns a set of structurally equivalent fault descriptors in the provided netlist. Descriptors are allocated on the
 * heap and need to by deallocated by the caller
 * @param nl netlist to analyze
 * @return
 */
std::set<SAFaultDescriptor*>* get_collapsed_faults(ds_structural::NetList* nl);

/*!
 * deletes all descriptor pointers contained in the provided equivalent fault classes
 * @param faults equivalent fault classes to delete
 */
void delete_faults(std::map<SAFaultDescriptor*, std::list<SAFaultDescriptor*>* >* faults);

/*!
 * deletes all descriptor pointers contained in the pset
 * @param faults to delete
 */
void delete_faults(std::set<SAFaultDescriptor*>* faults);

enum FaultCategory {
	UK,		//!< unknown
	DR,		//!< detected robustly
	DS,		//!< detected by simulation
	DI,		//!< detected by implication
	PT,		//!< possibly detected
	AP,		//!< ATPG untestable-possibly detected
	NP,		//!< not analyzed-possibly detected
	UD,		//!< undetectable
	UU,		//!< undetectable unused
	UT,		//!< undetectable tied
	UB,		//!< undetectable blocked
	UR,		//!< undetectable redundant
	AU,		//!< ATPG untestable
	AN,		//!< ATPG untestable-not detected
	ND,		//!< not detected, aborted
	NC,		//!< not controlled
	NO,		//!< not observed
	DT		//!< detected
};

class FaultList {

	/// probability of detecting a fault when the simulation value is X and the expected value is [0|1]
	const double X_WEIGHT = 0.5;

	std::set<SAFaultDescriptor*> uk; //!< unprocessed fault set
	std::set<SAFaultDescriptor*> ds; //!< detected by simulation fault set
	std::set<SAFaultDescriptor*> np; //!< probably detected fault set
	std::set<SAFaultDescriptor*> ud; //!< undetected fault set

	/// maps all fault descriptors to its representative fault descriptor
	std::map<std::string, SAFaultDescriptor*> representatives;
	/// keeps track of the current category for each representative descriptor
	std::map<SAFaultDescriptor*, FaultCategory> fault_map;

public:
	/*!
	 * generates equivalent fault categories and selects representatives faults for the provided netlist
	 * @param nl netlist for which fault descriptors are generated
	 */
	FaultList(ds_structural::NetList* nl);
	/*!
	 * deletes all generated fault descriptors
	 */
	~FaultList();
	/*!
	 * resets the categories of all descriptors to the "unprocessed" UK category
	 */
	void reset_fault_categories();
	/*!
	 * returns the fault category for the descriptor represented by the provided fault
	 * @param fault string representation of descripor to query (usually generated with SAFaultDescriptor::get_string())
	 * @return
	 */
	FaultCategory get_fault_category(const std::string& fault) {
		auto it = representatives.find(fault);
		if (it!=representatives.end()){
			SAFaultDescriptor* d = representatives[fault];
			return fault_map[d];
		}
	}
	/*!
	 * sets the fault category of the descriptor corresponding to the provided structural netlist data
	 * @param n netlist gate name
	 * @param p netlist port name
	 * @param v fault value
	 * @param cat fault category
	 */
	void set_fault_category(std::string n, std::string p, const ds_common::Value& v, const ds_faults::FaultCategory& cat);
	/*!
	 * sets the fault category of the provided descriptor pointer
	 * @param f pointer to the descrptor whose category is set
	 * @param category fault category
	 */
	void set_fault_category(SAFaultDescriptor *f, const FaultCategory& category);

	/*!
	 * copies all detected descriptors to the specified container
	 * @param container fault destination
	 */
	template<typename Container>
	void get_detected_faults(Container& container){
		typename Container::iterator it = container.begin();
		std::insert_iterator<Container> insert_it(container, it);
		std::copy(ds.begin(), ds.end(), insert_it);
	}
	/*!
	 * copies all possibly detected descriptors to the specified container
	 * @param container fault destination
	 */
	template<typename Container>
	void get_possibly_detected_faults(Container& container){
		typename Container::iterator it = container.begin();
		std::insert_iterator<Container> insert_it(container, it);
		std::copy(np.begin(), np.end(), insert_it);
	}
	/*!
	 * copies all undetected descriptors to the specified container
	 * @param container fault destination
	 */
	template<typename Container>
	void get_undetected_faults(Container& container){
		typename Container::iterator it = container.begin();
		std::insert_iterator<Container> insert_it(container, it);
		std::copy(uk.begin(), uk.end(), insert_it);
		std::copy(ud.begin(), ud.end(), insert_it);
	}
	/*!
	 * calculates the fault coverage: (# of detected faults + X_WEIGHT * # of possible detected faults ) / # of faults;
	 * @return the fault coverage
	 */
	double get_fc() const;
private:
	/*!
	 * reurns the container matching the provided fault category
	 * @param category fault category to search
	 * @return
	 */
	std::set<SAFaultDescriptor*>* find_container(const FaultCategory& category);

};

}
#endif /* DS_FAULTS_H_ */
