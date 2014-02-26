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
#include <boost/spirit/include/qi.hpp>
#include <boost/fusion/include/adapt_struct.hpp>

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

template<class T, class V>
V* resolve(const PinReference *pr, ds_lg::Resolver<T> *res){
	std::string name = pr->get_gate_name();
	std::string port = pr->get_port_name();
	T *n = res->get_node(name);
	// try a gate first
	if (n!=0){
		std::string hook_port = res->get_port_name(name, port);
		if (pr->is_input()){
			V *p = *n->get_input(hook_port);
			return p;
		}
		if (pr->is_output()){
			V *p = n->get_output(hook_port);
			return p;
		}
	} else {
		// no gate found. Try a primary input / output
		n = res->get_node(port);
		if (pr->is_input()){
			return n->get_output("o");		// only one port possible
		}
		if (pr->is_output()){
			return *n->get_input("a");		// only one port possible
		}
	}
	return 0;
}

/*!
 * Static faults depend only the the current logic state of the circuit. They are well-suited for combinational simulation.
 * Static faults are active if the specified nodes in the leveled graph have certain logic values
 */
template<class N, class T>
class StaticFault : public ds_lg::SimulationHook<N>, public PinReference {
protected:

	std::string node_name;							//!< node name
	std::string node_port_name;						//!< node port name
	bool isInput;									//!< true if input port
	bool isOutput;									//!< true if output port
	std::vector<const PinReference*> aggressors;	//!< list of simulation primitives to observe
	std::vector<T> polarity;						//!< conditions for the simulation primitives
	ds_lg::int64 mask;								//!< fault is only is mask bit position is true
	std::string gate_name;
	std::string gate_port_name;
public:

	virtual ds_lg::int64 compare(const T* a, const T& b) const = 0;
	virtual T convert(const ds_common::Value& v) const = 0;

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
	virtual N* get_hook_node(ds_lg::Resolver<N> *res) const {
		N *n = res->get_node(node_name);
		return n;
	}
	/*!
	 * A fault is active if all conditions for the observed nodes are true.
	 * If an 'X' value is specified in the condition, it evaluates to true if the observed values is also an 'X' and logic values are ignored.
	 * Otherwise, it is checked if the logical value in the simulation primitive matches that specified in the condition
	 * @return true if static fault is active
	 */
	virtual ds_lg::int64 hook(ds_lg::Resolver<N>* res) const{
		ds_lg::int64 active = -1L;
		for(std::size_t i=0;i<aggressors.size();i++){
			T *agg_value = ds_faults::resolve<N, T>(aggressors[i], res);
			active &= compare(agg_value, polarity[i]);
		}
		return active;
	}
	/*!
	 * adds a new node to observe, together with its required value for fault activation
	 * @param pin to observe
	 * @param v required simulation value
	 */
	void add_condition(const PinReference* pin, const ds_common::Value& v){

		T val = convert(v);
		aggressors.push_back(pin);
		polarity.push_back(val);
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
class StuckAt : public StaticFault<ds_lg::LogicNode, ds_lg::lg_v64> {
	ds_common::Value value;		//!< stuck-at value
public:
	/*!
	 * Constructs a new stuck-at fault for simulation. It adds a single observed node (also the victim node)
	 * @param nl netlist where this fault is injected
	 * @param g gate name
	 * @param p port name
	 * @param v stuck-at value. Behavior is undefined if 'X' is provided
	 */
	StuckAt(ds_structural::NetList* nl, std::string g, std::string p, ds_common::Value v):
		StaticFault<ds_lg::LogicNode, ds_lg::lg_v64>(nl, g, p), value(ds_common::BIT_X){
		if (v == ds_common::BIT_0){
			value = ds_common::BIT_1;
		} else if (v == ds_common::BIT_1)
			value = ds_common::BIT_0;
		add_condition(this,value);
	}

	virtual ds_lg::int64 compare(const ds_lg::lg_v64* a, const ds_lg::lg_v64& b) const{
		ds_common::int64 v = a->v;
		ds_common::int64 p = b.v;
		return ~(v ^ p) ;
	}

	virtual ds_lg::lg_v64 convert(const ds_common::Value& v) const {
		if (v==ds_common::BIT_0){
			return ds_lg::lg_v64(0L,0L);
		} else if (v==ds_common::BIT_1){
			return ds_lg::lg_v64(-1L,0L);
		}
		return ds_lg::lg_v64(0L,-1L);
	}

	virtual ~StuckAt(){}
};

class TransitionFault: public StaticFault<ds_lg::TNode, ds_lg::driver_v64>{
	ds_common::Value value;		//!< transition value
public:
	/*!
	 * Constructs a new transition delay fault for simulation. It adds a single observed node (also the victim node)
	 * @param nl netlist where this fault is injected
	 * @param g gate name
	 * @param p port name
	 * @param v transition value. Behavior is undefined if 'X' is provided
	 */
	TransitionFault(ds_structural::NetList* nl, std::string g, std::string p, ds_common::Value v):
		StaticFault<ds_lg::TNode, ds_lg::driver_v64>(nl,g,p), value(ds_common::BIT_X){
		if (v == ds_common::BIT_0){
			value = ds_common::BIT_1;
		} else if (v == ds_common::BIT_1)
			value = ds_common::BIT_0;
		add_condition(this,value);
	}

	virtual ds_lg::int64 compare(const ds_lg::driver_v64* a, const ds_lg::driver_v64& b) const{
		ds_lg::lg_v64 v = a->value;
		ds_lg::lg_v64 t = b.value;
		const ds_lg::TNode *driver = a->driver;
		const ds_lg::lg_v64 *p = driver->get_previous_value(a);
		return ~(v.v ^ t.v) & (p->v ^ v.v) & ~v.x & ~t.x & ~p->x  ;
	}

	virtual ds_lg::driver_v64 convert(const ds_common::Value& v) const {
		if (v==ds_common::BIT_0){
			return ds_lg::driver_v64(0L,0L);
		} else if (v==ds_common::BIT_1){
			return ds_lg::driver_v64(-1L,0L);
		}
		return ds_lg::driver_v64(0L,-1L);
	}
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

struct fastscan_descriptor{
	char type;
	std::string code;
	std::string path_name;
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

	template<typename I>
	FaultList(I begin, I end){
		for (I it=begin;it!=end;it++){
			fastscan_descriptor d = *it;
			ds_common::Value v = ds_common::BIT_X;
			if (d.type == '1'){
				v = ds_common::BIT_1;
			} else if (d.type == '0'){
				v = ds_common::BIT_0;
			}
			std::size_t marker = d.path_name.find_last_of('/');
			std::string gate_name = d.path_name.substr(0, marker);
			std::string port_name = d.path_name.substr(marker+1, std::string::npos);
			SAFaultDescriptor* f = new SAFaultDescriptor(gate_name, port_name, v);
			uk.insert(f);
			fault_map[f] = UK;
		}
	}

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

BOOST_FUSION_ADAPT_STRUCT(
    ds_faults::fastscan_descriptor,
    (char, type)
    (std::string, code)
    (std::string, path_name)
)

namespace ds_faults{

namespace qi = boost::spirit::qi;
template <typename Iterator>
struct fastscan_fault_parser : qi::grammar<Iterator,fastscan_descriptor()>
{

	fastscan_fault_parser() : fastscan_fault_parser::base_type(start)
	{
		namespace qi = boost::spirit::qi;
		namespace ascii = boost::spirit::ascii;
		using qi::lit;

		start %=
				type >> code >> path_name;

		path_name = -lit('/') >> qi::char_("a-zA-Z_") >> *qi::char_("a-zA-Z_0-9/");
		code = +qi::char_("A-Z");
		type = qi::char_("01");
	}

	qi::rule<Iterator, std::string()> path_name;
	qi::rule<Iterator, std::string()> code;
	qi::rule<Iterator, char()> type;
	qi::rule<Iterator, fastscan_descriptor()> start;
};

template <typename Iterator>
bool parse_fastscan_faults(Iterator first, Iterator last, std::vector<fastscan_descriptor>& descriptors){

	fastscan_descriptor d;
	fastscan_fault_parser<Iterator> p;

	//parse library
	bool parse =  qi::phrase_parse(first, last, p, boost::spirit::ascii::space, d);
	if (parse){
		descriptors.push_back(d);
	}
	return parse;
}

void read_fastscan_descriptors(const std::string& file_name, std::vector<fastscan_descriptor>& descriptors);

}
#endif /* DS_FAULTS_H_ */
