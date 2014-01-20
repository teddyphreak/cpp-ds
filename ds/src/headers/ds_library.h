/*
 * library.h
 *
 *  Created on: Jul 5, 2012
 *      Author: cookao
 */

#ifndef LIBRARY_H_
#define LIBRARY_H_

//#define BOOST_SPIRIT_DEBUG

#include "ds_common.h"
#include "ds_lg.h"
#include "stdio.h"
#include <fstream>
#include <map>
#include <boost/spirit/repository/include/qi_distinct.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>
#include <boost/spirit/include/phoenix_fusion.hpp>
#include <boost/config/warning_disable.hpp>
#include <boost/fusion/include/std_pair.hpp>
#include <boost/config/warning_disable.hpp>
#include <boost/variant.hpp>
#include <boost/spirit/include/phoenix_object.hpp>
#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/fusion/include/io.hpp>
#include <iostream>

namespace ds_workspace {
	class Workspace;
}

namespace ds_library {

	/*!
	 * generic logic functions
	 */
	enum LogicFunction {
		AND,
		OR,
		NOT,
		NAND,
		NOR,
		BUF,
		XOR,
		XNOR,
		MUX,
		UNKNOWN,
	};

	namespace qi = boost::spirit::qi;
	typedef std::map<std::string, ds_lg::bi_eval> function_map;
	typedef std::map<std::string, bool> inversion_map;
	typedef std::map<std::string, std::string> fusion_map;
	typedef std::map<std::string, ds_structural::Gate*> gate_map_t;
	typedef std::map<std::string, std::string> prototype_map_t;
	typedef std::map<std::string, ds_lg::LogicNode*> lgn_map_t;
	typedef std::map<ds_structural::Gate*, ds_lg::LogicNode*> node_map_t;
	typedef std::map<std::string, ds_library::LogicFunction> type_map_t;

	const std::string value_0 = "0";	//!< constant '0'
	const std::string value_1 = "1";	//!< constant '1'
	const std::string value_X = "X";	//!< unknown value

	class Library {

	friend class LibraryFactory;
	public:

	/*!
	 * queries whether a gate exists
	 * @param name name of the desired gate
	 * @param ports number of ports of the desired gate
	 * @return true if gate exists
	 */
	bool has_gate(const std::string& name, const std::size_t ports) {
		gate_map_t::const_iterator it = gate_map.find(name);
		bool found = false;
		if (it != gate_map.end()){
			ds_structural::Gate *c = it->second;
			if (c->get_num_ports() == ports)
				found = true;
		}
		if (!found){
			function_map::const_iterator it = functions.find(name);
			if (it != functions.end()){
				found = true;
			}
		}
		return found;
	}

	/*!
	 * returns logic function of a given gate type
	 * @param t gate type to query
	 * @return logic function implemented by gate type
	 */
	ds_library::LogicFunction get_function(const std::string& t){
		if (types.find(t)!= types.end())
			return types[t];
		else
			return ds_library::UNKNOWN;
	}

	ds_lg::LogicNode* get_primitive(const std::string& name, const std::size_t ports){
		gate_map_t::const_iterator gate_it = gate_map.find(name);
		if (gate_it != gate_map.end()){
			ds_structural::Gate *c = gate_it->second;
			if (c->get_num_ports() == ports){
				std::string lgn_name = prototype_map[name];
				ds_lg::LogicNode *n = prototypes[lgn_name];
				ds_lg::LogicNode *copy = n->clone();
				return copy;
			}
		}
		function_map::const_iterator func_it = functions.find(name);
		if (func_it != functions.end()){
			ds_lg::bi_eval func = func_it->second;
			bool invert = inversion.find(name)->second;
			ds_lg::LogicNode *n = new ds_lg::LGNodeArr(name, ports-1, func, invert);
			return n;
		}
		return 0;
	}

	/*!
	 * retrieves a copy of the desired gate according to the prototype design pattern
	 * @param name name of desired gate
	 * @param ports number of ports of the desired gate
	 * @return deep copy of desired gate
	 */
	ds_structural::Gate* get_gate(const std::string& name, const std::size_t ports) const {
		ds_structural::Gate* g = 0;
		gate_map_t::const_iterator it = gate_map.find(name);
		if (it != gate_map.end()){
			ds_structural::Gate *c = it->second;
			if (c->get_num_ports() == ports)
				g = it->second->clone();
		}
		if (g==0) {
			function_map::const_iterator it = functions.find(name);
			if (it != functions.end()){
				//ds_lg::bi_eval func = it->second;
				//bool invert = inversion.find(name)->second;
				g = new ds_structural::Gate();
				g->set_type(name);
				ds_structural::PortBit *o_port = new ds_structural::PortBit("Z", ds_structural::DIR_OUT);
				o_port->set_gate(g);
				g->add_port(o_port);
				g->add_mapping("Z", "z");
				char lgn_port = 'a';
				for (std::size_t i='A';i<'A'+ports-1;i++){
					std::string port_name;
					port_name += (char)i;
					ds_structural::PortBit *i_port = new ds_structural::PortBit(port_name, ds_structural::DIR_IN);
					i_port->set_gate(g);
					g->add_port(i_port);
					std::string lgn_port_name;
					lgn_port_name += lgn_port;
					g->add_mapping(port_name, lgn_port_name);
					lgn_port++;
				}
			//	ds_lg::LogicNode *n = new ds_lg::LGNodeArr(name, ports-1, func, invert);
			//	g->set_lgn(n);
			}
		}
		return g;
	}

	/*!
	 * retrieves a copy of the desired gate according to the prototype design pattern
	 * @param name name of desired gate
	 * @param ports ports of the desired gate
	 * @return deep copy of desired gate
	 */

	ds_structural::Gate* get_gate(const std::string& name, const std::vector<std::string>& ports) const {
		ds_structural::Gate* g = 0;
		gate_map_t::const_iterator iterator = gate_map.find(name);
		if (iterator != gate_map.end()){
			ds_structural::Gate *c = iterator->second;
			bool found = true;
			std::vector<std::string>::const_iterator it = ports.begin();
			for (;it!=ports.end();it++){
				std::string port_name = *it;
				if (c->find_port_by_name(port_name)==0) {
					found = false;
				}
				if (found)
					g = iterator->second->clone();
			}
		}
		return g;
	}

	virtual ~Library(){close();}

	protected:
		gate_map_t gate_map; 		//!< available gates in the library
		lgn_map_t prototypes;		//!< simulation primitives
		type_map_t types;			//!< boolean logic implemented by each function
		function_map functions;		//!< evaluation functions of 2 operands
		inversion_map inversion;	//!< true when the function entry is to be complemented
		prototype_map_t prototype_map; //!<maps gate names to prototype names


		/*!
		 * populates the gate container in this library.
		 * Library definitions are searched in the "gate_lib" directory in the DS installation.
		 * The environmental variable "DS" points to the installation directory
		 * @param lib_name library name to load
		 */
		virtual void load(const std::string &lib_name);
		/*!
		 * loads primitive simulation prototypes
		 */
		virtual void load_nodes();
		/*!
		 * clear the library and deallocates memory. After this call the library is no longer available
		 */
		void close();

	private:
		/*!
		 * A library may not be created directly. The factory design pattern is sed instead
		 * @param lib_path path to library definition
		 */
		Library(const std::string& lib_path){
			load_nodes();
			load(lib_path);
		}
	};

	typedef std::map<std::string,Library*> library_map_t;

	/*!
	 * provides access to gate libraries. Implemented after the Singleton design pattern
	 */
	class LibraryFactory {

	private:

		static LibraryFactory* instance;	//!< singleton instance
		library_map_t map;					//!< available libraries
		LibraryFactory(){}

	public:
		static LibraryFactory* getInstance(){
			if (instance == 0)
				instance = new LibraryFactory();
			return instance;
		}

		/*!
		 * loads and returns the desired library
		 * @param name name of desired library
		 * @return the
		 */
		Library* load_library(const std::string& name="default_lib");

		/*!
		 * clears and deallocates the desired library
		 * @param name name of the library to remove
		 */
		void remove_library(const std::string name = "default_lib"){
			library_map_t::iterator it = map.find(name);
			if (it != map.end()){
				delete (it->second);
				map.erase(it);
			}
		}
	};

	/*!
	 * intermediate aggregate representation for verilog parsing
	 */
	struct parse_nl_aggregate
	{
		int left; 	//!< leftmost index
		int right;	//!< rightmost index
		std::string name;	//!< name of aggregate
	};
	/*!
	* intermediate assignment representation for verilog parsing
	*/
	struct parse_nl_assignment
	{
		std::string lhs; //!< left hand side value
		std::string rhs; //!< right hand side value
	};
	/*!
	 * intermediate implicit instance representation for verilog parsing
	 */
	struct parse_nl_implicit_instance
	{
		std::string type;				//!< gate type
		std::string name;				//!< gate name
		std::vector<std::string> ports;	//!< actual ports
	};
	/*!
	 *  intermediate explicit instance representation for verilog parsing
	 */
	struct parse_nl_explicit_instance
	{
		std::string type;							//!< gate type
		std::string name;							//!< gate name
		std::map<std::string, std::string> ports;	//!< port map (formal, actual)
	};

	typedef boost::variant<parse_nl_aggregate, std::string> verilog_declaration;
	typedef boost::variant<parse_nl_implicit_instance, parse_nl_explicit_instance> verilog_instance;

	/*!
	 * intermediate netlist representation for verilog parsing
	 */
	struct parse_netlist
	{
		std::string nl_name;							//!< netlist name
		std::vector<std::string> ports;					//!< netlist ports (all)
		std::vector<verilog_declaration> inputs;		//!< input ports
		std::vector<verilog_declaration> outputs;		//!< output ports
		std::vector<verilog_declaration> inouts;		//!< bidirectional ports
		std::vector<verilog_declaration> signals;		//!< signals
		std::vector<parse_nl_assignment> assignments;	//!< assignments ('assign' in verilog)
		std::vector<verilog_instance> instances;		//!< instances, either implicit or explicit
	};

	/*!
	 * intermediate node representation for library parsing
	 */
	struct parse_lib_node
	{
		std::string gate_name;							//!< library gate name
		std::string node_name;							//!< primitive simulation node name
		int outputs;									//!< gate outputs
		std::map<std::string, std::string> o_mapping;	//!< output mappings between gate and primitive
		int inputs;										//!< gate inputs
		std::map<std::string, std::string> i_mapping;	//!< input mappings between gate and primitive
		bool flexible;									//!< true if variable inputs are allowed
	};
}

/*
 * Wrapper auxiliary structures necessary for boost::qi
 */

BOOST_FUSION_ADAPT_STRUCT(
    ds_library::parse_nl_aggregate,
    (int, left)
    (int, right)
    (std::string, name)
)

BOOST_FUSION_ADAPT_STRUCT(
    ds_library::parse_nl_implicit_instance,
    (std::string, type)
    (std::string, name)
    (std::vector<std::string>, ports)
)

BOOST_FUSION_ADAPT_STRUCT(
    ds_library::parse_nl_explicit_instance,
    (std::string, type)
    (std::string, name)
    (ds_library::fusion_map, ports)
)

BOOST_FUSION_ADAPT_STRUCT(
    ds_library::parse_nl_assignment,
    (std::string, lhs)
    (std::string, rhs)
)

BOOST_FUSION_ADAPT_STRUCT(
    ds_library::parse_netlist,
    (std::string, nl_name)
	(std::vector<std::string>, ports)
	(std::vector<ds_library::verilog_declaration>, inputs)
	(std::vector<ds_library::verilog_declaration>, outputs)
	(std::vector<ds_library::verilog_declaration>, inouts)
	(std::vector<ds_library::verilog_declaration>, signals)
	(std::vector<ds_library::parse_nl_assignment>, assignments)
	(std::vector<ds_library::verilog_instance>, instances)
)

BOOST_FUSION_ADAPT_STRUCT(
    ds_library::parse_lib_node,
    (std::string, gate_name)
    (std::string, node_name)
    (int, outputs)
    (ds_library::fusion_map, o_mapping)
    (int, inputs)
    (ds_library::fusion_map, i_mapping)
    (bool, flexible)
)

namespace ds_library {

	namespace ascii = boost::spirit::ascii;

	/*!
	 * verilog qi grammar
	 */
	template <typename Iterator>
	struct nxp_verilog_parser : qi::grammar<Iterator,std::vector<ds_library::parse_netlist>(), ascii::space_type>
	{
		nxp_verilog_parser() : nxp_verilog_parser::base_type(start, "netlists")
		{
			namespace qi = boost::spirit::qi;
			namespace phoenix = boost::phoenix;
			namespace ascii = boost::spirit::ascii;
			using qi::int_;
			using qi::char_;
			using qi::lit;
			using qi::_1;
			using qi::_0;
			using qi::eps;
			using phoenix::at_c;
			using phoenix::push_back;
			using phoenix::ref;
			using boost::spirit::_val;

			start = *netlist[push_back(_val,_1)];

			netlist = "module"
					>> name[at_c<0>(_val)=_1]
					>> lit('(')
					>> ports[at_c<1>(_val)=_1]
					>> lit(')') >> lit(';')
					>>
					*(
						  ("input"  >> declaration			[push_back(at_c<2>(_val), _1)] % ','
						| ("output" >> declaration			[push_back(at_c<3>(_val), _1)] % ',')
						| ("inout"  >> declaration			[push_back(at_c<4>(_val), _1)] % ',')
						| ("wire"   >> declaration 			[push_back(at_c<5>(_val), _1)] % ',')
						| (assign							[push_back(at_c<6>(_val), _1)])
						| (!lit("endmodule") >>  instance 	[push_back(at_c<7>(_val), _1)])
						)
						>> lit(';')
					)
					>> "endmodule";

			name = -lit("\\") >> qi::char_("a-zA-Z_") >> *qi::char_("a-zA-Z_0-9[]/") ;
			b_constant = "1'b" >> qi::char_("0-1");
			ports =  name  % ',';
			aggregate =   lit('[') >> int_ >> lit(':') >> int_ >> lit(']') >> name;
			declaration = (aggregate | name ) [_val = _1];
			assign = "assign" >>  ( name ) >> lit('=') >> ( name );
			instance = implicit_i | explicit_i;
			implicit_i = name >> name >> lit('(') >> ports >> lit(')');
			explicit_i = name >> name >> lit('(') >> binding % ',' >> lit(')');
			binding = lit('.') >> name >> lit('(') >> ( name | b_constant) >> lit(')');

			BOOST_SPIRIT_DEBUG_NODE(name);
			BOOST_SPIRIT_DEBUG_NODE(b_constant);
			BOOST_SPIRIT_DEBUG_NODE(slice);
			BOOST_SPIRIT_DEBUG_NODE(ports);
			BOOST_SPIRIT_DEBUG_NODE(aggregate);
			BOOST_SPIRIT_DEBUG_NODE(declaration);
			BOOST_SPIRIT_DEBUG_NODE(assign);
			BOOST_SPIRIT_DEBUG_NODE(implicit_i);
			BOOST_SPIRIT_DEBUG_NODE(start);
			BOOST_SPIRIT_DEBUG_NODE(netlist);
			BOOST_SPIRIT_DEBUG_NODE(assign);
			BOOST_SPIRIT_DEBUG_NODE(implicit_i);
			BOOST_SPIRIT_DEBUG_NODE(explicit_i);
			BOOST_SPIRIT_DEBUG_NODE(instance);

		}

		qi::rule<Iterator, std::string()> name;
		qi::rule<Iterator, std::string(), ascii::space_type> slice, number, b_constant;
		qi::rule<Iterator, std::vector<std::string>(), ascii::space_type> ports;
		qi::rule<Iterator, std::pair<std::string,std::string>(), ascii::space_type> binding;
		qi::rule<Iterator, ds_library::parse_nl_aggregate(), ascii::space_type> aggregate;
		qi::rule<Iterator, ds_library::verilog_declaration(), ascii::space_type> declaration;
		qi::rule<Iterator, ds_library::parse_nl_assignment(), ascii::space_type> assign;
		qi::rule<Iterator, ds_library::parse_nl_implicit_instance(), ascii::space_type> implicit_i;
		qi::rule<Iterator, ds_library::parse_nl_explicit_instance(), ascii::space_type> explicit_i;
		qi::rule<Iterator, ds_library::verilog_instance(), ascii::space_type> instance;
		qi::rule<Iterator, ds_library::parse_netlist(), ascii::space_type> netlist;
		qi::rule<Iterator, std::vector<ds_library::parse_netlist>(), ascii::space_type> start;

	};

	/*!
	 * library qi grammar
	 */
	template <typename Iterator>
	struct lib_parser : qi::grammar<Iterator,parse_lib_node()>
	{

		lib_parser() : lib_parser::base_type(start)
		{
			namespace qi = boost::spirit::qi;
			namespace ascii = boost::spirit::ascii;
			using qi::int_;
			using qi::char_;
			using qi::lit;
			using boost::phoenix::at_c;
			using boost::spirit::_val;
			using qi::eps;

			start %=
					eps[at_c<6>(_val)=false] >>
					name >> -lit('*')[at_c<6>(_val)=true] >> ','
		          	>> name >> ','
		            >> int_ >> ','
		            >> mapping >> ','
		            >> int_ >> ','>>
		            mapping;

			name = qi::char_("a-zA-Z_") >> *qi::char_("a-zA-Z_0-9");
			binding = name >> '@' >> name;
			mapping = binding >> *(',' >> binding);
		}

		qi::rule<Iterator, std::string()> port, name;
		qi::rule<Iterator, std::pair<std::string,std::string>()> binding;
		qi::rule<Iterator, std::map<std::string,std::string>()> mapping;
		qi::rule<Iterator, parse_lib_node()> start;
	};

	/*!
	 * skipper grammar for library parser. Comments start with '#'
	 */
	template<typename Iterator>
	struct lib_skipper : public qi::grammar<Iterator> {
		lib_skipper() : lib_skipper::base_type(skip) {
			skip = boost::spirit::qi::ascii::space | ('#' >> *(qi::char_));
		}
		qi::rule<Iterator> skip;
	};

	/*!
	 * parses a library description. It reads in text between the iterators and fills up the gate and function definitions.
	 * @param first iterator to the beginning of the line
	 * @param last iterator to the end of the line
	 * @param gates gate mappings. Gate definitions are stored here
	 * @param prototypes definition of simulation primitives. Necessary for gate mappings
	 * @param functions function mappings. Function definitions are stored here
	 * @param inversion function inversion. Inversions are stored here
	 * @return true if line was successfully parsed
	 */
	template <typename Iterator>
	bool parse_library(Iterator first, Iterator last, gate_map_t& gates, lgn_map_t& prototypes, type_map_t types, function_map& functions, inversion_map& inversion, prototype_map_t& prototype_map){

		parse_lib_node n;
		lib_parser<Iterator> p;
		lib_skipper<Iterator> skipper;

		//parse library
		bool parse =  boost::spirit::qi::phrase_parse(first, last, p, skipper, n);
		if (parse){

			// line constains a valid gate definition
			// perform sanity checks
			std::size_t num_inputs = n.inputs;
			std::size_t num_outputs = n.outputs;
			if (num_inputs != n.i_mapping.size() || num_outputs != n.o_mapping.size())

				BOOST_THROW_EXCEPTION(ds_common::parse_error()
				<< ds_common::errmsg_info("Inconsistent # of inputs/outputs"));

			else {
				// find an equivalent simulation primitive
				using ds_structural::Gate;
				using std::string;
				lgn_map_t::iterator iterator = prototypes.find(n.node_name);
				if (iterator == prototypes.end()){

					BOOST_THROW_EXCEPTION(ds_common::parse_error()
					<< ds_common::errmsg_info("Unknown prototype: " + n.node_name));

				}
				// create new gate and setup ports and mappings
				Gate *g = new Gate();
				g->set_type(n.gate_name);
				for (fusion_map::const_iterator it = n.i_mapping.begin(); it!=n.i_mapping.end();it++){
					string pb_name = it->first;
					string lgn_name = it->second;
					ds_structural::PortBit *p = new ds_structural::PortBit(pb_name, g, ds_structural::DIR_IN);
					g->add_port(p);
					g->add_mapping(pb_name, lgn_name);
				}
				for (fusion_map::const_iterator it = n.o_mapping.begin(); it!=n.o_mapping.end();it++){
					string pb_name = it->first;
					string lgn_name = it->second;
					ds_structural::PortBit *p = new ds_structural::PortBit(pb_name, g, ds_structural::DIR_OUT);
					g->add_port(p);
					g->add_mapping(pb_name, lgn_name);
				}

				//match gate name with prototype name
				prototype_map[n.gate_name] = n.node_name;

				// register gate in the library
				gates[n.gate_name] = g;

				//register function type
				types[n.gate_name] = types[n.node_name];

				//take care of variable input funcions/gates
				if (n.flexible){

					// each function is defined in terms of a function and an inversion flag

					if (n.gate_name == "and"){
						functions[n.gate_name] = ds_lg::f_and2;
						inversion[n.gate_name] = false;
					}
					if (n.gate_name == "or"){
						functions[n.gate_name] = ds_lg::f_or2;
						inversion[n.gate_name] = false;
					}
					if (n.gate_name == "xor"){
						functions[n.gate_name] = ds_lg::f_xor2;
						inversion[n.gate_name] = false;
					}
					if (n.gate_name == "nand"){
						functions[n.gate_name] = ds_lg::f_and2;
						inversion[n.gate_name] = true;
					}
					if (n.gate_name == "nor"){
						functions[n.gate_name] = ds_lg::f_or2;
						inversion[n.gate_name] = true;
					}
					if (n.gate_name == "xnor"){
						functions[n.gate_name] = ds_lg::f_xor2;
						inversion[n.gate_name] = true;
					}
				}

			}
		}
		return parse;
	}

	/*!
	 * reads in a circuit description and produces a netlist.
	 * The circuit description is first parsed into an intermediate description using boost:qi.
	 * Any design dependencies are automotically resolved
	 * @param file path the file containing the circuit description to elaborate
	 * @param toplevel name of the netlist to elaborate
	 * @param workspace design information for gate resolution and dependencies
	 * @return the elaborated netlist
	 *
	 */
	ds_structural::NetList* import(const std::string& file, const std::string& toplevel, ds_workspace::Workspace* workspace);

	/*!
	 * tranforms an intermediate netlist repreentation into a netlist
	 * @param nl intermediate netlist representation
	 * @param workspace design information for gate resolution and dependencies
	 * @return the elaborated netlist
	 */
	ds_structural::NetList* convert(const ds_library::parse_netlist& nl, ds_workspace::Workspace *workspace);

	/*!
	 * aggregate visitor (recommended strategy by qi)
	 */
	struct aggregate_visitor : boost::static_visitor<void> {

		bool create_port; 	//!< true if a port is also created

		/*!
		 * initializes a few parameters used for every created aggregate (port or signal)
		 * @param nl owner netlist
		 * @param pt port type created by this visitor
		 */
		aggregate_visitor(ds_structural::NetList *nl, const ds_structural::PortType& pt):create_port(true),
				netlist(nl), type(pt){}

		/*!
		 * sets the port type used for every aggregate created by this visitor
		 */
		void set_port_type(const ds_structural::PortType& t){
			type = t;
		}

		/*!
		 * creates a single port or signal in the netlist
		 * @param name name of port or signal to create
		 */
		void operator()(const std::string& name){
	           visit(name);
		}

		/*!
		 * creates a range of ports or signals in the netlist
		 * @param s range of ports or signals to create
		 */
		void operator()(const ds_library::parse_nl_aggregate& s){
			int low  = s.left > s.right ? s.right : s.left;
			int high = s.left < s.right ? s.right : s.left;
			for (int i=low;i<=high;i++){
				std::stringstream number;
				number << i;
				std::string name = s.name + "[" + number.str() + "]";
				visit(name);
			}
		}


	private:

		ds_structural::NetList *netlist;	//!< owner netlist
		ds_structural::PortType type;		//!< port type to be created

		/*!
		 * creates a signal with the specified name in the netlist.
		 * When the create_port flag is active a port is also created with the same name
		 */
		void visit(const std::string& name){
			ds_structural::Signal *s = new ds_structural::Signal(name);
			netlist->add_signal(s);
			if (create_port) {
				ds_structural::PortBit *pb = new ds_structural::PortBit(name, type);
				pb->set_gate(netlist);
				netlist->add_port(pb);
				pb->set_signal(s);
				s->add_port(pb);
			}
		}
	};

	/*!
	 * instance visitor (recommended strategy by qi)
	 */
	struct instance_visitor : boost::static_visitor<void> {

		ds_structural::NetList *netlist;	//!< owner netlist (all instances are added here)
		ds_workspace::Workspace *wp;		//!< design info for resolution

		/*!
		 * sets up the netlist where instances are stored and keeps a reference to the workspace for gate and dependency resolution
		 * @param nl owner netlist
		 * @param w workspace for resolution
		 */
		instance_visitor(ds_structural::NetList *n, ds_workspace::Workspace *w):netlist(n),wp(w) {}

		/*!
		 * creates and connects a gate instantiated explicitly
		 * @param instance explicit intermediate gate representation
		 */
		void operator()(const ds_library::parse_nl_explicit_instance& instance);

		/*!
		* creates and connects a gate instantiated implicitly
		* @param instance implicit intermediate gate representation
		*/
		void operator()(const ds_library::parse_nl_implicit_instance& implicit);
	};

	/*!
	 * keeps track of design dependencies. Queries the workspace to resolve all instances
	 */
	struct dependency_visitor : boost::static_visitor<void> {

		/*!
		 * all dependencies (<design name, # of ports> depends on <design_name, # of ports>)
		 */
		std::multimap<std::pair<std::string, int> , std::pair<std::string, int> > dep_map;
		std::set<std::pair<std::string, int> >dependencies;  //!< dependencies of the current design
		std::pair<std::string, int> design; //!< current design (name, # of ports)
		ds_workspace::Workspace *wp; //!< workspace for gate and netlist resolution

		/*!
		 * keeps a pointer to the workspace
		 * @param worskpace
		 */
		dependency_visitor(ds_workspace::Workspace *w):wp(w) {}

		/*!
		 * check the dependencies of an explicit instance. Unimplemented so far
		 * @param instance implicit instance to check
		 */
		void operator()(const ds_library::parse_nl_explicit_instance& instance);

		/*!
		 * check the dependencies of an implicit instance
		 * @param instance explicit instance to check
		 */
		void operator()(const ds_library::parse_nl_implicit_instance& implicit);
	};

	bool parse_verilog(const std::string& name, std::vector<parse_netlist>& netlists);

	Library* load_default_lib();

}



#endif /* LIBRARY_H_ */
