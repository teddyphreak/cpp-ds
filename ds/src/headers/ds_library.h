/*
 * library.h
 *
 *  Created on: Jul 5, 2012
 *      Author: cookao
 */

#ifndef LIBRARY_H_
#define LIBRARY_H_

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

namespace ds_library {

	struct file_read_error: virtual boost::exception, virtual std::exception { };
	struct parse_error: virtual boost::exception, virtual std::exception { };

	namespace qi = boost::spirit::qi;
	typedef std::map<std::string, ds_lg::bi_eval> function_map;
	typedef std::map<std::string, std::string> fusion_map;
	typedef std::map<std::string, ds_structural::Gate*> gate_map_t;
	typedef std::map<std::string, ds_lg::LGNode*> lgn_map_t;


	class Library {

	friend class LibraryFactory;
	public:
	ds_structural::Gate* getGate(const std::string& name, const std::size_t ports) const {
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
				ds_lg::bi_eval func = it->second;
				g = new ds_structural::Gate();
				g->set_type(name);
				g->add_port(new ds_structural::PortBit("Z", ds_structural::DIR_OUT));
				g->add_mapping("Z", "z");
				char lgn_port = 'a';
				for (std::size_t i='A';i<'A'+ports-1;i++){
					std::string port_name;
					port_name += (char)i;
					g->add_port(new ds_structural::PortBit(port_name, ds_structural::DIR_IN));
					std::string lgn_port_name;
					lgn_port_name += lgn_port;
					g->add_mapping(port_name, lgn_port_name);
					lgn_port++;
				}
				ds_lg::LGNode *n = new ds_lg::LGNodeArr(name, ports, func);
				g->set_lgn(n);
			}
		}
		return g;
	}

	virtual ~Library(){close();}

	protected:
		gate_map_t gate_map;
		lgn_map_t prototypes;
		function_map functions;
		virtual void load(const std::string &lib_name);
		virtual void load_nodes();
		void close();

	private:
		Library(const std::string& lib_path){
			load_nodes();
			load(lib_path);
		}
	};

	typedef std::map<std::string,Library*> library_map_t;

	class LibraryFactory {

	private:
		static LibraryFactory* instance;
		library_map_t map;
		LibraryFactory(){}
		~LibraryFactory(){instance = 0;}

	public:
		static LibraryFactory* getInstance(){
			if (instance == 0)
				instance = new LibraryFactory();
			return instance;
		}

		Library* loadLibrary(const std::string& name="default_lib");

		void remove_library(const std::string name = "default_lib"){
			library_map_t::iterator it = map.find(name);
			if (it != map.end()){
				delete (it->second);
				map.erase(it);
			}
		}
	};

	ds_structural::NetList* import(const std::string& file, const std::string& toplevel, const Library* lib);

	template<typename Iterator>
	ds_structural::NetList* import_verilog(Iterator begin, Iterator end, const std::string& toplevel, const Library* lib);

	struct parse_nl_aggregate
	{
		std::string name;
		int left;
		int right;
	};

	struct parse_nl_assignment
	{
		std::string lhs;
		std::string rhs;
	};

	struct parse_nl_implicit_instance
	{
		std::string type;
		std::string name;
		std::vector<std::string> ports;
	};

	typedef boost::variant<ds_library::parse_nl_aggregate, std::string> verilog_declaration;

	struct parse_netlist
	{
		std::string nl_name;
		std::vector<std::string> ports;
		std::vector<verilog_declaration> inputs;
		std::vector<verilog_declaration> outputs;
		std::vector<verilog_declaration> signals;
		std::vector<parse_nl_assignment> assignments;
		std::vector<parse_nl_implicit_instance> instances;
	};

	struct parse_lib_node
	{
		std::string gate_name;
		std::string node_name;
		int outputs;
		std::map<std::string, std::string> o_mapping;
		int inputs;
		std::map<std::string, std::string> i_mapping;
		bool flexible;
	};
}

BOOST_FUSION_ADAPT_STRUCT(
    ds_library::parse_lib_node,
    (std::string, gate_name)
    (std::string, node_name)
    (int, inputs)
    (ds_library::fusion_map, i_mapping)
    (int, outputs)
    (ds_library::fusion_map, o_mapping)
    (bool, flexible)
)

BOOST_FUSION_ADAPT_STRUCT(
    ds_library::parse_nl_aggregate,
    (std::string, name)
    (int, left)
    (int, right)
)

BOOST_FUSION_ADAPT_STRUCT(
    ds_library::parse_nl_implicit_instance,
    (std::string, type)
    (std::string, name)
    (std::vector<std::string>, ports)
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
	(std::vector<ds_library::verilog_declaration>, signals)
	(std::vector<ds_library::parse_nl_assignment>, assignments)
	(std::vector<ds_library::parse_nl_implicit_instance>, instances)
)

namespace ds_library {

	namespace ascii = boost::spirit::ascii;

	template <typename Iterator>
	//struct nxp_verilog_parser : qi::grammar<Iterator, ds_library::parse_netlist(), ascii::space_type >
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
			using qi::debug;
			using qi::on_error;
			using qi::fail;
			using qi::lexeme;
			using qi::omit;
			using qi::no_skip;
			using phoenix::construct;
			using phoenix::val;
			using phoenix::at_c;
			using phoenix::push_back;
			using phoenix::ref;
			using boost::spirit::_val;
			using boost::spirit::repository::distinct;

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
						| ("wire"   >> declaration 			[push_back(at_c<4>(_val), _1)] % ',')
						| (assign							[push_back(at_c<5>(_val), _1)])
						| (!lit("endmodule") >>  implicit 	[push_back(at_c<6>(_val), _1)])
						)
						>> lit(';')
					)
					>> "endmodule";

			name = qi::char_("a-zA-Z_") >> *qi::char_("a-zA-Z_0-9");
			ports = name % ',';
			aggregate = name >> lit('[') >> int_ >> lit(':') >> int_>> lit(']');
			declaration = (aggregate | name ) [_val = _1];
			assign = "assign" >> name >> lit('=') >> name;
			implicit = name_ns >> name >> lit('(') >> ports >> lit(')');
			name_ns = no_skip[omit[*ascii::space] >> (qi::char_("a-zA-Z_") >> *qi::char_("a-zA-Z_0-9")) >> omit[*ascii::space]];

			name.name("name");
			ports.name("ports");
			aggregate.name("aggregate");
			declaration.name("declaration");
			assign.name("name");
			implicit.name("implicit");
			start.name("start");
			name_ns.name("name_ns");
			netlist.name("netlist");
		}

		qi::rule<Iterator, std::string(), ascii::space_type> name, name_ns;
		qi::rule<Iterator, std::vector<std::string>(), ascii::space_type> ports;
		qi::rule<Iterator, ds_library::parse_nl_aggregate(), ascii::space_type> aggregate;
		qi::rule<Iterator, ds_library::verilog_declaration(), ascii::space_type> declaration;
		qi::rule<Iterator, ds_library::parse_nl_assignment(), ascii::space_type> assign;
		qi::rule<Iterator, ds_library::parse_nl_implicit_instance(), ascii::space_type> implicit;
		qi::rule<Iterator, ds_library::parse_netlist(), ascii::space_type> netlist;
		qi::rule<Iterator, std::vector<ds_library::parse_netlist>(), ascii::space_type> start;

	};

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

	template<typename Iterator>
	struct lib_skipper : public qi::grammar<Iterator> {
		lib_skipper() : lib_skipper::base_type(skip) {
			skip = boost::spirit::qi::ascii::space | ('#' >> *(qi::char_));
		}
		qi::rule<Iterator> skip;
	};

	template <typename Iterator>
	bool parse_library(Iterator first, Iterator last, gate_map_t& gates, lgn_map_t& prototypes, function_map& functions){

		parse_lib_node n;
		lib_parser<Iterator> p;
		lib_skipper<Iterator> skipper;
		bool parse =  boost::spirit::qi::phrase_parse(first, last, p, skipper, n);
		if (parse){
			std::size_t num_inputs = n.inputs;
			std::size_t num_outputs = n.outputs;
			if (num_inputs != n.i_mapping.size() || num_outputs != n.o_mapping.size())

				BOOST_THROW_EXCEPTION(ds_library::parse_error()
				<< ds_common::errmsg_info("Inconsistent # of inputs/outputs"));

			else {
				using ds_structural::Gate;
				using std::string;
				lgn_map_t::iterator iterator = prototypes.find(n.node_name);
				if (iterator == prototypes.end()){

					BOOST_THROW_EXCEPTION(ds_library::parse_error()
					<< ds_common::errmsg_info("Unknown prototype: " + n.node_name));

				}
				Gate *g = new Gate();
				g->set_type(n.gate_name);
				ds_lg::LGNode *proto = iterator->second;
				g->set_lgn(proto->clone());
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
				gates[n.gate_name] = g;
				if (n.flexible){
					if (n.gate_name == "and"){
						functions[n.gate_name] = ds_lg::f_and2;
					}
					if (n.gate_name == "or"){
						functions[n.gate_name] = ds_lg::f_or2;
					}
					if (n.gate_name == "xor"){
						functions[n.gate_name] = ds_lg::f_xor2;
					}
					if (n.gate_name == "nand"){
						functions[n.gate_name] = ds_lg::f_nand2;
					}
					if (n.gate_name == "nor"){
						functions[n.gate_name] = ds_lg::f_nor2;
					}
					if (n.gate_name == "xnor"){
						functions[n.gate_name] = ds_lg::f_xnor2;
					}
				}

			}
		}
		return parse;
	}

	enum LogicGate {
		AND,
		OR,
		NOT,
		NAND,
		NOR,
		BUF,
		XOR
	};

	ds_structural::NetList* convert(ds_library::parse_netlist nl, ds_library::Library);

	template<typename Container>
	struct aggregate_visitor : boost::static_visitor<void> {

		Container container;

		aggregate_visitor(const Container& c){container = c;}

		void operator()(std::string s){
	           container.push_back(s);
		}

		void operator()(ds_library::parse_nl_aggregate s){
			int low  = s.left > s.right ? s.right : s.left;
			int high = s.left < s.right ? s.right : s.left;
			for (int i=low;i<=high;i++){
				std::stringstream number;
				number << i;
				container.push_back(s.name + "(" + number.str() + ")");
			}
		}


		std::vector<std::string>::iterator begin(){
			return container.begin();
		}

		std::vector<std::string>::iterator  end(){
			return container.end();
		}

		void clear(){
			container.clear();
		}
	};

}

#endif /* LIBRARY_H_ */
