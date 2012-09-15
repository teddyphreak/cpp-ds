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
#include <fstream>
#include <map>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>
#include <boost/spirit/include/phoenix_fusion.hpp>
#include <boost/config/warning_disable.hpp>
#include <boost/fusion/include/std_pair.hpp>
#include <boost/config/warning_disable.hpp>
#include <boost/spirit/include/phoenix_object.hpp>
#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/fusion/include/io.hpp>

namespace ds_library {

	namespace qi = boost::spirit::qi;
	typedef std::map<std::string, std::string> fusion_map;
	typedef std::map<std::string,ds_structural::Gate*> gate_map_t;
	typedef std::map<std::string,ds_lg::LGNode*> lgn_map_t;

	class Library {

		friend class LibraryFactory;
		ds_structural::Gate* getGate(const std::string& name){
			gate_map_t::const_iterator it = gate_map.find(name);
			if (it != gate_map.end()){
				return it->second;
			}
		}
		virtual ~Library(){close();}

	protected:
		gate_map_t gate_map;
		virtual void load(const std::string &lib_name);
		virtual void load_nodes();
		lgn_map_t prototypes;
		void close();

	private:
		Library(){
			load_nodes();
			load("default");
		}
		Library(const std::string& lib_name){load(lib_name);}
	};

	typedef std::map<std::string,Library*> library_map_t;

	class LibraryFactory {
	private:
		static LibraryFactory* instance;
		library_map_t map;
		LibraryFactory(){}
	public:
		static LibraryFactory* getInstance(){
			if (instance ==0)
				instance = new LibraryFactory();
			return instance;
		}

		Library* loadLibrary(const std::string name="default"){
			Library *lib = 0;
			library_map_t::const_iterator it = map.find(name);
			if (it != map.end())
				lib = it->second;
			else if (name == "default"){
				lib = new Library();
				map[name] = lib;
			}
			return lib;
		}

		void remove_library(const std::string name){
			library_map_t::iterator it = map.find(name);
			if (it != map.end()){
				delete (it->second);
				map.erase(it);
			}
		}
	};

	ds_structural::NetList* import(const std::string& file, const std::string& toplevel);

	template<typename Iterator>
	ds_structural::NetList* import_verilog(Iterator begin, Iterator end, const std::string& toplevel) {
		return 0;
	}

	template <typename Iterator>
	struct verilog_parser : qi::grammar<Iterator, ds_structural::NetList(), qi::space_type>
	{


	};

	struct parse_lib_node
	{
		std::string gate_name;
		std::string node_name;
		int outputs;
		std::map<std::string, std::string> o_mapping;
		int inputs;
		std::map<std::string, std::string> i_mapping;
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

			start %=
					name >> ','
		          	>> name >> ','
		            >> int_ >> ','
		            >> mapping
		            >> int_ >> ','
		            >> mapping
		            ;

			port = qi::char_("a-zA-Z_") >> *qi::char_("a-zA-Z_0-9") >> -('(' >> +qi::char_("0-9")  >>')');
			name = qi::char_("a-zA-Z_") >> *qi::char_("a-zA-Z_0-9");
			binding = name >> '@' >> name;
			mapping = binding >> *(',' >> binding);
		}

		qi::rule<Iterator, std::string()> port, name;
		qi::rule<Iterator, std::pair<std::string,std::string>()> binding;
		qi::rule<Iterator, std::map<std::string,std::string>()> mapping;
		qi::rule<Iterator, parse_lib_node()> start;
	};

	template <typename Iterator>
	bool parse_library(Iterator first, Iterator last, gate_map_t *gates, lgn_map_t& prototypes){

		parse_lib_node n;
		lib_parser<Iterator> p;
		bool parse = qi::parse(first, last, p, n);
		if (parse){
			std::size_t num_inputs = n.inputs;
			std::size_t num_outputs = n.outputs;
			if (num_inputs != n.i_mapping.size() || num_outputs != n.o_mapping.size())
				parse = false;
			else {
				using ds_structural::Gate;
				using std::string;
				Gate *g = new Gate();
				g->set_type(n.gate_name);
				ds_lg::LGNode *proto = prototypes[n.node_name];
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
				(*gates)[n.gate_name] = g;

			}
		}
		return true;
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


	struct file_read_error: virtual boost::exception, virtual std::exception { };
	struct parse_error: virtual boost::exception, virtual std::exception { };

}

BOOST_FUSION_ADAPT_STRUCT(
    ds_library::parse_lib_node,
    (std::string, gate_name)
    (std::string, node_name)
    (int, inputs)
    (ds_library::fusion_map, i_mapping)
    (int, outputs)
    (ds_library::fusion_map, o_mapping)
)
#endif /* LIBRARY_H_ */
