/*
 * trans_test.cpp
 *
 *  Created on: 27.01.2014
 *      Author: cookao
 */
#include <boost/test/unit_test.hpp>
#include <boost/log/trivial.hpp>
#include "trans_test.h"
#include "ds_pattern.h"
#include "ds_library.h"
#include "ds_structural.h"
#include "ds_workspace.h"
#include "ds_trans.h"

void trans_test::convert_sff_netlist(const std::string& name) {
	const char* d = getenv("DS");
	if (!d){
		BOOST_LOG_TRIVIAL(warning) << "Environmental variable DS not set";
	}
	std::string path = d?d:"";
	std::string d_name = path + "/files/" + name + ".v.dsn";
	std::string scan_map_file = path + "/files/" + name + ".chain_info";
	std::string dump_file = path + "/files/" + name +"_sff"+ ".v";


	ds_workspace::Workspace *wp = ds_workspace::Workspace::get_workspace();
	BOOST_LOG_TRIVIAL(info) << "Reading netlist " << d_name;
	ds_structural::NetList* nl = ds_structural::load_netlist(d_name, wp);
	BOOST_LOG_TRIVIAL(info) << "Reading scan map " << scan_map_file;
	ds_structural::CombinationalScanMap *sm = ds_structural::get_combinational_scan_map(scan_map_file);

	ds_trans::ScanFFInfo info;
	info.name = "FD2S";
	info.d_port = "D";
	info.se_port = "TE";
	info.si_port = "TI";
	info.clk = "CP";
	info.q_port = "Q";
	info.has_qn_port = false;
	info.has_reset = true;
	info.reset_port = "CD";
	info.has_preset = false;

	ds_trans::ScanProperties properties;
	properties.has_reset = true;
	properties.reset_name = "reset_n";
	properties.has_preset = false;
	properties.clk_name = "clk";
	properties.se_name = "se";
	properties.ff_name = "FF";
	properties.si_name = "si";
	properties.so_name = "so";

	BOOST_LOG_TRIVIAL(info) << "Adding scan chains... " << scan_map_file;
	ds_structural::NetList* scan_nl = ds_trans::insert_scan_chains(nl, sm, properties, info);
	if (scan_nl !=0){
		BOOST_CHECK(scan_nl->check_netlist());
		BOOST_LOG_TRIVIAL(info) << "Dumping netlist... " << dump_file;
		std::ofstream verilog(dump_file);
		scan_nl->dump_verilog_implicit(verilog);
		verilog.close();
		BOOST_LOG_TRIVIAL(info) << "done.";
		delete scan_nl;
	}
	delete nl;
}
