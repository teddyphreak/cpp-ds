/*
 * ds_trans.cpp
 *
 *  Created on: 24.01.2014
 *      Author: cookao
 */

#include "ds_trans.h"
#include "ds_structural.h"
#include <boost/log/trivial.hpp>

ds_structural::Gate* ds_trans::get_scan_ff(const ds_trans::ScanFFInfo& info){
	using ds_structural::PortBit;
	using ds_structural::Gate;
	Gate *g = new Gate();
	g->set_type(info.name);

	PortBit *d = new PortBit(info.d_port, g, ds_structural::DIR_IN);
	g->add_port(d);
	d->set_gate(g);
	g->add_mapping(info.d_port, "d");

	PortBit *clk = new PortBit(info.clk, g, ds_structural::DIR_IN);
	g->add_port(clk);
	clk->set_gate(g);
	g->add_mapping(info.clk, "clk");

	PortBit *si = new PortBit(info.si_port, g, ds_structural::DIR_IN);
	g->add_port(si);
	si->set_gate(g);
	g->add_mapping(info.si_port, "si");

	PortBit *se = new PortBit(info.se_port, g, ds_structural::DIR_IN);
	g->add_port(se);
	se->set_gate(g);
	g->add_mapping(info.se_port, "se");

	if (info.has_reset){
		PortBit *rs = new PortBit(info.reset_port, g, ds_structural::DIR_IN);
		g->add_port(rs);
		rs->set_gate(g);
		g->add_mapping(info.reset_port, "rs");
	}

	if (info.has_preset){
		PortBit *ps = new PortBit(info.preset_port, g, ds_structural::DIR_IN);
		g->add_port(ps);
		ps->set_gate(g);
		g->add_mapping(info.preset_port, "ps");
	}

	PortBit *q = new PortBit(info.q_port, g, ds_structural::DIR_OUT);
	g->add_port(q);
	q->set_gate(g);
	g->add_mapping(info.q_port, "q");

	if (info.has_qn_port){
		PortBit *qn = new PortBit(info.qn_port, g, ds_structural::DIR_OUT);
		g->add_port(qn);
		g->add_mapping(info.qn_port, "qn");
	}

	return g;
}

ds_structural::NetList* ds_trans::insert_scan_chains(ds_structural::NetList *nl, const ds_structural::CombinationalScanMap *map, const ds_trans::ScanProperties& scan_properties, const ds_trans::ScanFFInfo& ff_info){

	ds_structural::NetList *seq = nl->clone();

	if (map->get_input_chains() != map->get_output_chains()){
		BOOST_LOG_TRIVIAL(error) << "Inputs and Outputs in the scan chain map do not match";
		delete seq;
		return 0;
	}

	int chain_nr = map->get_output_chains();
	std::vector<std::vector<ds_structural::Gate*> > ffs;
	for (int i=0;i<chain_nr;i++){
		std::vector<ds_structural::Gate*> ffs_in_chain;
		if (map->get_output_chain_length(i) != map->get_input_chain_length(i)){
			BOOST_LOG_TRIVIAL(error) << "Size of input and output scan chain " << i << " do not match";
			delete seq;
			return 0;
		}

		int chain_length = map->get_output_chain_length(i);

		for (int j=0;j<chain_length;j++){

			std::string output_name = map->get_input_signal(i,j);
			std::string input_name = map->get_output_signal(i,j);

			ds_structural::PortBit *input_port = seq->find_port_by_name(input_name);
			ds_structural::PortBit *output_port = seq->find_port_by_name(output_name);

			ds_structural::Gate *ff = get_scan_ff(ff_info);
			ff->set_instance_name(scan_properties.ff_name + "_" + std::to_string(i) + std::to_string(j));
			seq->add_gate(ff);
			ffs_in_chain.push_back(ff);

			if (input_port != 0){
				seq->remove_port(input_port);
				ds_structural::Signal *q_signal = seq->find_signal(input_name);
				ds_structural::PortBit *q_port = ff->find_port_by_name(ff_info.q_port);
				q_signal->add_port(q_port);
			} else {
				BOOST_LOG_TRIVIAL(warning) << "Input port not found: " << input_name << " (" << i << ";" << j << ")";
				ds_structural::Signal *q_signal = new ds_structural::Signal(input_name);
				seq->add_signal(q_signal);
				ds_structural::PortBit *q_port = ff->find_port_by_name(ff_info.q_port);
				q_signal->add_port(q_port);
			}
			if (output_port != 0){
				seq->remove_port(output_port);
				ds_structural::Signal *d_signal = seq->find_signal(output_name);
				ds_structural::PortBit *d_port = ff->find_port_by_name(ff_info.d_port);
				d_signal->add_port(d_port);
			} else {
				BOOST_LOG_TRIVIAL(warning) << "Output port not found: " << output_name << " (" << i << ";" << j << ")";
				ds_structural::Signal *d_signal = new ds_structural::Signal(output_name);
				seq->add_signal(d_signal);
				ds_structural::PortBit *d_port = ff->find_port_by_name(ff_info.d_port);
				d_signal->add_port(d_port);
			}

			if (j!=0){
				ds_structural::Gate *previous = ffs_in_chain[j-1];
				ds_structural::PortBit *previous_q_port = previous->find_port_by_name(ff_info.q_port);
				ds_structural::Signal *si_signal = previous_q_port->get_signal();
				ds_structural::PortBit *si_port = ff->find_port_by_name(ff_info.si_port);
				si_signal->add_port(si_port);
			}
		}

		std::string si_name = scan_properties.si_name + std::to_string(i);
		ds_structural::PortBit *si_port = new ds_structural::PortBit(si_name, ds_structural::DIR_IN);
		ds_structural::Signal *si_signal = new ds_structural::Signal(si_name);
		si_signal->add_port(si_port);
		seq->add_port(si_port);
		seq->add_signal(si_signal);
		si_port->set_gate(seq);

		ds_structural::Gate *first_ff = ffs_in_chain[0];
		ds_structural::PortBit *first_sp = first_ff->find_port_by_name(ff_info.si_port);
		si_signal->add_port(first_sp);

		std::string so_name = scan_properties.so_name + std::to_string(i);
		ds_structural::PortBit *so_port = new ds_structural::PortBit(so_name, ds_structural::DIR_OUT);
		ds_structural::Gate *last_ff = ffs_in_chain[chain_length-1];
		ds_structural::PortBit *last_sp = last_ff->find_port_by_name(ff_info.q_port);
		last_sp->get_signal()->add_port(so_port);
		so_port->set_gate(seq);

		ffs.push_back(ffs_in_chain);
	}

	if (scan_properties.has_reset){
		insert_top_level_scan(scan_properties.reset_name, ff_info.reset_port, seq, ffs);
	}

	if (scan_properties.has_preset){
		insert_top_level_scan(scan_properties.preset_name, ff_info.preset_port, seq, ffs);
	}

	insert_top_level_scan(scan_properties.clk_name, ff_info.clk, seq, ffs);
	insert_top_level_scan(scan_properties.se_name, ff_info.se_port, seq, ffs);

	return seq;
}

void ds_trans::insert_top_level_scan(std::string top_level, std::string scan_port, ds_structural::NetList *nl, std::vector<std::vector<ds_structural::Gate*> >& ffs){
	ds_structural::PortBit *port = new ds_structural::PortBit(top_level, ds_structural::DIR_IN);
	ds_structural::Signal *signal = new ds_structural::Signal(top_level);
	nl->add_port(port);
	port->set_gate(nl);
	nl->add_signal(signal);
	signal->add_port(port);

	for (std::size_t i=0;i<ffs.size();i++){
		for (std::size_t j=0;j<ffs[i].size();j++){
			ds_structural::Gate *g = ffs[i][j];
			ds_structural::PortBit *p = g->find_port_by_name(scan_port);
			signal->add_port(p);
		}
	}
}

