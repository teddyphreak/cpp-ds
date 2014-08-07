/*
 * ds_faults.cpp
 *
 *  Created on: 31.10.2013
 *      Author: cookao
 */
#include <iostream>
#include <fstream>
#include "ds_faults.h"
#include "ds_library.h"
#include "ds_workspace.h"
#include "ds_structural.h"
#include "ds_common.h"

#include <boost/log/trivial.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>


using ds_faults::SAFaultDescriptor;

void ds_faults::get_fault_classes(ds_structural::Gate* g, std::vector<std::list<SAFaultDescriptor*>* >& fault_classes){
	ds_workspace::Workspace *workspace = ds_workspace::Workspace::get_workspace();
	std::string gate_type = g->get_type();
	//query logic function from workspace
	ds_library::LogicFunction f = workspace->get_function(gate_type);
	//generated equivalent faults according to the gate's controlling and inverting values
	switch(f){
	case ds_library::BUF:
		ds_faults::get_gate_faults(g, ds_common::BIT_0, ds_common::BIT_0, fault_classes);
		break;
	case ds_library::NOT:
		ds_faults::get_gate_faults(g, ds_common::BIT_0, ds_common::BIT_1, fault_classes);
		break;
	case ds_library::AND:
		ds_faults::get_gate_faults(g, ds_common::BIT_0, ds_common::BIT_0, fault_classes);
		break;
	case ds_library::OR:
		ds_faults::get_gate_faults(g, ds_common::BIT_1, ds_common::BIT_0, fault_classes);
		break;
	case ds_library::NAND:
		ds_faults::get_gate_faults(g, ds_common::BIT_0, ds_common::BIT_1, fault_classes);
		break;
	case ds_library::NOR:
		ds_faults::get_gate_faults(g, ds_common::BIT_1, ds_common::BIT_1, fault_classes);
		break;
	case ds_library::XOR:
	case ds_library::XNOR:
	default:
		//generate all outputs faults if gate type is not found
		BOOST_LOG_TRIVIAL(trace) << "Generating all port faults for gate: " << gate_type << " " << f;
		const ds_structural::port_container *inputs = g->get_inputs();
		const ds_structural::port_container *outputs = g->get_outputs();
		ds_faults::get_port_faults(inputs->begin(), inputs->end(), fault_classes);
		ds_faults::get_port_faults(outputs->begin(), outputs->end(), fault_classes);
		break;
	}
}

void ds_faults::get_gate_faults(ds_structural::Gate* g, ds_common::Value c, ds_common::Value i, std::vector<std::list<SAFaultDescriptor*>* >& fault_classes){

	using ds_structural::PortBit;
	// inverted controlling value
	ds_common::Value nc = (c == ds_common::BIT_0) ? ds_common::BIT_1: ds_common::BIT_0;
	//controlling value xor inverting value
	ds_common::Value ci = (c == i) ? ds_common::BIT_0 : ds_common::BIT_1;
	//controlling value xnor inverting value
	ds_common::Value nci = (c != i) ? ds_common::BIT_0 : ds_common::BIT_1;

	//get gate's output port
	PortBit *output_port = *(g->get_outputs()->begin());

	const ds_structural::port_container *ports = g->get_inputs();
	if (ports->size()==1){

		//single input gate (buf/not)

		//get gate's input port
		PortBit *input_port = *(g->get_inputs()->begin());

		//first equivalent class
		std::list<SAFaultDescriptor*> *f1_class = new std::list<SAFaultDescriptor*>();
		SAFaultDescriptor *fic = new SAFaultDescriptor(g->get_instance_name(), output_port->get_instance_name(), c);
		SAFaultDescriptor *foci = new SAFaultDescriptor(g->get_instance_name(), input_port->get_instance_name(), ci);
		f1_class->push_back(fic);
		f1_class->push_back(foci);
		fault_classes.push_back(f1_class);

		//second equivalent class
		std::list<SAFaultDescriptor*> *f2_class = new std::list<SAFaultDescriptor*>();
		SAFaultDescriptor *finc = new SAFaultDescriptor(g->get_instance_name(), output_port->get_instance_name(), nc);
		SAFaultDescriptor *fonci = new SAFaultDescriptor(g->get_instance_name(), input_port->get_instance_name(), nci);
		f2_class->push_back(finc);
		f2_class->push_back(fonci);
		fault_classes.push_back(f2_class);

	} else {
		// generate one stuck-at-not-c at each input
		for (auto it = ports->begin();it!=ports->end();it++){
			PortBit *pb = *it;
			SAFaultDescriptor *f = new SAFaultDescriptor(g->get_instance_name(), pb->get_instance_name(), nc);
			std::list<SAFaultDescriptor*> *f_class = new std::list<SAFaultDescriptor*>();
			f_class->push_back(f);
			fault_classes.push_back(f_class);
		}
		// generate one stuck-at-c-xnor-i at output
		SAFaultDescriptor *f = new SAFaultDescriptor(g->get_instance_name(), output_port->get_instance_name(), nci);
		std::list<SAFaultDescriptor*> *f_class = new std::list<SAFaultDescriptor*>();
		f_class->push_back(f);
		fault_classes.push_back(f_class);

		//generate equivalence class
		//generate one stuck-at-c-xor-i at output
		SAFaultDescriptor *fo = new SAFaultDescriptor(g->get_instance_name(), output_port->get_instance_name(), ci);
		std::list<SAFaultDescriptor*> *equivalent = new std::list<SAFaultDescriptor*>();;
		equivalent->push_back(fo);
		for (auto it = ports->begin();it!=ports->end();it++){
			PortBit *pb = *it;
			//generate one stuck-at-c at each input port
			SAFaultDescriptor *f = new SAFaultDescriptor(g->get_instance_name(), pb->get_instance_name(), c);
			equivalent->push_back(f);
		}
		fault_classes.push_back(equivalent);
	}
}

void ds_faults::get_port_faults(ds_structural::port_container::const_iterator begin, ds_structural::port_container::const_iterator end, std::vector<std::list<SAFaultDescriptor*>* >& fault_classes){
	// no equivalent faults
	for (auto it = begin;it!=end;it++){
		ds_structural::PortBit *pb = *it;

		// generate stuck-at-0 at each port
		SAFaultDescriptor *f0 = new SAFaultDescriptor(pb->get_gate()->get_instance_name(), pb->get_instance_name(), ds_common::BIT_0);
		std::list<SAFaultDescriptor*> *f0_class = new std::list<SAFaultDescriptor*>();
		f0_class->push_back(f0);
		fault_classes.push_back(f0_class);

		// generate stuck-at-1 at each port
		SAFaultDescriptor *f1 = new SAFaultDescriptor(pb->get_gate()->get_instance_name(), pb->get_instance_name(), ds_common::BIT_1);
		std::list<SAFaultDescriptor*> *f1_class = new std::list<SAFaultDescriptor*>();
		f1_class->push_back(f1);
		fault_classes.push_back(f1_class);
	}
}

std::map<SAFaultDescriptor*, std::list<SAFaultDescriptor*>* >* ds_faults::get_fault_universe(ds_structural::NetList* nl){
	//fault classes
	std::map<SAFaultDescriptor*, std::list<SAFaultDescriptor*>* > *universe = new std::map<SAFaultDescriptor*, std::list<SAFaultDescriptor*>* >();
	//all gates in the netlist
	std::vector<ds_structural::Gate*> gates;
	nl->get_gates(gates);

	std::vector<std::list<SAFaultDescriptor*>* > fault_classes;
	std::map<SAFaultDescriptor*, SAFaultDescriptor*> descriptor_map;
	//calculate fault classes for each netlist gate
	for (ds_structural::Gate* g:gates){
		ds_faults::get_fault_classes(g, fault_classes);
	}

	//calculate all faults for inputs and outputs
	std::vector<const ds_structural::PortBit*> ports;
	const ds_structural::port_container* inputs = nl->get_inputs();
	const ds_structural::port_container* outputs = nl->get_outputs();
	ports.insert(ports.begin(), inputs->begin(), inputs->end());
	ports.insert(ports.begin(), outputs->begin(), outputs->end());
	for (const ds_structural::PortBit *pb: ports){
		SAFaultDescriptor *sa0 = new SAFaultDescriptor(pb->get_gate()->get_instance_name(), pb->get_instance_name(), ds_common::BIT_0);
		std::list<SAFaultDescriptor*> *list0 = new std::list<SAFaultDescriptor*>();
		list0->push_back(sa0);
		SAFaultDescriptor *sa1 = new SAFaultDescriptor(pb->get_gate()->get_instance_name(), pb->get_instance_name(), ds_common::BIT_1);
		std::list<SAFaultDescriptor*> *list1= new std::list<SAFaultDescriptor*>();
		list1->push_back(sa1);
		fault_classes.push_back(list0);
		fault_classes.push_back(list1);
	}

	// map fault name to descriptor and select representative
	std::map<std::string, SAFaultDescriptor*> fault_registry;
	for (auto it=fault_classes.begin();it!=fault_classes.end();it++){
		std::list<SAFaultDescriptor*>* f_class = *it;
		auto out_it = f_class->begin();
		SAFaultDescriptor *rep = *out_it;
		(*universe)[rep] = f_class;
		for (auto it=f_class->begin();it!=f_class->end();it++){
			SAFaultDescriptor *d = *it;
			descriptor_map[d] = rep;
			fault_registry[d->get_string()] = d;
		}
	}

	//evaluate all signals
	std::vector<ds_structural::Signal*> signals;
	nl->get_signals(signals);
	for (ds_structural::Signal* s:signals){
		if (s->count_ports() == 2){

			//calculate equivalent faults if only two ports are connected (no fan-out nodes)

			auto it =  s->port_begin();
			ds_structural::PortBit *first = *it;
			ds_structural::PortBit *second = *(++it);

			// the aggressor is kept and the victim is removed
			SAFaultDescriptor d_aggressor1 = SAFaultDescriptor(first->get_gate()->get_instance_name(), first->get_instance_name(), ds_common::BIT_1);
			SAFaultDescriptor d_aggressor0 = SAFaultDescriptor(first->get_gate()->get_instance_name(), first->get_instance_name(), ds_common::BIT_0);
			SAFaultDescriptor d_victim1 = SAFaultDescriptor(second->get_gate()->get_instance_name(), second->get_instance_name(), ds_common::BIT_1);
			SAFaultDescriptor d_victim0 = SAFaultDescriptor(second->get_gate()->get_instance_name(), second->get_instance_name(), ds_common::BIT_0);

			// pointer to created descriptor is not available. Search for it in the registry
			SAFaultDescriptor* aggressor1 = fault_registry[d_aggressor1.get_string()];
			SAFaultDescriptor* aggressor0 = fault_registry[d_aggressor0.get_string()];
			SAFaultDescriptor* victim1 = fault_registry[d_victim1.get_string()];
			SAFaultDescriptor* victim0 = fault_registry[d_victim0.get_string()];

			// find representatives
			SAFaultDescriptor* rep_aggressor1 = descriptor_map[aggressor1];
			SAFaultDescriptor* rep_aggressor0 = descriptor_map[aggressor0];
			SAFaultDescriptor* rep_victim1 = descriptor_map[victim1];
			SAFaultDescriptor* rep_victim0 = descriptor_map[victim0];

			// merge equivalent fault in the two classes (that of the aggressor and victim)
			std::list<SAFaultDescriptor*> *equivalent1 = (*universe)[rep_aggressor1];
			std::list<SAFaultDescriptor*> *equivalent0 = (*universe)[rep_aggressor0];
			std::list<SAFaultDescriptor*> *merge1 = (*universe)[rep_victim1];
			std::list<SAFaultDescriptor*> *merge0 = (*universe)[rep_victim0];

			if (equivalent0->size() == 1 || merge0->size() == 1){
				equivalent0->insert(equivalent0->begin(), merge0->begin(), merge0->end());
				//update representative map
				for (auto it=merge0->begin();it!=merge0->end();it++){
					descriptor_map[*it] = rep_aggressor0;
				}
				delete (merge0);
				universe->erase(rep_victim0);
			}

			if (equivalent1->size() == 1 || merge1->size() == 1){
				equivalent1->insert(equivalent1->begin(), merge1->begin(), merge1->end());
				for (auto it=merge1->begin();it!=merge1->end();it++){
					descriptor_map[*it] = rep_aggressor1;
				}
				delete (merge1);
				universe->erase(rep_victim1);
			}
		}
	}
	return universe;
}
std::set<SAFaultDescriptor*>* ds_faults::get_collapsed_faults(ds_structural::NetList* nl){
	std::set<SAFaultDescriptor*>* collapsed = new std::set<SAFaultDescriptor*>();
	//get all fault classes
	std::map<SAFaultDescriptor*, std::list<SAFaultDescriptor*>* >* universe = ds_faults::get_fault_universe(nl);
	for (auto it=universe->begin();it!=universe->end();it++){
		// make a copy
		SAFaultDescriptor* copy = new SAFaultDescriptor(*it->first);
		collapsed->insert(copy);
	}
	// delete fault classes
	ds_faults::delete_faults(universe);
	return collapsed;
}
void ds_faults::delete_faults(std::map<SAFaultDescriptor*, std::list<SAFaultDescriptor*>* >* faults){
	for (auto it=faults->begin();it!=faults->end();it++){
		SAFaultDescriptor* eq_ptr = it->first;
		SAFaultDescriptor eq = *eq_ptr;
		std::list<SAFaultDescriptor*> *faults = it->second;
		for (auto f_it=faults->begin();f_it!=faults->end();f_it++){
			SAFaultDescriptor* f = *f_it;
			delete(f);
		}
		delete(faults);
	}
}

ds_faults::FaultList::FaultList() : uk({}), ds({}), np({}), ud({}), representatives({}), fault_map({}) {}
void ds_faults::FaultList::add(SAFaultDescriptor* saf) {
    this->representatives[saf->get_string()] = saf;
    uk.insert(saf);
    fault_map[saf] = UK;
}
uint ds_faults::FaultList::size() {return this->representatives.size();}

ds_faults::FaultList::FaultList(ds_structural::NetList* nl) {
	std::map<SAFaultDescriptor*, std::list<SAFaultDescriptor*>* >* universe = ds_faults::get_fault_universe(nl);
	for (auto it=universe->begin();it!=universe->end();it++){
		SAFaultDescriptor* f = new SAFaultDescriptor(*it->first);
		uk.insert(f);
		fault_map[f] = UK;
		std::list<SAFaultDescriptor*> *faults = it->second;
		for (auto f_it=faults->begin();f_it!=faults->end();f_it++){
			SAFaultDescriptor* eq = *f_it;
			representatives[eq->get_string()] = f;
		}
	}
	ds_faults::delete_faults(universe);
}

void ds_faults::FaultList::reset_fault_categories(){
	uk.clear();
	uk.insert(ds.begin(), ds.end());
	uk.insert(ud.begin(), ud.end());
	fault_map.clear();
	for (SAFaultDescriptor *f:uk){
		fault_map[f] = ds_faults::UK;
	}
}
void ds_faults::FaultList::set_fault_category(SAFaultDescriptor *f, const FaultCategory& category){
	FaultCategory current_class = fault_map[f];
	std::set<SAFaultDescriptor*>* current_container = find_container(current_class);
	std::set<SAFaultDescriptor*>* new_container = find_container(category);
	if (new_container!=0){
		current_container->erase(f);
		new_container->insert(f);
		fault_map[f] = category;
	}
}
std::set<SAFaultDescriptor*>* ds_faults::FaultList::find_container(const FaultCategory& category){
	switch(category){
	case ds_faults::UK:
		return &uk;
	case ds_faults::DS:
		return &ds;
	case ds_faults::UD:
		return &ud;
	case ds_faults::NP:
		return &np;
	case ds_faults::UR:
	  return &ur;
	default:
		return 0;
	}
}
ds_faults::FaultList::~FaultList(){
	reset_fault_categories();
	for (SAFaultDescriptor *f:uk){
		delete f;
	}
	uk.clear();
}
void ds_faults::FaultList::set_fault_category(std::string n, std::string p, const ds_common::Value& v, const ds_faults::FaultCategory& category){
	SAFaultDescriptor d(n,p,v);
	SAFaultDescriptor* f = representatives[d.get_string()];
	set_fault_category(f, category);
}

double ds_faults::FaultList::get_fc() const {
	int total_faults = fault_map.size();
	double detected = ds.size() + X_WEIGHT * np.size();
	double fc = detected / total_faults * 100;
	return fc;
}

void ds_faults::read_fastscan_descriptors(const std::string& file_name, std::vector<ds_faults::fastscan_descriptor>& descriptors){
	std::ifstream input(file_name.c_str());
	std::string line;
	if (input.is_open()){
		while (!input.eof()){
			std::getline(input, line);
			if (!line.empty()) {
				parse_fastscan_faults<std::string::iterator>(line.begin(), line.end(), descriptors);
			}
		}
	} else {
		BOOST_LOG_TRIVIAL(error) << "Error loading fault descriptor file: " << file_name << ". Device not open";
		BOOST_THROW_EXCEPTION(ds_common::file_read_error() << boost::errinfo_errno(errno));
	}
}

ds_lg::int64 ds_faults::TransitionFault::compare(const ds_lg::driver_v64* a, const ds_lg::driver_v64& b) const{
	ds_lg::lg_v64 v = a->value;
	ds_lg::lg_v64 t = b.value;
	const ds_lg::TNode *driver = a->driver;
	const ds_lg::lg_v64 *p = driver->get_previous_value(a->port_id);
	ds_lg::int64 cond = ((~p->v & v.v & t.v) | (p->v & ~v.v & ~t.v)) & ~v.x & ~t.x & ~p->x;
	return cond;
}


ds_faults::MFaultList::MFaultList() : flst(ds_faults::FaultList()), mflst({}), cnt_rejected(0) {}

ds_faults::MFaultList::MFaultList(ds_structural::NetList* nl, std::string filename) : MFaultList() {
	load_MFL(nl, filename);
}

bool ds_faults::MFaultList::load_MFL(ds_structural::NetList* nl, std::string filename) {
    std::ifstream ifs(filename);
    std::string def;
    if (!ifs.is_open()) {return false;}
    while (!ifs.eof()) {
        std::getline(ifs, def);
        if (def != "" && !boost::starts_with(def, "//")) {if (!add(nl, def)) {return false;}}
    }
    ifs.close();
    return true;
}

bool ds_faults::MFaultList::store_MFL(std::string filename) {
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {return false;}
    for(auto it : mflst) {
        FaultCategory ftype = flst.get_fault_category(it.first);
        if (ftype == DS) {ofs << "DS,";} else if (ftype == UR) {ofs << "UR,";} else {ofs << "UK,";}
        ds_faults::SAFaultDescriptor *f = flst.representatives[it.first];
        ofs << f->gate_name << "@" << f->port_name << "@" << (f->value == 0 ? "0" : "1") << "=" << this->get_MF_str(it.first) << std::endl;
    }
    ofs.close();
    return true;
}

bool ds_faults::MFaultList::add(ds_faults::SAFaultDescriptor* saf, ds_faults::StuckAt* sa) {
    mflst[saf->get_string()].push_back(sa);
    flst.add_undetected(saf);
    this->cnt_total++;
    return true;
}

// line format example: \inst_cpu_ip_alx/alx_e_dra_reg[0] @Q@1=\inst_cpu_ip_alx/alx_e_dra_reg_T2[0] @Z@1,SI273@Z@1,\inst_cpu_ip_alx/alx_e_dra_reg_T3[0] @Z@1,
bool ds_faults::MFaultList::add(ds_structural::NetList* nl, std::string definition) {
    boost::regex rx_def("([A-Z][A-Z]),([a-zA-Z0-9\\\\\\[\\]\\/\\_\\-\\s]*)@([a-zA-Z0-9\\\\\\[\\]\\/\\_\\-\\s]+)@([01])=(.*)");
    boost::regex rx_singlefault("([a-zA-Z0-9\\\\\\[\\]\\/\\_\\-\\s]*)@([a-zA-Z0-9\\\\\\[\\]\\/\\_\\-\\s]+)@([01])");
    boost::smatch match;

    if (!boost::regex_search(definition, match, rx_def)) {
        std::cout << "Warning: fault list entry not parseable: " << definition << "\n";
        return false;
    }

    std::string ftype = match[1].str();
    ds_faults::SAFaultDescriptor* saf = new ds_faults::SAFaultDescriptor(match[2].str(),match[3].str(), (ds_common::Value) boost::lexical_cast<int>(match[4].str()));
    //std::cout << "adding fault " << match[1].str() << ", " << match[2].str() << ", " << match[3].str() << ", " << match[4].str() << std::endl;
    std::string fl = match[5].str();
    std::vector<std::string> sfd;
    ds_faults::StuckAt* sa;
    boost::algorithm::split(sfd, fl, boost::algorithm::is_any_of(","));
    for (std::string sf : sfd) {             //for all single faults of this fault group:
        if (!boost::regex_search(sf, match, rx_singlefault)) {return false;}
        if (match[1].str() != "" && nl->find_gate(match[1].str()) == NULL) {            //check if gate exists
            std::cout << "Warning: gate " << match[1].str() << " not existing, fault list entry " << definition << "\n";
            this->cnt_rejected++;
        } else if (match[1].str() == "" && nl->find_port_by_name(match[2].str()) == NULL) {
            std::cout << "Warning: top-level port " << match[2].str() << " not existing, fault list entry " << definition << "\n";
            this->cnt_rejected++;
        } else {
            if (match[1].str() == nl->get_instance_name()) {
                        sa = new ds_faults::StuckAt(nl, "", match[2].str(), (ds_common::Value) boost::lexical_cast<int>(match[3].str()));
            } else {
                        sa = new ds_faults::StuckAt(nl, match[1].str(), match[2].str(), (ds_common::Value) boost::lexical_cast<int>(match[3].str()));
            }
            mflst[saf->get_string()].push_back(sa);
        }
    }
    flst.add_undetected(saf);

    if (ftype == "DS") {
        flst.set_fault_category(saf, ds_faults::DS);
    } else if (ftype == "US") {
        flst.set_fault_category(saf, ds_faults::UR);
    } else {
        if (ftype != "UK") {std::cout << "Warning: unparseable fault type " << ftype << " degraded to UK\n";}

    }
    this->cnt_total++;
    return true;
}

ds_faults::FaultList* ds_faults::MFaultList::get_faultlist() {
	return &flst;
}

ds_faults::MSAFault ds_faults::MFaultList::get_MF(std::string fault_descriptor) {
	return mflst[fault_descriptor];
}

std::string ds_faults::MFaultList::get_MF_str(std::string fault_descriptor) {
    std::string result = "";
    MSAFault msa = this->get_MF(fault_descriptor);
    for (uint i=0; i<msa.size(); i++) {
        result += msa[i]->get_gate_name() + "@" + msa[i]->get_port_name() + "@" + (msa[i]->get_value() == 0 ? "1" : "0");
        if (i < msa.size()-1) {result += ",";}
    }
    return result;
}

uint ds_faults::MFaultList::get_count(std::string fault_descriptor) {
    ds_faults::MSAFault fmlst = this->get_MF(fault_descriptor);
    return fmlst.size();
}

