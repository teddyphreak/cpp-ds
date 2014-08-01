/*
 * ds_diagnosis.h
 *
 *  Created on: Mar 31, 2014
 *      Author: cookao
 */

#ifndef DS_DIAGNOSIS_H_
#define DS_DIAGNOSIS_H_

#include "ds_faults.h"
#include "ds_pattern.h"
#include "ds_lg.h"
#include <vector>
#include <set>
#include <unordered_map>
#include <boost/log/trivial.hpp>

namespace ds_diagnosis {

class Evidence {
public:

	int get_sigma() const {
		return sigma;
	}

	void set_sigma(const int& s) {
		sigma = s;
	}

	void add_sigma(const int& s){
		sigma += s;
	}

	int get_gamma() const {
		return gamma;
	}

	void set_gamma(const int& g) {
		gamma = g;
	}

	void add_gamma(const int& g){
		gamma += g;
	}

	int get_iota() const {
		return iota;
	}

	void set_iota(const int& i) {
		iota = i;
	}

	void add_iota(const int& i){
		iota += i;
	}

	int get_tau() const {
		return tau;
	}

	void set_tau(const int& t) {
		tau = t;
	}

	void add_tau(const int& t){
		tau += t;
	}

	int get_list() const {
		return list;
	}

	void set_list(const int& list) {
		this->list = list;
	}

	int get_rank() const {
		return rank;
	}

	void set_rank(const int& rank) {
		this->rank = rank;
	}

	void reset(){
		sigma = 0;
		iota = 0;
		tau = 0;
		gamma = 0;
	}

protected:

	int sigma;
	int iota;
	int tau;
	int gamma;

	int rank;
	int list;

};

template<class N, class T, class F>
class Candidate : public ds_faults::PinReference{

public:

	Candidate(const unsigned int size, F *f):num_evidences(size), fault(f){
		for (unsigned int i=0;i<num_evidences;i++){
			Evidence* e = new Evidence();
			evidences.push_back(e);
		}
	}

	virtual std::string get_gate_name() const {
		return fault->get_gate_name();
	}
	virtual std::string get_port_name() const {
		return fault->get_port_name();
	}
	virtual std::string get_qualified_name() const {
		return fault->get_qualified_name();
	}
	virtual bool is_input() const {
		return fault->is_input();
	}
	virtual bool is_output() const {
		return fault->is_output();
	}

	~Candidate(){
		for (unsigned int i=0;i<num_evidences;i++){
			Evidence* e = evidences[i];
			delete(e);
		}
	}

	void reset(){
		for (Evidence *e:evidences){
			e->reset();
		}
	}

	void add_sigma(const std::size_t& index, const int& sigma){
		evidences[index]->add_sigma(sigma);
	}

	void add_iota(const std::size_t& index, const int& iota){
		evidences[index]->add_iota(iota);
	}

	void add_gamma(const std::size_t& index, const int& gamma){
		evidences[index]->add_gamma(gamma);
	}

	void add_tau(const std::size_t& index, const int& tau){
		evidences[index]->add_tau(tau);
	}

	unsigned int get_num_evidences() const {
		return num_evidences;
	}

	Evidence* get_evidence(const std::size_t& index) const{
		return evidences[index];
	}

	F* get_fault()const{
		return fault;
	}

protected:
	unsigned int num_evidences;
	F* fault;
	std::vector<Evidence*> evidences;
};

template <class N, class T, class F, class DO>
class DiagnosticObserver : public ds_lg::Monitor<T> {

protected:

	std::vector<T> spec;
	std::vector<T> faulty;
	std::set<DO*>* faulty_nodes;
	T sim;

public:

	virtual void observe(const T& v){
		sim = v;
	}

	DiagnosticObserver(const std::size_t& size, std::set<DO*>* fn): faulty_nodes(fn){
		for (std::size_t i=0;i<size;i++){
			T default_val;
			spec.push_back(default_val);
			faulty.push_back(default_val);
		}
	}

	virtual void update_evidence(Candidate<N,T,F>* c, const ds_lg::lg_v64& obs){
		for (unsigned int i=0;i<c->get_num_evidences();i++){
			unsigned int sigma = compare_sigma(spec[i], faulty[i], obs);
			c->add_sigma(i, sigma);
			unsigned int iota = compare_iota(spec[i], faulty[i], obs);
			c->add_iota(i, iota);
			unsigned int gamma = compare_gamma(spec[i], faulty[i], obs);
			c->add_gamma(i, gamma);
			unsigned int tau = compare_tau(spec[i], faulty[i], obs);
			c->add_tau(i, tau);
		}
	}
	void set_spec(const unsigned int& index, const T& v){
		if (index < 0 || index > spec.size()){
			throw std::out_of_range ("Value out of range: " + index);
		}
		spec[index] = v;
	}
	void set_spec(const T& v){
		for (std::size_t index=0;index<spec.size();index++){
			spec[index] = v;
		}
	}
	void set_faulty_result(const unsigned int& index, const T& v){
		if (index < 0 || index > faulty.size()){
			throw std::out_of_range ("Value out of range: " + index);
		}
		faulty[index] = v;
	}
	virtual unsigned int compare_sigma(const T& spec, const T& faulty, const ds_lg::lg_v64& obs)=0;
	virtual unsigned int  compare_iota(const T& spec, const T& faulty, const ds_lg::lg_v64& obs)=0;
	virtual unsigned int compare_gamma(const T& spec, const T& faulty, const ds_lg::lg_v64& obs)=0;
	virtual unsigned int   compare_tau(const T& spec, const T& faulty, const ds_lg::lg_v64& obs)=0;
};

class TDiagnosticObserver : public DiagnosticObserver<ds_lg::TNode, ds_lg::driver_v64, ds_faults::TStuckAt, TDiagnosticObserver> {
public:
	TDiagnosticObserver(const std::size_t& size,  std::set<TDiagnosticObserver*>* fn):DiagnosticObserver(size, fn){}

	virtual void observe(const ds_lg::driver_v64& v){
		sim.value = v.value;
		for (ds_lg::driver_v64 s:spec){
			if ((sim.value.v ^ s.value.v) & ~sim.value.x & ~s.value.x){
				faulty_nodes->insert(this);
				break;
			}
		}
	}

	virtual unsigned int compare_sigma(const ds_lg::driver_v64& s, const ds_lg::driver_v64& f, const ds_lg::lg_v64& obs){
		ds_common::int64 sigma = (s.value.v ^ f.value.v) & ~(f.value.v ^ sim.value.v) & obs.v & ~s.value.x & ~obs.x & ~sim.value.x;
		return __builtin_popcountl(sigma);
	}
	virtual unsigned int compare_iota(const ds_lg::driver_v64& s, const ds_lg::driver_v64& f, const ds_lg::lg_v64& obs){
		ds_common::int64 iota = ~(s.value.v ^ f.value.v) & (f.value.v ^ sim.value.v) & obs.v & ~s.value.x & ~obs.x & ~sim.value.x;
		return __builtin_popcountl(iota);
	}
	virtual unsigned int compare_gamma(const ds_lg::driver_v64& s, const ds_lg::driver_v64& f, const ds_lg::lg_v64& obs){
		ds_common::int64 gamma = (s.value.v ^ f.value.v) & (s.value.v ^ sim.value.v) & obs.v & ~s.value.x & ~obs.x & ~sim.value.x;
		return __builtin_popcountl(gamma);
	}
	virtual unsigned int compare_tau(const ds_lg::driver_v64& s, const ds_lg::driver_v64& f, const ds_lg::lg_v64& obs){
		ds_common::int64 sigma = (s.value.v ^ f.value.v) & ~(f.value.v ^ sim.value.v) & obs.v & ~s.value.x & ~obs.x & ~sim.value.x;
		ds_common::int64 iota = ~(s.value.v ^ f.value.v) & (f.value.v ^ sim.value.v) & obs.v & ~s.value.x & ~obs.x & ~sim.value.x;
		unsigned int sigma_cnt = __builtin_popcountl(sigma);
		unsigned int iota_cnt = __builtin_popcountl(iota);
		if (sigma_cnt < iota_cnt)
			return sigma_cnt;
		else
			return iota_cnt;
	}
};

template <class L, class C, class N, class F, class O>
class Diagnoser {

protected:

	L *lg;
	std::vector<C*> candidates;
	std::unordered_map<N*, O*> observer_map;
	ds_faults::FaultList *fl;
	ds_common::int64 detected;
	ds_common::int64 possibly_detected;
	std::map<N*, std::set<C*>* > check_points;
	std::set<O*> faulty_nodes;

public:

	typedef std::vector<O*> node_container;

	Diagnoser(L* graph, const std::size_t& size){

		lg = graph;
		fl = new ds_faults::FaultList(lg->get_netlist());

		//get all faults
		std::set<ds_faults::SAFaultDescriptor*> fault_set;
		fl->get_undetected_faults(fault_set);

		//the check points are inputs, outputs or fanount nodes. The values of the map are the faults dominated by
		//the corresponding check point

		for (ds_faults::SAFaultDescriptor* d:fault_set){

			N *node = lg->get_node(d->gate_name);

			if (node==0){
				node = lg->get_node(d->port_name);
			}

			N *cp = lg->get_check_point(node);

			F *f = new F(lg->get_netlist(), d->gate_name, d->port_name, d->value);

			C *c = new C(size, f);
			candidates.push_back(c);

			if ((cp->has_state() && d->gate_name == cp->get_name() && f->is_input()) ||
					(cp->has_state() && d->gate_name != cp->get_name())){
				cp = cp->get_sink();
			}

			std::set<C*>* faults = 0;
			auto it = check_points.find(cp);
			if (it==check_points.end()){
				faults = new std::set<C*>();	// no checkpoint for this fault yet
				check_points[cp] = faults;
			} else {
				faults = check_points[cp];		// insert into the available check point entry
			}
			faults->insert(c);
		}

		for (auto it=lg->outputs.begin(); it!=lg->outputs.end();it++){
			N *o = *it;
			O *observer = new O(size, &faulty_nodes);
			observer_map[o] = observer;
			o->add_monitor(observer);
		}
		for (auto it=lg->registers.begin(); it!=lg->registers.end();it++){
			N *r = *it;
			N *d = r->get_sink();
			O *observer = new O(size, &faulty_nodes);
			observer_map[d] = observer;
			d->add_monitor(observer);
		}
	}

	ds_faults::SAFaultDescriptor* get_representative(ds_faults::SAFaultDescriptor* f){
		ds_faults::SAFaultDescriptor* d = fl->get_representative(f->get_string());
		if (d==0){
			std::string name = lg->get_netlist()->get_instance_name() + f->get_string();
			d = fl->get_representative(name);
		}
		return d;
	}

	void clear_nodes(){
		faulty_nodes.clear();
	}

	void reset(){
		fl->reset_fault_categories();
		for (C* c:candidates){
			c->reset();
		}
		clear_nodes();

	}

	L* get_leveled_graph()const{
		return lg;
	}

	O* get_observer(N* node){
		return observer_map[node];
	}

	void set_spec(ds_pattern::SimPatternBlock* block){
		lg->clear_hooks();
		lg->sim(block);

		for (auto it=lg->outputs.begin(); it!=lg->outputs.end();it++){
			N *o = *it;
			O *observer  = observer_map[o];
			observer->set_spec(o->peek());

		}
		for (auto it=lg->registers.begin(); it!=lg->registers.end();it++){
			N *r = *it;
			N *d = r->get_sink();
			O *observer = observer_map[d];
			observer->set_spec(d->peek());
		}
	}

	template<class NN>
	void set_faulty_response(const std::size_t& index, NN* node){
		std::string name = node->get_name();
		ds_lg::lg_v64 val = node->resolve();
		N* n = lg->get_node(name);
		O* observer = observer_map[n];
		if (node->has_state()){
			val = node->get_sink()->resolve();
			observer = observer_map[n->get_sink()];
		}

		if (observer==0){
			std::cout << "Observer not found" << std::endl;
		}
		observer->set_faulty_result(index, val);

	}

	void update(){

		for (auto cp_it=check_points.begin();cp_it!=check_points.end();cp_it++){

			N* cp = cp_it->first;
			std::set<C*> *candidates = cp_it->second;

			clear_nodes();

			cp->flip_and_observe();
			for (N* o:cp->outputs){
				lg->push_node(o);
			}

			//intermediate simulation
			lg->sim_intermediate();

			//reset check point to fault-free state
			cp->rollback();

			for (auto c_it=candidates->begin();c_it!=candidates->end();c_it++){

				C* c = *c_it;

				F *f = c->get_fault();

				ds_lg::lg_v64 obs = lg->propagate_to_check_point(f);

				for (auto n_it=faulty_nodes.begin();n_it!=faulty_nodes.end();n_it++){

					O* observer = *n_it;

					observer->update_evidence(c, obs);

				}
			}
		}
	}
};

class TCandidate : public ds_diagnosis::Candidate<ds_lg::TNode, ds_lg::driver_v64, ds_faults::TStuckAt> {
public:
	TCandidate(const unsigned int size, ds_faults::TStuckAt *f) : ds_diagnosis::Candidate<ds_lg::TNode,
			ds_lg::driver_v64, ds_faults::TStuckAt>(size, f){}
};

class TDiagnoser : public Diagnoser<ds_lg::TLeveledGraph, TCandidate,
ds_lg::TNode, ds_faults::TStuckAt, TDiagnosticObserver> {

public:

	TDiagnoser(ds_lg::TLeveledGraph* graph, const std::size_t& size): Diagnoser(graph, size){}
	void rank(std::vector<TCandidate*>* container, const std::size_t& index){
		container->insert(container->begin(), candidates.begin(), candidates.end());
		std::sort(container->begin(), container->end(), [index](const TCandidate* c1, const TCandidate* c2){
			Evidence *e1 = c1->get_evidence(index);
			Evidence *e2 = c2->get_evidence(index);
			if (e1->get_sigma() > e2->get_sigma())
				return true;
			if ((e1->get_sigma() == e2->get_sigma()) && (e1->get_iota() < e2->get_iota()))
				return true;
			if ((e1->get_sigma() == e2->get_sigma()) && (e1->get_iota() == e2->get_iota()) && (e1->get_gamma() < e2->get_gamma()))
				return true;
			return false;
		});
	}
};

template<class LG>
void diagnose_loc(ds_diagnosis::TDiagnoser* diagnoser, ds_pattern::PatternProvider *provider, std::vector<LG*>& faulty_circuits,
		std::vector<std::vector<ds_diagnosis::TCandidate*>* >& candidates){

	diagnoser->reset();

	while (provider->has_next()){

		ds_pattern::SimPatternBlock *block = provider->next();

		ds_lg::TLeveledGraph* lg = diagnoser->get_leveled_graph();
		lg->sim(block);

		diagnoser->set_spec(block);

		for (int i=0;i<faulty_circuits.size();i++){

			LG* dud = faulty_circuits[i];
			dud->sim(block);

			for (auto reg_it=dud->registers.begin();reg_it!=dud->registers.end();reg_it++){

				diagnoser->set_faulty_response(i, *reg_it);

			}

			for (auto out_it=dud->outputs.begin();out_it!=dud->outputs.end();out_it++){

				diagnoser->set_faulty_response(i, *out_it);

			}

		}

		diagnoser->update();

	}

	for (int i=0;i<faulty_circuits.size();i++){

		std::vector<ds_diagnosis::TCandidate*>* list = candidates[i];

		diagnoser->rank(list, i);

	}
}

}
#endif /* DS_DIAGNOSIS_H_ */
