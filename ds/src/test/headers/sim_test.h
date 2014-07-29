/*
 * sim_test.h
 *
 *  Created on: 23.04.2013
 *      Author: cookao
 */

#ifndef SIM_TEST_H_
#define SIM_TEST_H_

namespace sim_test {

	void lg_sim_test(const std::string& design, const std::string& wgl_file, const std::string& top);
	void fc_test(const std::string& design, const std::string& wgl_file, const std::string& top);
	void lg_loc_test(const std::string& design, const std::string& wgl_file, const std::string& top);
	void fc_tdf_test(const std::string& design, const std::string& wgl_file, const std::string& top, const std::string& faults);
	void fc_timing_test(const std::string& design, const std::string& wgl_file, const std::string& top, const std::string& sdf_file);
	void test_sim();
	void test_fc();
	void test_loc();
	void test_tdf();
	void test_timing();

	struct sim_ts : public boost::unit_test::test_suite
	{
		sim_ts(const std::string& name):boost::unit_test::test_suite(name)
		{
			//add( BOOST_TEST_CASE( &test_sim ) );
			//add( BOOST_TEST_CASE( &test_fc ) );
			//add( BOOST_TEST_CASE( &test_loc ) );
			//add( BOOST_TEST_CASE( &test_tdf ) );
			add( BOOST_TEST_CASE( &test_timing ) );

		}
	};
}


#endif /* SIM_TEST_H_ */
