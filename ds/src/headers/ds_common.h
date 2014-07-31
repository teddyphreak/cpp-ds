#ifndef DS_COMMON_H_
#define DS_COMMON_H_

#include <boost/exception/all.hpp>
#include <boost/serialization/access.hpp>

namespace ds_common {

	// Word length for simulation
	const int WIDTH = 64;
	typedef unsigned long int64;

	// Exceptions (not extensively or consistently used)
	typedef boost::error_info<struct tag_errmsg, std::string> errmsg_info;
	struct file_read_error: virtual boost::exception, virtual std::exception { };
	struct parse_error: virtual boost::exception, virtual std::exception { };
	/*!
	 * data structure for 3-valued simulation (0,1,X)
	 */
	struct lg_v64 {
		int64 v; //!< values
		int64 x; //!< x mask

		/*!
		 * default constructor. All values are X
		 */
		lg_v64():v(0),x(-1){}
		/*!
		 * initializes internal structure for given values
		 * @param vv values
		 * @param xx x mask
		 */
		lg_v64(const ds_common::int64 vv, const ds_common::int64 xx):v(vv),x(xx){}
		/*!
		 * copy constructor. It copies the values of the provided lg_v64
		 * @param v
		 */
		lg_v64(const lg_v64& v):v(v.v),x(v.x){}

		/*!
		 * unary inversion
		 * @return one's complement of the defined values
		 */
		lg_v64 operator~() const{
			lg_v64 o;
			o.v = ~v & ~x;
			o.x = x;
			return o;
		}

		/*!
		 * assignment operation
		 * @param rhs right-hand-side value
		 * @return a copy of the right-hand-side value
		 */
		lg_v64& operator=(const lg_v64 &rhs) {
			if (this != &rhs) {
				v = rhs.v;
				x = rhs.x;
			}

			return *this;
		}
		/*!
		 * unary 'and'
		 * @param rhs 'and' operand
		 * @return logic 'and' between this and provided operand
		 */
		lg_v64& operator&=(const lg_v64& rhs){
			v &= rhs.v;
			x = (x & ~rhs.x & rhs.v) | (rhs.x & ~x & v);
			return *this;
		}
		/*!
		 * unary 'or'
		 * @param rhs 'or' operand
		 * @return logic 'or' between this and provided operand
		 */
		lg_v64& operator|=(const lg_v64& rhs){
			v |= rhs.v;
			x = (x & ~rhs.x & ~rhs.v) | (rhs.x & x & ~v);
			return *this;
		}
		/*!
		 * unary 'xor'
		 * @param rhs 'xor' operand
		 * @return logic 'xor' between this and provided operand
		 */
		lg_v64& operator^=(const lg_v64& rhs){
			v ^= rhs.v;
			x = rhs.x | x;
			return *this;
		}
		private:
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version){
			ar & v;
			ar & x;
		}
		
	};
	/*!
	 * binary 'or'
	 * @param lhs first 'or' operand
	 * @param rhs second 'or' operand
	 * @return logic 'or' between this and provided operand
	 */
	inline lg_v64 operator&(lg_v64 lhs, const lg_v64& rhs){
		lhs &= rhs;
		return lhs;
	}
	/*!
	 * binary 'and'
	 * @param lhs first 'and' operand
	 * @param rhs second 'and' operand
	 * @return logic 'and' between this and provided operand
	 */
	inline lg_v64 operator|(lg_v64 lhs, const lg_v64& rhs){
		lhs |= rhs;
		return lhs;
	}
	/*!
	 * binary 'xor'
	 * @param lhs first 'xor' operand
	 * @param rhs second 'xor' operand
	 * @return logic 'or' between this and provided operand
	 */
	inline lg_v64 operator^(lg_v64 lhs, const lg_v64& rhs){
		lhs ^= rhs;
		return lhs;
	}
	
	/*!
	 * Symbolic representations of simulation values (for parsing)
	 */
	enum Value {
		BIT_0 = 0,
		BIT_1,
		BIT_X,
		BIT_UD
	};
}


#endif /* DS_COMMON_H_ */
