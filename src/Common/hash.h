

#ifndef COMMON_HASH_H
#define COMMON_HASH_H

#include <functional>
#include "common_global.h"

namespace insight{

	// Combine hashing value
	// http://www.boost.org/doc/libs/1_37_0/doc/html/hash/reference.html#boost.hash_combine
	template <class T>
	inline void hash_combine(std::size_t& seed, const T& v)
	{
		std::hash<T> hasher;
		seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
	}

}//name space insight
#endif // COMMON_HASH_H


