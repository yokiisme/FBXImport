#pragma once
#include "PlatformEnvironment.h"
//////////////////////////////////////////////////////////////////////////
/// Union with two types
template<typename A, typename B>
union UnionTuple
{
	typedef A first_type;
	typedef B second_type;

	A first;
	B second;
};

template<class Target, class Source>
Target bit_cast(const Source& a)
{
	// See also C++ proposal of function with similar semantics and the same name:
	// http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2017/p0476r2.html

	// TODO: forbid casting of pointers via this. Without additional compiler
	// support this definitely breaks rules of strict aliasing.

	// Using union to convert between bit representations is the only way that
	// is somewhat portable between compilers. Note that by the letter of the
	// C++ standard even this is undefined behavior, because we're reading from
	// the union member which we did not write to during the last write. However,
	// the compilers provide support for this pattern as an extension.


	UnionTuple<Source, Target> storage;
	storage.first = a;
	return storage.second;
}

// Use this metafunction together with UnionTuple<A, B>, for type B. It returns a type of same size
// that can be safely loaded into registers while data is in endianess-swapped form.
template<class T>
struct BitSafeTypeFor
{
	typedef T type;
};

template<>
struct BitSafeTypeFor<float>
{
	typedef UInt32 type;
};

template<>
struct BitSafeTypeFor<double>
{
	typedef UInt64 type;
};
