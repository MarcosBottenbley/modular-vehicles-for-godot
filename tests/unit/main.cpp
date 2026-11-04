// Unit-test entry point for the std-only math core (src/math). Built by `scons tests` into
// bin/tests/mv_tests; doctest supplies main() and discovers the TEST_CASEs in the other files.
//
// Terms:
//   test binary: bin/tests/mv_tests(.exe), this file plus tests/unit/test_*.cpp linked together.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
