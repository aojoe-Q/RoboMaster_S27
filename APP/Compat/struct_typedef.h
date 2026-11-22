#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

/*
 * Compatibility header for the APP/Modules code taken from S27_ARM_H7.
 * The original project re-typedef'd the standard integer types, which
 * conflicts with <stdint.h> already used by the S27_TEST_1 template.
 * Only the project-specific floating point aliases are kept here.
 */
#include <stdint.h>

typedef float  fp32;
typedef double fp64;
typedef unsigned char bool_t;

#endif /* STRUCT_TYPEDEF_H */
