// =============================================================================
// Header file for mems operations
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================void
#ifndef MEMS_DRV_H
#define MEMS_DRV_H

#include "fcmp.h"

void mems_enable();
void mems_disable();
void mems_set(fcmp_field_pos *position);

#endif
