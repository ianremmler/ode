#include <ode/ode.h>
#include <stdio.h>
#include "_cgo_export.h"

void callNearCallback(void *data, dGeomID obj1, dGeomID obj2) {
	nearCallback(data, obj1, obj2);
}
