package ode

// #include <ode/ode.h>
import "C"

import (
	"unsafe"
)

type HeightfieldData uintptr

func cToHeightfieldData(c C.dHeightfieldDataID) HeightfieldData {
	return HeightfieldData(unsafe.Pointer(c))
}

func (h HeightfieldData) c() C.dHeightfieldDataID {
	return C.dHeightfieldDataID(unsafe.Pointer(h))
}

func NewHeightfieldData() HeightfieldData {
	return cToHeightfieldData(C.dGeomHeightfieldDataCreate())
}

func (d *HeightfieldData) Destroy() {
	C.dGeomHeightfieldDataDestroy(d.c())
}
