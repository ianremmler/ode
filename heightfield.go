package ode

// #include <ode/ode.h>
import "C"

import (
	"unsafe"
)

// HeightfieldData represents heightfield data.
type HeightfieldData uintptr

func cToHeightfieldData(c C.dHeightfieldDataID) HeightfieldData {
	return HeightfieldData(unsafe.Pointer(c))
}

func (h HeightfieldData) c() C.dHeightfieldDataID {
	return C.dHeightfieldDataID(unsafe.Pointer(h))
}

// NewHeightfieldData returns a new HeightfieldData instance.
func NewHeightfieldData() HeightfieldData {
	return cToHeightfieldData(C.dGeomHeightfieldDataCreate())
}

// Destroy destroys the heightfield data.
func (h *HeightfieldData) Destroy() {
	C.dGeomHeightfieldDataDestroy(h.c())
}
