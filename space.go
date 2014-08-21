package ode

// #include <ode/ode.h>
// extern void callNearCallback(void *data, dGeomID obj1, dGeomID obj2);
import "C"

import (
	"unsafe"
)

// Space

type Space interface {
	c() C.dSpaceID
	Destroy()
	SetCleanup(mode bool)
	Cleanup() bool
	SetManualCleanup(mode bool)
	SetSublevel(sublevel int)
	Sublevel() int
	ManualCleanup() bool
	Clean()
	Class() int
	Add(g Geom)
	Remove(g Geom)
	Query(g Geom) bool
	NumGeoms(g Geom) int
	Geom(index int) Geom
	Collide(data interface{}, cb NearCallback)
	NewSphere(radius float64) Sphere
	NewBox(lens Vector3) Box
	NewPlane(params Vector4) Plane
	NewCapsule(radius, length float64) Capsule
	NewCylinder(radius, length float64) Cylinder
	NewRay(length float64) Ray
	NewHeightfield(data HeightfieldData, placeable bool) Heightfield
	NewSimpleSpace() SimpleSpace
	NewHashSpace() HashSpace
	NewQuadTreeSpace(center, extents Vector3, depth int) QuadTreeSpace
	NewSweepAndPruneSpace(axisOrder int) SweepAndPruneSpace
}

// SpaceBase

type SpaceBase uintptr

func cToSpace(c C.dSpaceID) Space {
	base := SpaceBase(unsafe.Pointer(c))
	var s Space
	switch int(C.dSpaceGetClass(c)) {
	case SimpleSpaceClass:
		s = SimpleSpace{base}
	case HashSpaceClass:
		s = HashSpace{base}
	case QuadTreeSpaceClass:
		s = QuadTreeSpace{base}
	case SweepAndPruneSpaceClass:
		s = SweepAndPruneSpace{base}
	default:
		s = base
	}
	return s
}

func NilSpace() Space {
	return SpaceBase(0)
}

func (s SpaceBase) c() C.dSpaceID {
	return C.dSpaceID(unsafe.Pointer(s))
}

func (s SpaceBase) Destroy() {
	C.dSpaceDestroy(s.c())
}

func (s SpaceBase) SetCleanup(mode bool) {
	C.dSpaceSetCleanup(s.c(), C.int(btoi(mode)))
}

func (s SpaceBase) Cleanup() bool {
	return C.dSpaceGetCleanup(s.c()) != 0
}

func (s SpaceBase) SetManualCleanup(mode bool) {
	C.dSpaceSetManualCleanup(s.c(), C.int(btoi(mode)))
}

func (s SpaceBase) SetSublevel(sublevel int) {
	C.dSpaceSetSublevel(s.c(), C.int(sublevel))
}

func (s SpaceBase) Sublevel() int {
	return int(C.dSpaceGetSublevel(s.c()))
}

func (s SpaceBase) ManualCleanup() bool {
	return C.dSpaceGetManualCleanup(s.c()) != 0
}

func (s SpaceBase) Clean() {
	C.dSpaceClean(s.c())
}

func (s SpaceBase) Class() int {
	return int(C.dSpaceGetClass(s.c()))
}

func (s SpaceBase) Add(g Geom) {
	C.dSpaceAdd(s.c(), g.c())
}

func (s SpaceBase) Remove(g Geom) {
	C.dSpaceRemove(s.c(), g.c())
}

func (s SpaceBase) Query(g Geom) bool {
	return C.dSpaceQuery(s.c(), g.c()) != 0
}

func (s SpaceBase) NumGeoms(g Geom) int {
	return int(C.dSpaceGetNumGeoms(s.c()))
}

func (s SpaceBase) Geom(index int) Geom {
	return cToGeom(C.dSpaceGetGeom(s.c(), C.int(index)))
}

func (s SpaceBase) Collide(data interface{}, cb NearCallback) {
	cbData := &NearCallbackData{fn: cb, data: data}
	C.dSpaceCollide(s.c(), unsafe.Pointer(cbData),
		(*C.dNearCallback)(C.callNearCallback))
}

func (s SpaceBase) NewSphere(radius float64) Sphere {
	return cToGeom(C.dCreateSphere(s.c(), C.dReal(radius))).(Sphere)
}

func (s SpaceBase) NewBox(lens Vector3) Box {
	return cToGeom(C.dCreateBox(s.c(), C.dReal(lens[0]), C.dReal(lens[1]), C.dReal(lens[2]))).(Box)
}

func (s SpaceBase) NewPlane(params Vector4) Plane {
	return cToGeom(C.dCreatePlane(s.c(), C.dReal(params[0]), C.dReal(params[1]),
		C.dReal(params[2]), C.dReal(params[3]))).(Plane)
}

func (s SpaceBase) NewCapsule(radius, length float64) Capsule {
	return cToGeom(C.dCreateCapsule(s.c(), C.dReal(radius), C.dReal(length))).(Capsule)
}

func (s SpaceBase) NewCylinder(radius, length float64) Cylinder {
	return cToGeom(C.dCreateCylinder(s.c(), C.dReal(radius), C.dReal(length))).(Cylinder)
}

func (s SpaceBase) NewRay(length float64) Ray {
	return cToGeom(C.dCreateRay(s.c(), C.dReal(length))).(Ray)
}

func (s SpaceBase) NewHeightfield(data HeightfieldData, placeable bool) Heightfield {
	return cToGeom(C.dCreateHeightfield(s.c(), data.c(), C.int(btoi(placeable)))).(Heightfield)
}

func (s SpaceBase) NewSimpleSpace() SimpleSpace {
	return cToSpace(C.dSimpleSpaceCreate(s.c())).(SimpleSpace)
}

func (s SpaceBase) NewHashSpace() HashSpace {
	return cToSpace(C.dHashSpaceCreate(s.c())).(HashSpace)
}

func (s SpaceBase) NewQuadTreeSpace(center, extents Vector3, depth int) QuadTreeSpace {
	return cToSpace(C.dQuadTreeSpaceCreate(s.c(), (*C.dReal)(&center[0]),
		(*C.dReal)(&extents[0]), C.int(depth))).(QuadTreeSpace)
}

func (s SpaceBase) NewSweepAndPruneSpace(axisOrder int) SweepAndPruneSpace {
	return cToSpace(C.dSweepAndPruneSpaceCreate(s.c(), C.int(axisOrder))).(SweepAndPruneSpace)
}

// SimpleSpace

type SimpleSpace struct {
	SpaceBase
}

// HashSpace

type HashSpace struct {
	SpaceBase
}

func (s HashSpace) SetLevels(min, max int) {
	C.dHashSpaceSetLevels(s.c(), C.int(min), C.int(max))
}

func (s HashSpace) Levels() (int, int) {
	var min, max C.int
	C.dHashSpaceGetLevels(s.c(), &min, &max)
	return int(min), int(max)
}

// QuadTreeSpace

type QuadTreeSpace struct {
	SpaceBase
}

// SweepAndPruneSpace

type SweepAndPruneSpace struct {
	SpaceBase
}
