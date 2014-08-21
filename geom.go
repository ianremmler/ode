package ode

// #include <ode/ode.h>
// extern void callNearCallback(void *data, dGeomID obj1, dGeomID obj2);
import "C"

import (
	"unsafe"
)

const (
	SphereClass             = C.dSphereClass
	BoxClass                = C.dBoxClass
	CapsuleClass            = C.dCapsuleClass
	CylinderClass           = C.dCylinderClass
	PlaneClass              = C.dPlaneClass
	RayClass                = C.dRayClass
	ConvexClass             = C.dConvexClass
	TriMeshClass            = C.dTriMeshClass
	HeightfieldClass        = C.dHeightfieldClass
	SimpleSpaceClass        = C.dSimpleSpaceClass
	HashSpaceClass          = C.dHashSpaceClass
	SweepAndPruneSpaceClass = C.dSweepAndPruneSpaceClass
	QuadTreeSpaceClass      = C.dQuadTreeSpaceClass

	NumClasses = C.dGeomNumClasses

	MaxUserClasses = C.dMaxUserClasses
	FirstUserClass = C.dFirstUserClass
	LastUserClass  = C.dLastUserClass

	FirstSpaceClass = C.dFirstSpaceClass
	LastSpaceClass  = C.dLastSpaceClass
)

var (
	geomData = map[Geom]interface{}{}
)

type Geom interface {
	c() C.dGeomID
	Destroy()
	SetData(data interface{})
	Data() interface{}
	SetBody(body Body)
	Body() Body
	SetPosition(pos Vector3)
	Position() Vector3
	SetRotation(rot Matrix3)
	Rotation() Matrix3
	SetQuaternion(quat Quaternion)
	Quaternion() Quaternion
	AABB() AABB
	IsSpace() bool
	Space() Space
	Class() int
	SetCategoryBits(bits int)
	SetCollideBits(bits int)
	CategoryBits() int
	CollideBits() int
	SetEnabled(isEnabled bool)
	Enabled() bool
	RelPointPos(pt Vector3) Vector3
	PosRelPoint(pos Vector3) Vector3
	VectorToWorld(vec Vector3) Vector3
	VectorFromWorld(wld Vector3) Vector3
	OffsetPosition() Vector3
	SetOffsetPosition(pos Vector3)
	OffsetRotation() Matrix3
	SetOffsetRotation(rot Matrix3)
	OffsetQuaternion() Quaternion
	SetOffsetQuaternion(quat Quaternion)
	SetOffsetWorldPosition(pos Vector3)
	SetOffsetWorldRotation(rot Matrix3)
	SetOffsetWorldQuaternion(quat Quaternion)
	ClearOffset()
	IsOffset() bool
	Collide(other Geom, maxContacts uint16, flags int) []ContactGeom
	Collide2(other Geom, data interface{}, cb NearCallback)
	Next() Geom
}

type GeomBase uintptr

func cToGeom(c C.dGeomID) Geom {
	base := GeomBase(unsafe.Pointer(c))
	var g Geom
	switch int(C.dGeomGetClass(c)) {
	case SphereClass:
		g = Sphere{base}
	case BoxClass:
		g = Box{base}
	case CapsuleClass:
		g = Capsule{base}
	case CylinderClass:
		g = Cylinder{base}
	case PlaneClass:
		g = Plane{base}
	case RayClass:
		g = Ray{base}
	// case ConvexClass:
	// g = &Convex{base}
	// case TriMeshClass:
	// g = &TriMesh{base}
	case HeightfieldClass:
		g = Heightfield{base}
	default:
		g = base
	}
	return g
}

func (g GeomBase) c() C.dGeomID {
	return C.dGeomID(unsafe.Pointer(g))
}

func (g GeomBase) Destroy() {
	delete(geomData, g)
	C.dGeomDestroy(g.c())
}

func (g GeomBase) SetData(data interface{}) {
	geomData[g] = data
}

func (g GeomBase) Data() interface{} {
	return geomData[g]
}

func (g GeomBase) Body() Body {
	return cToBody(C.dGeomGetBody(g.c()))
}

func (g GeomBase) SetBody(body Body) {
	C.dGeomSetBody(g.c(), body.c())
}

func (g GeomBase) SetPosition(pos Vector3) {
	C.dGeomSetPosition(g.c(), C.dReal(pos[0]), C.dReal(pos[1]), C.dReal(pos[2]))
}

func (g GeomBase) Position() Vector3 {
	pos := NewVector3()
	C.dGeomCopyPosition(g.c(), (*C.dReal)(&pos[0]))
	return pos
}

func (g GeomBase) SetRotation(rot Matrix3) {
	C.dGeomSetRotation(g.c(), (*C.dReal)(&rot[0][0]))
}

func (g GeomBase) Rotation() Matrix3 {
	rot := NewMatrix3()
	C.dGeomCopyRotation(g.c(), (*C.dReal)(&rot[0][0]))
	return rot
}

func (g GeomBase) SetQuaternion(quat Quaternion) {
	C.dGeomSetQuaternion(g.c(), (*C.dReal)(&quat[0]))
}

func (g GeomBase) Quaternion() Quaternion {
	quat := NewQuaternion()
	C.dGeomGetQuaternion(g.c(), (*C.dReal)(&quat[0]))
	return quat
}

func (g GeomBase) AABB() AABB {
	aabb := NewAABB()
	C.dGeomGetAABB(g.c(), (*C.dReal)(&aabb[0]))
	return aabb
}

func (g GeomBase) IsSpace() bool {
	return C.dGeomIsSpace(g.c()) != 0
}

func (g GeomBase) Space() Space {
	return cToSpace(C.dGeomGetSpace(g.c()))
}

func (g GeomBase) Class() int {
	return int(C.dGeomGetClass(g.c()))
}

func (g GeomBase) SetCategoryBits(bits int) {
	C.dGeomSetCategoryBits(g.c(), C.ulong(bits))
}

func (g GeomBase) CategoryBits() int {
	return int(C.dGeomGetCategoryBits(g.c()))
}

func (g GeomBase) SetCollideBits(bits int) {
	C.dGeomSetCollideBits(g.c(), C.ulong(bits))
}

func (g GeomBase) CollideBits() int {
	return int(C.dGeomGetCollideBits(g.c()))
}

func (g GeomBase) SetEnabled(isEnabled bool) {
	if isEnabled {
		C.dGeomEnable(g.c())
	} else {
		C.dGeomDisable(g.c())
	}
}

func (g GeomBase) Enabled() bool {
	return bool(C.dGeomIsEnabled(g.c()) != 0)
}

// TODO dGeomLowLevelControl

func (g GeomBase) RelPointPos(pt Vector3) Vector3 {
	pos := NewVector3()
	C.dGeomGetRelPointPos(g.c(), C.dReal(pt[0]), C.dReal(pt[1]), C.dReal(pt[2]), (*C.dReal)(&pos[0]))
	return pos
}

func (g GeomBase) PosRelPoint(pos Vector3) Vector3 {
	pt := NewVector3()
	C.dGeomGetPosRelPoint(g.c(), C.dReal(pos[0]), C.dReal(pos[1]), C.dReal(pos[2]), (*C.dReal)(&pt[0]))
	return pt
}

func (g GeomBase) VectorToWorld(vec Vector3) Vector3 {
	wld := NewVector3()
	C.dGeomVectorToWorld(g.c(), C.dReal(vec[0]), C.dReal(vec[1]), C.dReal(vec[2]), (*C.dReal)(&wld[0]))
	return wld
}

func (g GeomBase) VectorFromWorld(wld Vector3) Vector3 {
	vec := NewVector3()
	C.dGeomVectorFromWorld(g.c(), C.dReal(wld[0]), C.dReal(wld[1]), C.dReal(wld[2]), (*C.dReal)(&vec[0]))
	return vec
}

func (g GeomBase) SetOffsetPosition(pos Vector3) {
	C.dGeomSetOffsetPosition(g.c(), C.dReal(pos[0]), C.dReal(pos[1]), C.dReal(pos[2]))
}

func (g GeomBase) OffsetPosition() Vector3 {
	pos := NewVector3()
	C.dGeomCopyOffsetPosition(g.c(), (*C.dReal)(&pos[0]))
	return pos
}

func (g GeomBase) SetOffsetRotation(rot Matrix3) {
	C.dGeomSetOffsetRotation(g.c(), (*C.dReal)(&rot[0][0]))
}

func (g GeomBase) OffsetRotation() Matrix3 {
	rot := NewMatrix3()
	C.dGeomCopyOffsetRotation(g.c(), (*C.dReal)(&rot[0][0]))
	return rot
}

func (g GeomBase) SetOffsetQuaternion(quat Quaternion) {
	C.dGeomSetOffsetQuaternion(g.c(), (*C.dReal)(&quat[0]))
}

func (g GeomBase) OffsetQuaternion() Quaternion {
	quat := NewQuaternion()
	C.dGeomGetOffsetQuaternion(g.c(), (*C.dReal)(&quat[0]))
	return quat
}

func (g GeomBase) SetOffsetWorldPosition(pos Vector3) {
	C.dGeomSetOffsetWorldPosition(g.c(), C.dReal(pos[0]), C.dReal(pos[1]), C.dReal(pos[2]))
}

func (g GeomBase) SetOffsetWorldRotation(rot Matrix3) {
	C.dGeomSetOffsetWorldRotation(g.c(), (*C.dReal)(&rot[0][0]))
}

func (g GeomBase) SetOffsetWorldQuaternion(quat Quaternion) {
	C.dGeomSetOffsetWorldQuaternion(g.c(), (*C.dReal)(&quat[0]))
}

func (g GeomBase) ClearOffset() {
	C.dGeomClearOffset(g.c())
}

func (g GeomBase) IsOffset() bool {
	return C.dGeomIsOffset(g.c()) != 0
}

func (g GeomBase) Collide(other Geom, maxContacts uint16, flags int) []ContactGeom {
	cts := make([]C.dContactGeom, maxContacts)
	numCts := int(C.dCollide(g.c(), other.c(), C.int(int(maxContacts)|flags), &cts[0],
		C.int(unsafe.Sizeof(cts[0]))))
	contacts := make([]ContactGeom, numCts)
	for i := range contacts {
		contacts[i] = *NewContactGeom()
		contacts[i].fromC(&cts[i])
	}
	return contacts
}

func (g GeomBase) Collide2(other Geom, data interface{}, cb NearCallback) {
	cbData := &NearCallbackData{fn: cb, data: data}
	C.dSpaceCollide2(g.c(), other.c(), unsafe.Pointer(cbData),
		(*C.dNearCallback)(C.callNearCallback))
}

func (g GeomBase) Next() Geom {
	return cToGeom(C.dBodyGetNextGeom(g.c()))
}

// Sphere

type Sphere struct {
	GeomBase
}

func (s Sphere) SetRadius(radius float64) {
	C.dGeomSphereSetRadius(s.c(), C.dReal(radius))
}

func (s Sphere) Radius() float64 {
	return float64(C.dGeomSphereGetRadius(s.c()))
}

func (s Sphere) SpherePointDepth(pt Vector3) float64 {
	return float64(C.dGeomSpherePointDepth(s.c(), C.dReal(pt[0]), C.dReal(pt[1]), C.dReal(pt[2])))
}

// TODO ??? func (g Geom) NewConvex...

// TODO ??? func (g Geom) SetConvex...

// Box

type Box struct {
	GeomBase
}

func (b Box) SetLengths(lens Vector3) {
	C.dGeomBoxSetLengths(b.c(), C.dReal(lens[0]), C.dReal(lens[1]), C.dReal(lens[2]))
}

func (b Box) Lengths() Vector3 {
	lens := NewVector3()
	C.dGeomBoxGetLengths(b.c(), (*C.dReal)(&lens[0]))
	return lens
}

func (b Box) PointDepth(pt Vector3) float64 {
	return float64(C.dGeomBoxPointDepth(b.c(), C.dReal(pt[0]), C.dReal(pt[1]), C.dReal(pt[2])))
}

// Plane

type Plane struct {
	GeomBase
}

func (p Plane) SetParams(params Vector4) {
	C.dGeomPlaneSetParams(p.c(), C.dReal(params[0]), C.dReal(params[1]), C.dReal(params[2]), C.dReal(params[3]))
}

func (p Plane) Params() Vector4 {
	params := NewVector4()
	C.dGeomPlaneGetParams(p.c(), (*C.dReal)(&params[0]))
	return params
}

func (p Plane) PointDepth(pt Vector3) float64 {
	return float64(C.dGeomPlanePointDepth(p.c(), C.dReal(pt[0]), C.dReal(pt[1]), C.dReal(pt[2])))
}

// Capsule

type Capsule struct {
	GeomBase
}

func (c Capsule) SetParams(radius, length float64) {
	C.dGeomCapsuleSetParams(c.c(), C.dReal(radius), C.dReal(length))
}

func (c Capsule) Params() (float64, float64) {
	var radius, length float64
	C.dGeomCapsuleGetParams(c.c(), (*C.dReal)(&radius), (*C.dReal)(&length))
	return radius, length
}

func (c Capsule) PointDepth(pt Vector3) float64 {
	return float64(C.dGeomCapsulePointDepth(c.c(), C.dReal(pt[0]), C.dReal(pt[1]), C.dReal(pt[2])))
}

// Cylinder

type Cylinder struct {
	GeomBase
}

func (c Cylinder) SetParams(radius, length float64) {
	C.dGeomCylinderSetParams(c.c(), C.dReal(radius), C.dReal(length))
}

func (c Cylinder) Params() (float64, float64) {
	var radius, length float64
	C.dGeomCylinderGetParams(c.c(), (*C.dReal)(&radius), (*C.dReal)(&length))
	return radius, length
}

// Ray

type Ray struct {
	GeomBase
}

func (r Ray) SetPosDir(pos, dir Vector3) {
	C.dGeomRaySet(r.c(), C.dReal(pos[0]), C.dReal(pos[1]), C.dReal(pos[2]),
		C.dReal(dir[0]), C.dReal(dir[1]), C.dReal(dir[2]))
}

func (r Ray) PosDir() (Vector3, Vector3) {
	pos, dir := NewVector3(), NewVector3()
	C.dGeomRayGet(r.c(), (*C.dReal)(&pos[0]), (*C.dReal)(&dir[0]))
	return pos, dir
}

func (r Ray) SetLength(length float64) {
	C.dGeomRaySetLength(r.c(), C.dReal(length))
}

func (r Ray) Length() float64 {
	return float64(C.dGeomRayGetLength(r.c()))
}

func (r Ray) SetFirstContact(firstContact bool) {
	C.dGeomRaySetFirstContact(r.c(), C.int(btoi(firstContact)))
}

func (r Ray) FirstContact() bool {
	return C.dGeomRayGetFirstContact(r.c()) != 0
}

func (r Ray) SetBackfaceCull(backfaceCull bool) {
	C.dGeomRaySetBackfaceCull(r.c(), C.int(btoi(backfaceCull)))
}

func (r Ray) BackfaceCull() bool {
	return C.dGeomRayGetBackfaceCull(r.c()) != 0
}

func (r Ray) SetClosestHit(closestHit bool) {
	C.dGeomRaySetClosestHit(r.c(), C.int(btoi(closestHit)))
}

func (r Ray) ClosestHit() bool {
	return C.dGeomRayGetClosestHit(r.c()) != 0
}

// Heightfield

type Heightfield struct {
	GeomBase
}
