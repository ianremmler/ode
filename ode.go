package ode

// #cgo LDFLAGS: -lode
// #include <ode/ode.h>
import "C"

import (
	"unsafe"
)

const (
	ManualThreadCleanupIFlag = C.dInitFlagManualThreadCleanup
)

const (
	BasicDataAFlag     = C.dAllocateFlagBasicData
	CollisionDataAFlag = C.dAllocateFlagCollisionData
	AllAFlag           = C.dAllocateMaskAll
)

// for convenience
var (
	V3 = NewVector3
	V4 = NewVector4
	M3 = NewMatrix3
	M4 = NewMatrix4
	Q  = NewQuaternion
	BB = NewAABB
)

type NearCallback func(data interface{}, obj1, obj2 Geom)

type NearCallbackData struct {
	data interface{}
	fn   NearCallback
}

//export nearCallback
func nearCallback(data unsafe.Pointer, obj1, obj2 C.dGeomID) {
	cbData := (*NearCallbackData)(data)
	cbData.fn(cbData.data, cToGeom(obj1), cToGeom(obj2))
}

type Vector3 []float64
type Vector4 []float64
type Quaternion []float64
type AABB []float64
type Matrix3 [][]float64
type Matrix4 [][]float64

// round up to nearest multiple of 4 for alignment purposes (ode likes that)
func align4(n int) int {
	return (n + 3) &^ 3
}

func newVector(size int, vals []float64) []float64 {
	alignedSize := align4(size)
	v := make([]float64, size, alignedSize)
	copy(v, vals)
	return v
}

func NewVector3(vals ...float64) Vector3 {
	return Vector3(newVector(3, vals))
}

func NewVector4(vals ...float64) Vector4 {
	return Vector4(newVector(4, vals))
}

func NewQuaternion(vals ...float64) Quaternion {
	q := make(Quaternion, 4)
	copy(q, vals)
	return q
}

func NewAABB(vals ...float64) AABB {
	aabb := make(AABB, 6)
	copy(aabb, vals)
	return aabb
}

func newMatrix(size int, vals []float64) [][]float64 {
	mat := make([][]float64, size)
	alignedSize := align4(size)
	elts := make([]float64, alignedSize*size)
	for i := range mat {
		mat[i], elts = elts[:size], elts[alignedSize:]
		n := size
		if len(vals) < size {
			n = len(vals)
		}
		copy(mat[i], vals[:n])
		vals = vals[n:]
	}
	return mat
}

func NewMatrix3(vals ...float64) Matrix3 {
	return Matrix3(newMatrix(3, vals))
}

func NewMatrix4(vals ...float64) Matrix4 {
	return Matrix4(newMatrix(4, vals))
}

func cToVector3(a *C.dReal) Vector3 {
	vec := NewVector3()
	vec.fromC(a)
	return vec
}

func (v Vector3) fromC(c *C.dReal) {
	for i := range v {
		v[i] = float64(*c)
		c = (*C.dReal)(unsafe.Pointer(uintptr(unsafe.Pointer(c)) + unsafe.Sizeof(*c)))
	}
}

func (v Vector3) toC(c *C.dReal) {
	for i := range v {
		*c = C.dReal(v[i])
		c = (*C.dReal)(unsafe.Pointer(uintptr(unsafe.Pointer(c)) + unsafe.Sizeof(*c)))
	}
}

func (m Matrix3) fromC(c *C.dReal) {
	for i := range m {
		for j := 0; j < align4(len(m[i])); j++ {
			if j < len(m[i]) {
				m[i][j] = float64(*c)
			}
			c = (*C.dReal)(unsafe.Pointer(uintptr(unsafe.Pointer(c)) + unsafe.Sizeof(*c)))
		}
	}
}

func (m Matrix3) toC(c *C.dReal) {
	for i := range m {
		for j := 0; j < align4(len(m[i])); j++ {
			if j < len(m[i]) {
				*c = C.dReal(m[i][j])
			}
			c = (*C.dReal)(unsafe.Pointer(uintptr(unsafe.Pointer(c)) + unsafe.Sizeof(*c)))
		}
	}
}

func Init(initFlags, allocFlags int) {
	C.dInitODE2(C.uint(initFlags))
	C.dAllocateODEDataForThread(C.uint(allocFlags))
}

func Close() {
	C.dCloseODE()
}

func CleanupAllDataForThread() {
	C.dCleanupODEAllDataForThread()
}

func btoi(b bool) int {
	if b {
		return 1
	}
	return 0
}
