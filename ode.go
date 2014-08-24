// Package ode provide a Go interface to the Open Dynamics Engine library.
// See the ODE documentation for more information.
package ode

// #cgo LDFLAGS: -lode
// #include <ode/ode.h>
import "C"

import (
	"unsafe"
)

// Initialization flags
const (
	ManualThreadCleanupIFlag = C.dInitFlagManualThreadCleanup
)

// Allocation flags
const (
	BasicDataAFlag     = C.dAllocateFlagBasicData
	CollisionDataAFlag = C.dAllocateFlagCollisionData
	AllAFlag           = C.dAllocateMaskAll
)

// Short constructor aliases for convenience
var (
	V3 = NewVector3
	V4 = NewVector4
	M3 = NewMatrix3
	M4 = NewMatrix4
	Q  = NewQuaternion
	BB = NewAABB
)

// NearCallback is a callback type for handling potential object collisions.
type NearCallback func(data interface{}, obj1, obj2 Geom)

type nearCallbackData struct {
	data interface{}
	fn   NearCallback
}

//export nearCallback
func nearCallback(data unsafe.Pointer, obj1, obj2 C.dGeomID) {
	cbData := (*nearCallbackData)(data)
	cbData.fn(cbData.data, cToGeom(obj1), cToGeom(obj2))
}

// Vector3 represents a 3 component vector.
type Vector3 []float64

// Vector4 represents a 4 component vector.
type Vector4 []float64

// Quaternion represents a quaternion.
type Quaternion []float64

// AABB represents an axis-aligned bounding box.
type AABB []float64

// Matrix3 represents a 3x3 matrix.
type Matrix3 [][]float64

// Matrix4 represents a 4x4 matrix.
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

// NewVector3 returns a new Vector3 instance.
func NewVector3(vals ...float64) Vector3 {
	return Vector3(newVector(3, vals))
}

// NewVector4 returns a new Vector4 instance.
func NewVector4(vals ...float64) Vector4 {
	return Vector4(newVector(4, vals))
}

// NewQuaternion returns a new Quaternion instance.
func NewQuaternion(vals ...float64) Quaternion {
	q := make(Quaternion, 4)
	copy(q, vals)
	return q
}

// NewAABB returns a new AABB instance.
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

// NewMatrix3 returns a new Matrix3 instance.
func NewMatrix3(vals ...float64) Matrix3 {
	return Matrix3(newMatrix(3, vals))
}

// NewMatrix4 returns a new Matrix4 instance.
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

// Init initializes ODE.
func Init(initFlags, allocFlags int) {
	C.dInitODE2(C.uint(initFlags))
	C.dAllocateODEDataForThread(C.uint(allocFlags))
}

// Close releases ODE resources.
func Close() {
	C.dCloseODE()
}

// CleanupAllDataForThread manually releases ODE resources for the current thread.
func CleanupAllDataForThread() {
	C.dCleanupODEAllDataForThread()
}

func btoi(b bool) int {
	if b {
		return 1
	}
	return 0
}
