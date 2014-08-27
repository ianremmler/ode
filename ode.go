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

// Vector represents a vector.
type Vector []float64

func (v Vector) fromC(c *C.dReal) {
	for i := range v {
		v[i] = float64(*c)
		c = (*C.dReal)(unsafe.Pointer(uintptr(unsafe.Pointer(c)) + unsafe.Sizeof(*c)))
	}
}

func (v Vector) toC(c *C.dReal) {
	for i := range v {
		*c = C.dReal(v[i])
		c = (*C.dReal)(unsafe.Pointer(uintptr(unsafe.Pointer(c)) + unsafe.Sizeof(*c)))
	}
}

// Vector3 represents a 3 component vector.
type Vector3 Vector

func cToVector3(a *C.dReal) Vector3 {
	vec := NewVector3()
	Vector(vec).fromC(a)
	return vec
}

// NewVector3 returns a new Vector3 instance.
func NewVector3(vals ...float64) Vector3 {
	return Vector3(newVector(3, vals))
}

// Vector4 represents a 4 component vector.
type Vector4 Vector

// NewVector4 returns a new Vector4 instance.
func NewVector4(vals ...float64) Vector4 {
	return Vector4(newVector(4, vals))
}

// Quaternion represents a quaternion.
type Quaternion []float64

// NewQuaternion returns a new Quaternion instance.
func NewQuaternion(vals ...float64) Quaternion {
	q := make(Quaternion, 4)
	copy(q, vals)
	return q
}

// AABB represents an axis-aligned bounding box.
type AABB []float64

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

// Matrix represents a matrix.
type Matrix [][]float64

func (m Matrix) fromC(c *C.dReal) {
	for i := range m {
		for j := 0; j < align4(len(m[i])); j++ {
			if j < len(m[i]) {
				m[i][j] = float64(*c)
			}
			c = (*C.dReal)(unsafe.Pointer(uintptr(unsafe.Pointer(c)) + unsafe.Sizeof(*c)))
		}
	}
}

func (m Matrix) toC(c *C.dReal) {
	for i := range m {
		for j := 0; j < align4(len(m[i])); j++ {
			if j < len(m[i]) {
				*c = C.dReal(m[i][j])
			}
			c = (*C.dReal)(unsafe.Pointer(uintptr(unsafe.Pointer(c)) + unsafe.Sizeof(*c)))
		}
	}
}

// Matrix3 represents a 3x3 matrix.
type Matrix3 Matrix

// NewMatrix3 returns a new Matrix3 instance.
func NewMatrix3(vals ...float64) Matrix3 {
	return Matrix3(newMatrix(3, vals))
}

// Matrix4 represents a 4x4 matrix.
type Matrix4 Matrix

// NewMatrix4 returns a new Matrix4 instance.
func NewMatrix4(vals ...float64) Matrix4 {
	return Matrix4(newMatrix(4, vals))
}

// VertexList represents a list of 3D vertices.
type VertexList [][]float64

// NewVertexList returns a new VertexList instance.
func NewVertexList(size int, vals ...float64) VertexList {
	list := make([][]float64, size)
	elts := make([]float64, 3*size)
	for i := range list {
		list[i], elts = elts[:3], elts[3:]
		n := 3
		if len(vals) < 3 {
			n = len(vals)
		}
		copy(list[i], vals[:n])
		vals = vals[n:]
	}
	return VertexList(list)
}

// TriVertexIndexList represents a list of triangle vertex indices.
type TriVertexIndexList [][]uint32

// NewTriVertexIndexList returns a new TriVertexIndexList instance.
func NewTriVertexIndexList(size int, indices ...uint32) TriVertexIndexList {
	list := make([][]uint32, size)
	elts := make([]uint32, 3*size)
	for i := range list {
		list[i], elts = elts[:3], elts[3:]
		n := 3
		if len(indices) < 3 {
			n = len(indices)
		}
		copy(list[i], indices[:n])
		indices = indices[n:]
	}
	return TriVertexIndexList(list)
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
