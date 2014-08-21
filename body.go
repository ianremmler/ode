package ode

// #include <ode/ode.h>
// extern void callMovedCallback(dBodyID body);
import "C"

import (
	"unsafe"
)

var (
	bodyData       = map[Body]interface{}{}
	movedCallbacks = map[Body]MovedCallback{}
)

type Body uintptr

type MovedCallback func(b Body)

//export movedCallback
func movedCallback(c C.dBodyID) {
	body := cToBody(c)
	if cb, ok := movedCallbacks[body]; ok {
		cb(body)
	}
}

func cToBody(c C.dBodyID) Body {
	return Body(unsafe.Pointer(c))
}

func (b Body) c() C.dBodyID {
	return C.dBodyID(unsafe.Pointer(b))
}

func (b Body) Destroy() {
	delete(bodyData, b)
	C.dBodyDestroy(b.c())
}

func (b Body) SetAutoDisableLinearThreshold(thresh float64) {
	C.dBodySetAutoDisableLinearThreshold(b.c(), C.dReal(thresh))
}

func (b Body) AutoDisableLinearThreshold() float64 {
	return float64(C.dBodyGetAutoDisableLinearThreshold(b.c()))
}

func (b Body) SetAutoDisableAngularThreshold(thresh float64) {
	C.dBodySetAutoDisableAngularThreshold(b.c(), C.dReal(thresh))
}

func (b Body) AutoDisableAngularThreshold() float64 {
	return float64(C.dBodyGetAutoDisableAngularThreshold(b.c()))
}

func (b Body) SetAutoAutoDisableAverageSamplesCount(count int) {
	C.dBodySetAutoDisableAverageSamplesCount(b.c(), C.uint(count))
}

func (b Body) AutoDisableAverageSamplesCount() int {
	return int(C.dBodyGetAutoDisableAverageSamplesCount(b.c()))
}

func (b Body) SetAutoDisableSteps(steps int) {
	C.dBodySetAutoDisableSteps(b.c(), C.int(steps))
}

func (b Body) AutoDisableSteps() int {
	return int(C.dBodyGetAutoDisableSteps(b.c()))
}

func (b Body) SetAutoDisableTime(time float64) {
	C.dBodySetAutoDisableTime(b.c(), C.dReal(time))
}

func (b Body) AutoDisableTime() float64 {
	return float64(C.dBodyGetAutoDisableTime(b.c()))
}

func (b Body) SetAutoDisableFlag(doAutoDisable bool) {
	C.dBodySetAutoDisableFlag(b.c(), C.int(btoi(doAutoDisable)))
}

func (b Body) AutoAutoDisableFlag() bool {
	return C.dBodyGetAutoDisableFlag(b.c()) != 0
}

func (b Body) SetAutoDisableDefaults() {
	C.dBodySetAutoDisableDefaults(b.c())
}

func (b Body) World() World {
	return cToWorld(C.dBodyGetWorld(b.c()))
}

func (b Body) Data() interface{} {
	return bodyData[b]
}

func (b Body) SetData(data interface{}) {
	bodyData[b] = data
}

func (b Body) SetPosition(pos Vector3) {
	C.dBodySetPosition(b.c(), C.dReal(pos[0]), C.dReal(pos[1]), C.dReal(pos[2]))
}

func (b Body) Position() Vector3 {
	pos := NewVector3()
	C.dBodyCopyPosition(b.c(), (*C.dReal)(&pos[0]))
	return pos
}

func (b Body) SetRotation(rot Matrix3) {
	C.dBodySetRotation(b.c(), (*C.dReal)(&rot[0][0]))
}

func (b Body) Rotation() Matrix3 {
	rot := NewMatrix3()
	C.dBodyCopyRotation(b.c(), (*C.dReal)(&rot[0][0]))
	return rot
}

func (b Body) SetQuaternion(quat Quaternion) {
	C.dBodySetQuaternion(b.c(), (*C.dReal)(&quat[0]))
}

func (b Body) Quaternion() Quaternion {
	quat := NewQuaternion()
	C.dBodyCopyQuaternion(b.c(), (*C.dReal)(&quat[0]))
	return quat
}

func (b Body) SetLinearVel(vel Vector3) {
	C.dBodySetLinearVel(b.c(), C.dReal(vel[0]), C.dReal(vel[1]), C.dReal(vel[2]))
}

func (b Body) LinearVel() Vector3 {
	return cToVector3(C.dBodyGetLinearVel(b.c()))
}

func (b Body) SetAngularVel(vel Vector3) {
	C.dBodySetAngularVel(b.c(), C.dReal(vel[0]), C.dReal(vel[1]), C.dReal(vel[2]))
}

func (b Body) AngularVel() Vector3 {
	return cToVector3(C.dBodyGetAngularVel(b.c()))
}

func (b Body) SetMass(mass *Mass) {
	c := &C.dMass{}
	mass.toC(c)
	C.dBodySetMass(b.c(), c)
}

func (b Body) Mass() *Mass {
	mass := NewMass()
	c := &C.dMass{}
	C.dBodyGetMass(b.c(), c)
	mass.fromC(c)
	return mass
}

func (b Body) SetForce(force Vector3) {
	C.dBodySetForce(b.c(), C.dReal(force[0]), C.dReal(force[1]), C.dReal(force[2]))
}

func (b Body) Force() Vector3 {
	return cToVector3(C.dBodyGetForce(b.c()))
}

func (b Body) SetTorque(torque Vector3) {
	C.dBodySetTorque(b.c(), C.dReal(torque[0]), C.dReal(torque[1]), C.dReal(torque[2]))
}

func (b Body) Torque() Vector3 {
	return cToVector3(C.dBodyGetTorque(b.c()))
}

func (b Body) RelPointPos(pt Vector3) Vector3 {
	pos := NewVector3()
	C.dBodyGetRelPointPos(b.c(), C.dReal(pt[0]), C.dReal(pt[1]), C.dReal(pt[2]), (*C.dReal)(&pos[0]))
	return pos
}

func (b Body) RelPointVel(pt Vector3) Vector3 {
	vel := NewVector3()
	C.dBodyGetRelPointVel(b.c(), C.dReal(pt[0]), C.dReal(pt[1]), C.dReal(pt[2]), (*C.dReal)(&vel[0]))
	return vel
}

func (b Body) PointVel(pt Vector3) Vector3 {
	vel := NewVector3()
	C.dBodyGetPointVel(b.c(), C.dReal(pt[0]), C.dReal(pt[1]), C.dReal(pt[2]), (*C.dReal)(&vel[0]))
	return vel
}

func (b Body) PosRelPoint(pos Vector3) Vector3 {
	pt := NewVector3()
	C.dBodyGetPosRelPoint(b.c(), C.dReal(pos[0]), C.dReal(pos[1]), C.dReal(pos[2]), (*C.dReal)(&pt[0]))
	return pt
}

func (b Body) VectorToWorld(vec Vector3) Vector3 {
	wld := NewVector3()
	C.dBodyVectorToWorld(b.c(), C.dReal(vec[0]), C.dReal(vec[1]), C.dReal(vec[2]), (*C.dReal)(&wld[0]))
	return wld
}

func (b Body) VectorFromWorld(wld Vector3) Vector3 {
	vec := NewVector3()
	C.dBodyVectorFromWorld(b.c(), C.dReal(wld[0]), C.dReal(wld[1]), C.dReal(wld[2]), (*C.dReal)(&vec[0]))
	return vec
}

func (b Body) SetFiniteRotationMode(mode bool) {
	C.dBodySetFiniteRotationMode(b.c(), C.int(btoi(mode)))
}

func (b Body) FiniteRotationMode() bool {
	return C.dBodyGetFiniteRotationMode(b.c()) != 0
}

func (b Body) SetFiniteRotationAxis(axis Vector3) {
	C.dBodySetFiniteRotationAxis(b.c(), C.dReal(axis[0]), C.dReal(axis[1]), C.dReal(axis[2]))
}

func (b Body) FiniteRotationAxis() Vector3 {
	axis := NewVector3()
	C.dBodyGetFiniteRotationAxis(b.c(), (*C.dReal)(&axis[0]))
	return axis
}

func (b Body) NumJoints() int {
	return int(C.dBodyGetNumJoints(b.c()))
}

func (b Body) Joint(index int) Joint {
	return cToJoint(C.dBodyGetJoint(b.c(), C.int(index)))
}

func (b Body) SetDynamic() {
	C.dBodySetDynamic(b.c())
}

func (b Body) SetKinematic() {
	C.dBodySetKinematic(b.c())
}

func (b Body) SetGravityMode(mode bool) {
	C.dBodySetGravityMode(b.c(), C.int(btoi(mode)))
}

func (b Body) SetMovedCallback(cb MovedCallback) {
	if cb == nil {
		C.dBodySetMovedCallback(b.c(), (*[0]byte)(nil)) // clear callback
		delete(movedCallbacks, b)
	} else {
		movedCallbacks[b] = cb
		C.dBodySetMovedCallback(b.c(), (*[0]byte)(unsafe.Pointer(C.callMovedCallback)))
	}
}

func (b Body) GravityMode() bool {
	return C.dBodyGetGravityMode(b.c()) != 0
}

func (b Body) FirstGeom() Geom {
	return cToGeom(C.dBodyGetFirstGeom(b.c()))
}

func (b Body) SetDampingDefaults() {
	C.dBodySetDampingDefaults(b.c())
}

func (b Body) SetLinearDamping(scale float64) {
	C.dBodySetLinearDamping(b.c(), C.dReal(scale))
}

func (b Body) LinearDamping() float64 {
	return float64(C.dBodyGetLinearDamping(b.c()))
}

func (b Body) SetAngularDamping(scale float64) {
	C.dBodySetAngularDamping(b.c(), C.dReal(scale))
}

func (b Body) AngularDamping() float64 {
	return float64(C.dBodyGetAngularDamping(b.c()))
}

func (b Body) SetLinearDampingThreshold(threshold float64) {
	C.dBodySetLinearDampingThreshold(b.c(), C.dReal(threshold))
}

func (b Body) LinearDampingThreshold() float64 {
	return float64(C.dBodyGetLinearDampingThreshold(b.c()))
}

func (b Body) SetAngularDampingThreshold(threshold float64) {
	C.dBodySetAngularDampingThreshold(b.c(), C.dReal(threshold))
}

func (b Body) AngularDampingThreshold() float64 {
	return float64(C.dBodyGetAngularDampingThreshold(b.c()))
}

func (b Body) SetMaxAngularSpeed(maxSpeed float64) {
	C.dBodySetMaxAngularSpeed(b.c(), C.dReal(maxSpeed))
}

func (b Body) MaxAngularSpeed() float64 {
	return float64(C.dBodyGetMaxAngularSpeed(b.c()))
}

func (b Body) SetGyroscopicMode(enabled bool) {
	C.dBodySetGyroscopicMode(b.c(), C.int(btoi(enabled)))
}

func (b Body) GyroscopicMode() bool {
	return C.dBodyGetGyroscopicMode(b.c()) != 0
}

func (b Body) Connected(other Body) bool {
	return C.dAreConnected(b.c(), other.c()) != 0
}

func (b Body) ConnectedExcluding(other Body, jointType int) bool {
	return C.dAreConnectedExcluding(b.c(), other.c(), C.int(jointType)) != 0
}

func (b Body) AddForce(force Vector3) {
	C.dBodyAddForce(b.c(), C.dReal(force[0]), C.dReal(force[1]), C.dReal(force[2]))
}

func (b Body) AddRelForce(force Vector3) {
	C.dBodyAddRelForce(b.c(), C.dReal(force[0]), C.dReal(force[1]), C.dReal(force[2]))
}

func (b Body) AddTorque(torque Vector3) {
	C.dBodyAddTorque(b.c(), C.dReal(torque[0]), C.dReal(torque[1]), C.dReal(torque[2]))
}

func (b Body) AddRelTorque(torque Vector3) {
	C.dBodyAddRelTorque(b.c(), C.dReal(torque[0]), C.dReal(torque[1]), C.dReal(torque[2]))
}

func (b Body) AddForceAtPos(force, pos Vector3) {
	C.dBodyAddForceAtPos(b.c(), C.dReal(force[0]), C.dReal(force[1]), C.dReal(force[2]),
		C.dReal(pos[0]), C.dReal(pos[1]), C.dReal(pos[2]))
}

func (b Body) AddForceAtRelPos(force, pos Vector3) {
	C.dBodyAddForceAtRelPos(b.c(), C.dReal(force[0]), C.dReal(force[1]), C.dReal(force[2]),
		C.dReal(pos[0]), C.dReal(pos[1]), C.dReal(pos[2]))
}

func (b Body) AddRelForceAtPos(force, pos Vector3) {
	C.dBodyAddRelForceAtPos(b.c(), C.dReal(force[0]), C.dReal(force[1]), C.dReal(force[2]),
		C.dReal(pos[0]), C.dReal(pos[1]), C.dReal(pos[2]))
}

func (b Body) AddRelForceAtRelPos(force, pos Vector3) {
	C.dBodyAddRelForceAtRelPos(b.c(), C.dReal(force[0]), C.dReal(force[1]), C.dReal(force[2]),
		C.dReal(pos[0]), C.dReal(pos[1]), C.dReal(pos[2]))
}

func (b Body) SetEnabled(isEnabled bool) {
	if isEnabled {
		C.dBodyEnable(b.c())
	} else {
		C.dBodyDisable(b.c())
	}
}

func (b Body) Enabled() bool {
	return bool(C.dBodyIsEnabled(b.c()) != 0)
}

func (b Body) Kinematic() bool {
	return bool(C.dBodyIsKinematic(b.c()) != 0)
}

func (b Body) ConnectingJoint(other Body) Joint {
	return cToJoint(C.dConnectingJoint(b.c(), other.c()))
}

func (b Body) ConnectingJointList(other Body) []Joint {
	jointList := []Joint{}
	for i := 0; i < b.NumJoints(); i++ {
		joint := b.Joint(i)
		for j := 0; j < joint.NumBodies(); j++ {
			if body := joint.Body(j); body == other {
				jointList = append(jointList, joint)
				break
			}
		}
	}
	return jointList
}
