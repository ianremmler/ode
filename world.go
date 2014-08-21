package ode

// #include <ode/ode.h>
import "C"

import (
	"unsafe"
)

var (
	worldData = map[World]interface{}{}
)

type World uintptr

func NewWorld() World {
	return cToWorld(C.dWorldCreate())
}

func cToWorld(c C.dWorldID) World {
	return World(unsafe.Pointer(c))
}

func (w World) c() C.dWorldID {
	return C.dWorldID(unsafe.Pointer(w))
}

func (w World) Destroy() {
	delete(worldData, w)
	C.dWorldDestroy(w.c())
}

func (w World) SetData(data interface{}) {
	worldData[w] = data
}

func (w World) Data() interface{} {
	return worldData[w]
}
func (w World) NewBody() Body {
	return cToBody(C.dBodyCreate(w.c()))
}

func (w World) SetGravity(grav Vector3) {
	C.dWorldSetGravity(w.c(), C.dReal(grav[0]), C.dReal(grav[1]), C.dReal(grav[2]))
}

func (w World) Gravity() Vector3 {
	grav := NewVector3()
	C.dWorldGetGravity(w.c(), (*C.dReal)(&grav[0]))
	return grav
}

func (w World) SetERP(erp float64) {
	C.dWorldSetERP(w.c(), C.dReal(erp))
}

func (w World) ERP() float64 {
	return float64(C.dWorldGetERP(w.c()))
}

func (w World) SetCFM(cfm float64) {
	C.dWorldSetCFM(w.c(), C.dReal(cfm))
}

func (w World) CFM() float64 {
	return float64(C.dWorldGetCFM(w.c()))
}

func (w World) SetStepIslandsProcessingMaxThreadCount(count int) {
	C.dWorldSetStepIslandsProcessingMaxThreadCount(w.c(), C.unsigned(count))
}

func (w World) StepIslandsProcessingMaxThreadCount() int {
	return int(C.dWorldGetStepIslandsProcessingMaxThreadCount(w.c()))
}

func (w World) UseSharedWorkingMemory(from World) bool {
	return C.dWorldUseSharedWorkingMemory(w.c(), from.c()) != 0
}

func (w World) CleanupWorkingMemory() {
	C.dWorldCleanupWorkingMemory(w.c())
}

// TODO dWorldSetStepMemoryReservationPolicy

// TODO dWorldSetStepMemoryManager

// TODO dWorldSetStepThreadingImplementation

func (w World) Step(stepSize float64) bool {
	return C.dWorldStep(w.c(), C.dReal(stepSize)) != 0
}

func (w World) QuickStep(stepSize float64) bool {
	return C.dWorldQuickStep(w.c(), C.dReal(stepSize)) != 0
}

func (w World) ImpulseToForce(stepSize float64, impulse Vector3) Vector3 {
	force := NewVector3()
	C.dWorldImpulseToForce(w.c(), C.dReal(stepSize),
		C.dReal(impulse[0]), C.dReal(impulse[1]), C.dReal(impulse[2]),
		(*C.dReal)(&force[0]))
	return force
}

func (w World) SetQuickStepNumIterations(num int) {
	C.dWorldSetQuickStepNumIterations(w.c(), C.int(num))
}

func (w World) QuickStepNumIterations() int {
	return int(C.dWorldGetQuickStepNumIterations(w.c()))
}

func (w World) SetQuickStepW(overRelaxation float64) {
	C.dWorldSetQuickStepW(w.c(), C.dReal(overRelaxation))
}

func (w World) QuickStepW() float64 {
	return float64(C.dWorldGetQuickStepW(w.c()))
}

func (w World) SetContactMaxCorrectingVel(overRelaxation float64) {
	C.dWorldSetContactMaxCorrectingVel(w.c(), C.dReal(overRelaxation))
}

func (w World) ContactMaxCorrectingVel() float64 {
	return float64(C.dWorldGetContactMaxCorrectingVel(w.c()))
}

func (w World) SetContactSurfaceLayer(depth float64) {
	C.dWorldSetContactSurfaceLayer(w.c(), C.dReal(depth))
}

func (w World) ContactSurfaceLayer() float64 {
	return float64(C.dWorldGetContactSurfaceLayer(w.c()))
}

func (w World) SetAutoDisableLinearThreshold(linearThreshold float64) {
	C.dWorldSetAutoDisableLinearThreshold(w.c(), C.dReal(linearThreshold))
}

func (w World) AutoDisableLinearThreshold() float64 {
	return float64(C.dWorldGetAutoDisableLinearThreshold(w.c()))
}

func (w World) SetAutoDisableAngularThreshold(angularThreshold float64) {
	C.dWorldSetAutoDisableAngularThreshold(w.c(), C.dReal(angularThreshold))
}

func (w World) AutoDisableAngularThreshold() float64 {
	return float64(C.dWorldGetAutoDisableAngularThreshold(w.c()))
}

func (w World) SetAutoAutoDisableAverageSamplesCount(averageSamplesCount bool) {
	C.dWorldSetAutoDisableAverageSamplesCount(w.c(), C.unsigned(btoi(averageSamplesCount)))
}

func (w World) AutoDisableAverageSamplesCount() bool {
	return C.dWorldGetAutoDisableAverageSamplesCount(w.c()) != 0
}

func (w World) SetAutoDisableSteps(steps int) {
	C.dWorldSetAutoDisableSteps(w.c(), C.int(steps))
}

func (w World) AutoDisableSteps() int {
	return int(C.dWorldGetAutoDisableSteps(w.c()))
}

func (w World) SetAutoDisableTime(time float64) {
	C.dWorldSetAutoDisableTime(w.c(), C.dReal(time))
}

func (w World) AutoDisableTime() float64 {
	return float64(C.dWorldGetAutoDisableTime(w.c()))
}

func (w World) SetAutoDisableFlag(doAutoDisable bool) {
	C.dWorldSetAutoDisableFlag(w.c(), C.int(btoi(doAutoDisable)))
}

func (w World) AutoAutoDisableFlag() bool {
	return C.dWorldGetAutoDisableFlag(w.c()) != 0
}

func (w World) SetLinearDamping(scale float64) {
	C.dWorldSetLinearDamping(w.c(), C.dReal(scale))
}

func (w World) LinearDamping() float64 {
	return float64(C.dWorldGetLinearDamping(w.c()))
}

func (w World) SetAngularDamping(scale float64) {
	C.dWorldSetAngularDamping(w.c(), C.dReal(scale))
}

func (w World) AngularDamping() float64 {
	return float64(C.dWorldGetAngularDamping(w.c()))
}

func (w World) SetLinearDampingThreshold(threshold float64) {
	C.dWorldSetLinearDampingThreshold(w.c(), C.dReal(threshold))
}

func (w World) LinearDampingThreshold() float64 {
	return float64(C.dWorldGetLinearDampingThreshold(w.c()))
}

func (w World) SetAngularDampingThreshold(threshold float64) {
	C.dWorldSetAngularDampingThreshold(w.c(), C.dReal(threshold))
}

func (w World) AngularDampingThreshold() float64 {
	return float64(C.dWorldGetAngularDampingThreshold(w.c()))
}

func (w World) SetMaxAngularSpeed(maxSpeed float64) {
	C.dWorldSetMaxAngularSpeed(w.c(), C.dReal(maxSpeed))
}

func (w World) MaxAngularSpeed() float64 {
	return float64(C.dWorldGetMaxAngularSpeed(w.c()))
}

// Joints

func (w World) NewBallJoint(group JointGroup) BallJoint {
	return cToJoint(C.dJointCreateBall(w.c(), group.c())).(BallJoint)
}

func (w World) NewHingeJoint(group JointGroup) HingeJoint {
	return cToJoint(C.dJointCreateHinge(w.c(), group.c())).(HingeJoint)
}

func (w World) NewSliderJoint(group JointGroup) SliderJoint {
	return cToJoint(C.dJointCreateSlider(w.c(), group.c())).(SliderJoint)
}

func (w World) NewContactJoint(group JointGroup, contact *Contact) ContactJoint {
	c := &C.dContact{}
	contact.toC(c)
	return cToJoint(C.dJointCreateContact(w.c(), group.c(), c)).(ContactJoint)
}

func (w World) NewUniversalJoint(group JointGroup) UniversalJoint {
	return cToJoint(C.dJointCreateUniversal(w.c(), group.c())).(UniversalJoint)
}

func (w World) NewHinge2Joint(group JointGroup) Hinge2Joint {
	return cToJoint(C.dJointCreateHinge2(w.c(), group.c())).(Hinge2Joint)
}

func (w World) NewFixedJoint(group JointGroup) FixedJoint {
	return cToJoint(C.dJointCreateFixed(w.c(), group.c())).(FixedJoint)
}

func (w World) NewNullJoint(group JointGroup) NullJoint {
	return cToJoint(C.dJointCreateNull(w.c(), group.c())).(NullJoint)
}

func (w World) NewAMotorJoint(group JointGroup) AMotorJoint {
	return cToJoint(C.dJointCreateAMotor(w.c(), group.c())).(AMotorJoint)
}

func (w World) NewLMotorJoint(group JointGroup) LMotorJoint {
	return cToJoint(C.dJointCreateLMotor(w.c(), group.c())).(LMotorJoint)
}

func (w World) NewPlane2DJoint(group JointGroup) Plane2DJoint {
	return cToJoint(C.dJointCreatePlane2D(w.c(), group.c())).(Plane2DJoint)
}

func (w World) NewPRJoint(group JointGroup) PRJoint {
	return cToJoint(C.dJointCreatePR(w.c(), group.c())).(PRJoint)
}

func (w World) NewPUJoint(group JointGroup) PUJoint {
	return cToJoint(C.dJointCreatePU(w.c(), group.c())).(PUJoint)
}

func (w World) NewPistonJoint(group JointGroup) PistonJoint {
	return cToJoint(C.dJointCreatePiston(w.c(), group.c())).(PistonJoint)
}

func (w World) NewDBallJoint(group JointGroup) BallJoint {
	return cToJoint(C.dJointCreateDBall(w.c(), group.c())).(BallJoint)
}

func (w World) NewDHingeJoint(group JointGroup) HingeJoint {
	return cToJoint(C.dJointCreateDHinge(w.c(), group.c())).(HingeJoint)
}

func (w World) NewTransmissionJoint(group JointGroup) HingeJoint {
	return cToJoint(C.dJointCreateTransmission(w.c(), group.c())).(HingeJoint)
}
