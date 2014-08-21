package ode

// #include <ode/ode.h>
import "C"

import (
	"unsafe"
)

const (
	BallJointType         = C.dJointTypeBall
	HingeJointType        = C.dJointTypeHinge
	SliderJointType       = C.dJointTypeSlider
	ContactJointType      = C.dJointTypeContact
	UniversalJointType    = C.dJointTypeUniversal
	Hinge2JointType       = C.dJointTypeHinge2
	FixedJointType        = C.dJointTypeFixed
	NullJointType         = C.dJointTypeNull
	AMotorJointType       = C.dJointTypeAMotor
	LMotorJointType       = C.dJointTypeLMotor
	Plane2DJointType      = C.dJointTypePlane2D
	PRJointType           = C.dJointTypePR
	PUJointType           = C.dJointTypePU
	PistonJointType       = C.dJointTypePiston
	DBallJointType        = C.dJointTypeDBall
	DHingeJointType       = C.dJointTypeDHinge
	TransmissionJointType = C.dJointTypeTransmission
)

const (
	LoStopJtParam        = C.dParamLoStop
	HiStopJtParam        = C.dParamHiStop
	VelJtParam           = C.dParamVel
	LoVelJtParam         = C.dParamLoVel
	HiVelJtParam         = C.dParamHiVel
	FMaxJtParam          = C.dParamFMax
	FudgeFactorJtParam   = C.dParamFudgeFactor
	BounceJtParam        = C.dParamBounce
	CFMJtParam           = C.dParamCFM
	StopERPJtParam       = C.dParamStopERP
	StopCFMJtParam       = C.dParamStopCFM
	SuspensionERPJtParam = C.dParamSuspensionERP
	SuspensionCFMJtParam = C.dParamSuspensionCFM
	ERPJtParam           = C.dParamERP

	NumJtParams = C.dParamsInGroup

	JtParamGroup1         = C.dParamGroup1
	LoStopJtParam1        = C.dParamLoStop1
	HiStopJtParam1        = C.dParamHiStop1
	VelJtParam1           = C.dParamVel1
	LoVelJtParam1         = C.dParamLoVel1
	HiVelJtParam1         = C.dParamHiVel1
	FMaxJtParam1          = C.dParamFMax1
	FudgeFactorJtParam1   = C.dParamFudgeFactor1
	BounceJtParam1        = C.dParamBounce1
	CFMJtParam1           = C.dParamCFM1
	StopERPJtParam1       = C.dParamStopERP1
	StopCFMJtParam1       = C.dParamStopCFM1
	SuspensionERPJtParam1 = C.dParamSuspensionERP1
	SuspensionCFMJtParam1 = C.dParamSuspensionCFM1
	ERPJtParam1           = C.dParamERP1

	JtParamGroup2         = C.dParamGroup2
	LoStopJtParam2        = C.dParamLoStop2
	HiStopJtParam2        = C.dParamHiStop2
	VelJtParam2           = C.dParamVel2
	LoVelJtParam2         = C.dParamLoVel2
	HiVelJtParam2         = C.dParamHiVel2
	FMaxJtParam2          = C.dParamFMax2
	FudgeFactorJtParam2   = C.dParamFudgeFactor2
	BounceJtParam2        = C.dParamBounce2
	CFMJtParam2           = C.dParamCFM2
	StopERPJtParam2       = C.dParamStopERP2
	StopCFMJtParam2       = C.dParamStopCFM2
	SuspensionERPJtParam2 = C.dParamSuspensionERP2
	SuspensionCFMJtParam2 = C.dParamSuspensionCFM2
	ERPJtParam2           = C.dParamERP2

	JtParamGroup3         = C.dParamGroup3
	LoStopJtParam3        = C.dParamLoStop3
	HiStopJtParam3        = C.dParamHiStop3
	VelJtParam3           = C.dParamVel3
	LoVelJtParam3         = C.dParamLoVel3
	HiVelJtParam3         = C.dParamHiVel3
	FMaxJtParam3          = C.dParamFMax3
	FudgeFactorJtParam3   = C.dParamFudgeFactor3
	BounceJtParam3        = C.dParamBounce3
	CFMJtParam3           = C.dParamCFM3
	StopERPJtParam3       = C.dParamStopERP3
	StopCFMJtParam3       = C.dParamStopCFM3
	SuspensionERPJtParam3 = C.dParamSuspensionERP3
	SuspensionCFMJtParam3 = C.dParamSuspensionCFM3
	ERPJtParam3           = C.dParamERP3
)

var (
	jointData = map[Joint]interface{}{}
)

// JointFeedback

type JointFeedback struct {
	F1 Vector3 // force applied to body 1
	T1 Vector3 // torque applied to body 1
	F2 Vector3 // force applied to body 2
	T2 Vector3 // torque applied to body 2
}

func (f *JointFeedback) fromC(c *C.dJointFeedback) {
	f.F1.fromC(&c.f1[0])
	f.T1.fromC(&c.t1[0])
	f.F2.fromC(&c.f2[0])
	f.T2.fromC(&c.t2[0])
}

func (f *JointFeedback) toC(c *C.dJointFeedback) {
	f.F1.toC((*C.dReal)(&c.f1[0]))
	f.T1.toC((*C.dReal)(&c.t1[0]))
	f.F2.toC((*C.dReal)(&c.f2[0]))
	f.T2.toC((*C.dReal)(&c.t2[0]))
}

// JointGroup

type JointGroup uintptr

func NewJointGroup(maxJoints int) JointGroup {
	return cToJointGroup(C.dJointGroupCreate(C.int(maxJoints)))
}

func cToJointGroup(c C.dJointGroupID) JointGroup {
	return JointGroup(unsafe.Pointer(c))
}

func (g JointGroup) c() C.dJointGroupID {
	return C.dJointGroupID(unsafe.Pointer(g))
}

func (g JointGroup) Destroy() {
	C.dJointGroupDestroy(g.c())
}

func (g JointGroup) Empty() {
	C.dJointGroupEmpty(g.c())
}

// Joint

type Joint interface {
	c() C.dJointID
	Destroy()
	SetData(data interface{})
	Data() interface{}
	NumBodies() int
	Attach(body1, body2 Body)
	SetEnabled(isEnabled bool)
	Enabled() bool
	Type() int
	Body(index int) Body
	SetFeedback(f *JointFeedback)
	Feedback() *JointFeedback
}

// JointBase

type JointBase uintptr

func cToJoint(c C.dJointID) Joint {
	base := JointBase(unsafe.Pointer(c))
	var j Joint
	switch int(C.dJointGetType(c)) {
	case BallJointType:
		j = BallJoint{base}
	case HingeJointType:
		j = HingeJoint{base}
	case SliderJointType:
		j = SliderJoint{base}
	case ContactJointType:
		j = ContactJoint{base}
	case UniversalJointType:
		j = UniversalJoint{base}
	case Hinge2JointType:
		j = Hinge2Joint{base}
	case FixedJointType:
		j = FixedJoint{base}
	case NullJointType:
		j = NullJoint{base}
	case AMotorJointType:
		j = AMotorJoint{base}
	case LMotorJointType:
		j = LMotorJoint{base}
	case Plane2DJointType:
		j = Plane2DJoint{base}
	case PRJointType:
		j = PRJoint{base}
	case PUJointType:
		j = PUJoint{base}
	case PistonJointType:
		j = PistonJoint{base}
	case DBallJointType:
		j = DBallJoint{base}
	case DHingeJointType:
		j = DHingeJoint{base}
	case TransmissionJointType:
		j = TransmissionJoint{base}
	default:
		j = base
	}
	return j
}

func (j JointBase) c() C.dJointID {
	return C.dJointID(unsafe.Pointer(j))
}

func (j JointBase) Destroy() {
	delete(jointData, j)
	C.dJointDestroy(j.c())
}

func (j JointBase) SetData(data interface{}) {
	jointData[j] = data
}

func (j JointBase) Data() interface{} {
	return jointData[j]
}

func (j JointBase) NumBodies() int {
	return int(C.dJointGetNumBodies(j.c()))
}

func (j JointBase) Attach(body1, body2 Body) {
	C.dJointAttach(j.c(), body1.c(), body2.c())
}

func (j JointBase) SetEnabled(isEnabled bool) {
	if isEnabled {
		C.dJointEnable(j.c())
	} else {
		C.dJointDisable(j.c())
	}
}

func (j JointBase) Enabled() bool {
	return bool(C.dJointIsEnabled(j.c()) != 0)
}

func (j JointBase) Type() int {
	return int(C.dJointGetType(j.c()))
}

func (j JointBase) Body(index int) Body {
	return cToBody(C.dJointGetBody(j.c(), C.int(index)))
}

func (j JointBase) SetFeedback(f *JointFeedback) {
	c := &C.dJointFeedback{}
	f.toC(c)
	C.dJointSetFeedback(j.c(), c)
}

func (j JointBase) Feedback() *JointFeedback {
	f := &JointFeedback{}
	f.fromC(C.dJointGetFeedback(j.c()))
	return f
}

// BallJoint

type BallJoint struct {
	JointBase
}

func (j BallJoint) SetParam(parameter int, value float64) {
	C.dJointSetBallParam(j.c(), C.int(parameter), C.dReal(value))
}

func (j BallJoint) Param(parameter int) float64 {
	return float64(C.dJointGetBallParam(j.c(), C.int(parameter)))
}

func (j BallJoint) SetAnchor(pt Vector3) {
	C.dJointSetBallAnchor(j.c(), C.dReal(pt[0]), C.dReal(pt[1]), C.dReal(pt[2]))
}

func (j BallJoint) Anchor() Vector3 {
	pt := NewVector3()
	C.dJointGetBallAnchor(j.c(), (*C.dReal)(&pt[0]))
	return pt
}

func (j BallJoint) SetAnchor2(pt Vector3) {
	C.dJointSetBallAnchor2(j.c(), C.dReal(pt[0]), C.dReal(pt[1]), C.dReal(pt[2]))
}

func (j BallJoint) Anchor2() Vector3 {
	pt := NewVector3()
	C.dJointGetBallAnchor2(j.c(), (*C.dReal)(&pt[0]))
	return pt
}

// HingeJoint

type HingeJoint struct {
	JointBase
}

func (j HingeJoint) SetParam(parameter int, value float64) {
	C.dJointSetHingeParam(j.c(), C.int(parameter), C.dReal(value))
}

func (j HingeJoint) Param(parameter int) float64 {
	return float64(C.dJointGetHingeParam(j.c(), C.int(parameter)))
}

func (j HingeJoint) SetAnchor(pt Vector3) {
	C.dJointSetHingeAnchor(j.c(), C.dReal(pt[0]), C.dReal(pt[1]), C.dReal(pt[2]))
}

func (j HingeJoint) SetAnchorDelta(pt, delta Vector3) {
	C.dJointSetHingeAnchorDelta(j.c(), C.dReal(pt[0]), C.dReal(pt[1]), C.dReal(pt[2]),
		C.dReal(delta[0]), C.dReal(delta[1]), C.dReal(delta[2]))
}

func (j HingeJoint) Anchor() Vector3 {
	pt := NewVector3()
	C.dJointGetHingeAnchor(j.c(), (*C.dReal)(&pt[0]))
	return pt
}

func (j HingeJoint) Anchor2() Vector3 {
	pt := NewVector3()
	C.dJointGetHingeAnchor2(j.c(), (*C.dReal)(&pt[0]))
	return pt
}

func (j HingeJoint) SetAxis(axis Vector3) {
	C.dJointSetHingeAxis(j.c(), C.dReal(axis[0]), C.dReal(axis[1]), C.dReal(axis[2]))
}

func (j HingeJoint) SetAxisOffset(axis Vector3, angle float64) {
	C.dJointSetHingeAxisOffset(j.c(), C.dReal(axis[0]), C.dReal(axis[1]), C.dReal(axis[2]),
		C.dReal(angle))
}

func (j HingeJoint) Axis() Vector3 {
	axis := NewVector3()
	C.dJointGetHingeAxis(j.c(), (*C.dReal)(&axis[0]))
	return axis
}

func (j HingeJoint) AddTorque(torque float64) {
	C.dJointAddHingeTorque(j.c(), C.dReal(torque))
}

func (j HingeJoint) Angle() float64 {
	return float64(C.dJointGetHingeAngle(j.c()))
}

func (j HingeJoint) AngleRate() float64 {
	return float64(C.dJointGetHingeAngleRate(j.c()))
}

// SliderJoint

type SliderJoint struct {
	JointBase
}

func (j SliderJoint) SetParam(parameter int, value float64) {
	C.dJointSetSliderParam(j.c(), C.int(parameter), C.dReal(value))
}

func (j SliderJoint) Param(parameter int) float64 {
	return float64(C.dJointGetSliderParam(j.c(), C.int(parameter)))
}

func (j SliderJoint) Position() float64 {
	return float64(C.dJointGetSliderPosition(j.c()))
}

func (j SliderJoint) PositionRate() float64 {
	return float64(C.dJointGetSliderPositionRate(j.c()))
}

func (j SliderJoint) SetAxis(axis Vector3) {
	C.dJointSetSliderAxis(j.c(), C.dReal(axis[0]), C.dReal(axis[1]), C.dReal(axis[2]))
}

func (j SliderJoint) SetAxisDelta(pt, delta Vector3) {
	C.dJointSetSliderAxisDelta(j.c(), C.dReal(pt[0]), C.dReal(pt[1]), C.dReal(pt[2]),
		C.dReal(delta[0]), C.dReal(delta[1]), C.dReal(delta[2]))
}

func (j SliderJoint) Axis() Vector3 {
	axis := NewVector3()
	C.dJointGetSliderAxis(j.c(), (*C.dReal)(&axis[0]))
	return axis
}

func (j SliderJoint) AddForce(force float64) {
	C.dJointAddSliderForce(j.c(), C.dReal(force))
}

// ContactJoint

type ContactJoint struct {
	JointBase
}

// UniversalJoint

type UniversalJoint struct {
	JointBase
}

func (j UniversalJoint) SetParam(parameter int, value float64) {
	C.dJointSetUniversalParam(j.c(), C.int(parameter), C.dReal(value))
}

func (j UniversalJoint) Param(parameter int) float64 {
	return float64(C.dJointGetUniversalParam(j.c(), C.int(parameter)))
}

func (j UniversalJoint) SetAnchor(pt Vector3) {
	C.dJointSetUniversalAnchor(j.c(), C.dReal(pt[0]), C.dReal(pt[1]), C.dReal(pt[2]))
}

func (j UniversalJoint) Anchor() Vector3 {
	pt := NewVector3()
	C.dJointGetUniversalAnchor(j.c(), (*C.dReal)(&pt[0]))
	return pt
}

func (j UniversalJoint) Anchor2() Vector3 {
	pt := NewVector3()
	C.dJointGetUniversalAnchor2(j.c(), (*C.dReal)(&pt[0]))
	return pt
}

func (j UniversalJoint) SetAxis1(axis Vector3) {
	C.dJointSetUniversalAxis1(j.c(), C.dReal(axis[0]), C.dReal(axis[1]), C.dReal(axis[2]))
}

func (j UniversalJoint) SetAxis1Offset(axis Vector3, offset1, offset2 float64) {
	C.dJointSetUniversalAxis1Offset(j.c(), C.dReal(axis[0]), C.dReal(axis[1]), C.dReal(axis[2]),
		C.dReal(offset1), C.dReal(offset2))
}

func (j UniversalJoint) Axis1() Vector3 {
	axis := NewVector3()
	C.dJointGetUniversalAxis1(j.c(), (*C.dReal)(&axis[0]))
	return axis
}

func (j UniversalJoint) SetAxis2(axis Vector3) {
	C.dJointSetUniversalAxis2(j.c(), C.dReal(axis[0]), C.dReal(axis[1]), C.dReal(axis[2]))
}

func (j UniversalJoint) SetAxis2Offset(axis Vector3, offset1, offset2 float64) {
	C.dJointSetUniversalAxis2Offset(j.c(), C.dReal(axis[0]), C.dReal(axis[1]), C.dReal(axis[2]),
		C.dReal(offset1), C.dReal(offset1))
}

func (j UniversalJoint) Axis2() Vector3 {
	axis := NewVector3()
	C.dJointGetUniversalAxis2(j.c(), (*C.dReal)(&axis[0]))
	return axis
}

func (j UniversalJoint) Angle1() float64 {
	return float64(C.dJointGetUniversalAngle1(j.c()))
}

func (j UniversalJoint) Angle1Rate() float64 {
	return float64(C.dJointGetUniversalAngle1Rate(j.c()))
}

func (j UniversalJoint) Angle2() float64 {
	return float64(C.dJointGetUniversalAngle2(j.c()))
}

func (j UniversalJoint) Angle2Rate() float64 {
	return float64(C.dJointGetUniversalAngle2Rate(j.c()))
}

func (j UniversalJoint) Angles() (float64, float64) {
	var angle1, angle2 float64
	C.dJointGetUniversalAngles(j.c(), (*C.dReal)(&angle1), (*C.dReal)(&angle2))
	return angle1, angle2
}

func (j UniversalJoint) AddTorques(torque1, torque2 float64) {
	C.dJointAddUniversalTorques(j.c(), C.dReal(torque1), C.dReal(torque2))
}

// Hinge2Joint

type Hinge2Joint struct {
	JointBase
}

func (j Hinge2Joint) SetParam(parameter int, value float64) {
	C.dJointSetHinge2Param(j.c(), C.int(parameter), C.dReal(value))
}

func (j Hinge2Joint) Param(parameter int) float64 {
	return float64(C.dJointGetHinge2Param(j.c(), C.int(parameter)))
}

func (j Hinge2Joint) SetAnchor(pt Vector3) {
	C.dJointSetHinge2Anchor(j.c(), C.dReal(pt[0]), C.dReal(pt[1]), C.dReal(pt[2]))
}

func (j Hinge2Joint) Anchor() Vector3 {
	pt := NewVector3()
	C.dJointGetHinge2Anchor(j.c(), (*C.dReal)(&pt[0]))
	return pt
}

func (j Hinge2Joint) Anchor2() Vector3 {
	pt := NewVector3()
	C.dJointGetHinge2Anchor2(j.c(), (*C.dReal)(&pt[0]))
	return pt
}

func (j Hinge2Joint) SetAxis1(axis Vector3) {
	C.dJointSetHinge2Axis1(j.c(), C.dReal(axis[0]), C.dReal(axis[1]), C.dReal(axis[2]))
}

func (j Hinge2Joint) Axis1() Vector3 {
	axis := NewVector3()
	C.dJointGetHinge2Axis1(j.c(), (*C.dReal)(&axis[0]))
	return axis
}

func (j Hinge2Joint) SetAxis2(axis Vector3) {
	C.dJointSetHinge2Axis2(j.c(), C.dReal(axis[0]), C.dReal(axis[1]), C.dReal(axis[2]))
}

func (j Hinge2Joint) Axis2() Vector3 {
	axis := NewVector3()
	C.dJointGetHinge2Axis2(j.c(), (*C.dReal)(&axis[0]))
	return axis
}

func (j Hinge2Joint) Angle1() float64 {
	return float64(C.dJointGetHinge2Angle1(j.c()))
}

func (j Hinge2Joint) Angle1Rate() float64 {
	return float64(C.dJointGetHinge2Angle1Rate(j.c()))
}

func (j Hinge2Joint) Angle2() float64 {
	return float64(C.dJointGetHinge2Angle2(j.c()))
}

func (j Hinge2Joint) Angle2Rate() float64 {
	return float64(C.dJointGetHinge2Angle2Rate(j.c()))
}

func (j Hinge2Joint) AddTorques(torque1, torque2 float64) {
	C.dJointAddHinge2Torques(j.c(), C.dReal(torque1), C.dReal(torque2))
}

// FixedJoint

type FixedJoint struct {
	JointBase
}

func (j FixedJoint) SetParam(parameter int, value float64) {
	C.dJointSetFixedParam(j.c(), C.int(parameter), C.dReal(value))
}

func (j FixedJoint) Param(parameter int) float64 {
	return float64(C.dJointGetFixedParam(j.c(), C.int(parameter)))
}

func (j FixedJoint) Fix() {
	C.dJointSetFixed(j.c())
}

// NullJoint

type NullJoint struct {
	JointBase
}

// AMotorJoint

type AMotorJoint struct {
	JointBase
}

func (j AMotorJoint) SetParam(parameter int, value float64) {
	C.dJointSetAMotorParam(j.c(), C.int(parameter), C.dReal(value))
}

func (j AMotorJoint) Param(parameter int) float64 {
	return float64(C.dJointGetAMotorParam(j.c(), C.int(parameter)))
}

func (j AMotorJoint) SetNumAxes(num int) {
	C.dJointSetAMotorNumAxes(j.c(), C.int(num))
}

func (j AMotorJoint) NumAxes() int {
	return int(C.dJointGetAMotorNumAxes(j.c()))
}

func (j AMotorJoint) SetAxis(num, rel int, axis Vector3) {
	C.dJointSetAMotorAxis(j.c(), C.int(num), C.int(rel),
		C.dReal(axis[0]), C.dReal(axis[1]), C.dReal(axis[2]))
}

func (j AMotorJoint) Axis(num int) Vector3 {
	axis := NewVector3()
	C.dJointGetAMotorAxis(j.c(), C.int(num), (*C.dReal)(&axis[0]))
	return axis
}

func (j AMotorJoint) AxisRel(num int) int {
	return int(C.dJointGetAMotorAxisRel(j.c(), C.int(num)))
}

func (j AMotorJoint) SetAngle(num int, angle float64) {
	C.dJointSetAMotorAngle(j.c(), C.int(num), C.dReal(angle))
}

func (j AMotorJoint) Angle(num int) float64 {
	return float64(C.dJointGetAMotorAngle(j.c(), C.int(num)))
}

func (j AMotorJoint) AngleRate(num int) float64 {
	return float64(C.dJointGetAMotorAngleRate(j.c(), C.int(num)))
}

func (j AMotorJoint) SetMode(mode int) {
	C.dJointSetAMotorMode(j.c(), C.int(mode))
}

func (j AMotorJoint) Mode() int {
	return int(C.dJointGetAMotorMode(j.c()))
}

func (j AMotorJoint) AddTorques(torque1, torque2, torque3 float64) {
	C.dJointAddAMotorTorques(j.c(), C.dReal(torque1), C.dReal(torque2), C.dReal(torque3))
}

// LMotorJoint

type LMotorJoint struct {
	JointBase
}

func (j LMotorJoint) SetParam(parameter int, value float64) {
	C.dJointSetLMotorParam(j.c(), C.int(parameter), C.dReal(value))
}

func (j LMotorJoint) Param(parameter int) float64 {
	return float64(C.dJointGetLMotorParam(j.c(), C.int(parameter)))
}

func (j LMotorJoint) SetNumAxes(num int) {
	C.dJointSetLMotorNumAxes(j.c(), C.int(num))
}

func (j LMotorJoint) NumAxes() int {
	return int(C.dJointGetLMotorNumAxes(j.c()))
}

func (j LMotorJoint) SetAxis(num, rel int, axis Vector3) {
	C.dJointSetLMotorAxis(j.c(), C.int(num), C.int(rel),
		C.dReal(axis[0]), C.dReal(axis[1]), C.dReal(axis[2]))
}

func (j LMotorJoint) Axis(num int) Vector3 {
	axis := NewVector3()
	C.dJointGetLMotorAxis(j.c(), C.int(num), (*C.dReal)(&axis[0]))
	return axis
}

// Plane2DJoint

type Plane2DJoint struct {
	JointBase
}

func (j Plane2DJoint) Set2DXParam(parameter int, value float64) {
	C.dJointSetPlane2DXParam(j.c(), C.int(parameter), C.dReal(value))
}

func (j Plane2DJoint) Set2DYParam(parameter int, value float64) {
	C.dJointSetPlane2DYParam(j.c(), C.int(parameter), C.dReal(value))
}

func (j Plane2DJoint) Set2DAngleParam(parameter int, value float64) {
	C.dJointSetPlane2DAngleParam(j.c(), C.int(parameter), C.dReal(value))
}

// PRJoint

type PRJoint struct {
	JointBase
}

func (j PRJoint) SetParam(parameter int, value float64) {
	C.dJointSetPRParam(j.c(), C.int(parameter), C.dReal(value))
}

func (j PRJoint) Param(parameter int) float64 {
	return float64(C.dJointGetPRParam(j.c(), C.int(parameter)))
}

func (j PRJoint) SetAnchor(pt Vector3) {
	C.dJointSetPRAnchor(j.c(), C.dReal(pt[0]), C.dReal(pt[1]), C.dReal(pt[2]))
}

func (j PRJoint) Anchor() Vector3 {
	pt := NewVector3()
	C.dJointGetPRAnchor(j.c(), (*C.dReal)(&pt[0]))
	return pt
}

func (j PRJoint) SetAxis1(axis Vector3) {
	C.dJointSetPRAxis1(j.c(), C.dReal(axis[0]), C.dReal(axis[1]), C.dReal(axis[2]))
}

func (j PRJoint) Axis1() Vector3 {
	axis := NewVector3()
	C.dJointGetPRAxis1(j.c(), (*C.dReal)(&axis[0]))
	return axis
}

func (j PRJoint) SetAxis2(axis Vector3) {
	C.dJointSetPRAxis2(j.c(), C.dReal(axis[0]), C.dReal(axis[1]), C.dReal(axis[2]))
}

func (j PRJoint) Axis2() Vector3 {
	axis := NewVector3()
	C.dJointGetPRAxis2(j.c(), (*C.dReal)(&axis[0]))
	return axis
}

func (j PRJoint) Position() float64 {
	return float64(C.dJointGetPRPosition(j.c()))
}

func (j PRJoint) PositionRate() float64 {
	return float64(C.dJointGetPRPositionRate(j.c()))
}

func (j PRJoint) Angle() float64 {
	return float64(C.dJointGetPRAngle(j.c()))
}

func (j PRJoint) AngleRate() float64 {
	return float64(C.dJointGetPRAngleRate(j.c()))
}

func (j PRJoint) AddTorque(torque float64) {
	C.dJointAddPRTorque(j.c(), C.dReal(torque))
}

// PUJoint

type PUJoint struct {
	JointBase
}

func (j PUJoint) SetParam(parameter int, value float64) {
	C.dJointSetPUParam(j.c(), C.int(parameter), C.dReal(value))
}

func (j PUJoint) Param(parameter int) float64 {
	return float64(C.dJointGetPUParam(j.c(), C.int(parameter)))
}

func (j PUJoint) SetAnchor(pt Vector3) {
	C.dJointSetPUAnchor(j.c(), C.dReal(pt[0]), C.dReal(pt[1]), C.dReal(pt[2]))
}

func (j PUJoint) Anchor() Vector3 {
	pt := NewVector3()
	C.dJointGetPUAnchor(j.c(), (*C.dReal)(&pt[0]))
	return pt
}

func (j PUJoint) SetAnchorOffset(pt, delta Vector3) {
	C.dJointSetPUAnchorOffset(j.c(), C.dReal(pt[0]), C.dReal(pt[1]), C.dReal(pt[2]),
		C.dReal(delta[0]), C.dReal(delta[1]), C.dReal(delta[2]))
}

func (j PUJoint) SetAxis1(axis Vector3) {
	C.dJointSetPUAxis1(j.c(), C.dReal(axis[0]), C.dReal(axis[1]), C.dReal(axis[2]))
}

func (j PUJoint) Axis1() Vector3 {
	axis := NewVector3()
	C.dJointGetPUAxis1(j.c(), (*C.dReal)(&axis[0]))
	return axis
}

func (j PUJoint) SetAxis2(axis Vector3) {
	C.dJointSetPUAxis2(j.c(), C.dReal(axis[0]), C.dReal(axis[1]), C.dReal(axis[2]))
}

func (j PUJoint) Axis2() Vector3 {
	axis := NewVector3()
	C.dJointGetPUAxis2(j.c(), (*C.dReal)(&axis[0]))
	return axis
}

func (j PUJoint) SetAxis3(axis Vector3) {
	C.dJointSetPUAxis3(j.c(), C.dReal(axis[0]), C.dReal(axis[1]), C.dReal(axis[2]))
}

func (j PUJoint) Axis3() Vector3 {
	axis := NewVector3()
	C.dJointGetPUAxis3(j.c(), (*C.dReal)(&axis[0]))
	return axis
}

func (j PUJoint) SetAxisP(axis Vector3) {
	C.dJointSetPUAxisP(j.c(), C.dReal(axis[0]), C.dReal(axis[1]), C.dReal(axis[2]))
}

func (j PUJoint) AxisP() Vector3 {
	axis := NewVector3()
	C.dJointGetPUAxisP(j.c(), (*C.dReal)(&axis[0]))
	return axis
}

func (j PUJoint) Position() float64 {
	return float64(C.dJointGetPUPosition(j.c()))
}

func (j PUJoint) PositionRate() float64 {
	return float64(C.dJointGetPUPositionRate(j.c()))
}

func (j PUJoint) Angle1() float64 {
	return float64(C.dJointGetPUAngle1(j.c()))
}

func (j PUJoint) Angle1Rate() float64 {
	return float64(C.dJointGetPUAngle1Rate(j.c()))
}

func (j PUJoint) Angle2() float64 {
	return float64(C.dJointGetPUAngle2(j.c()))
}

func (j PUJoint) Angle2Rate() float64 {
	return float64(C.dJointGetPUAngle2Rate(j.c()))
}

func (j PUJoint) Angles() (float64, float64) {
	var angle1, angle2 float64
	C.dJointGetPUAngles(j.c(), (*C.dReal)(&angle1), (*C.dReal)(&angle2))
	return angle1, angle2
}

// PistonJoint

type PistonJoint struct {
	JointBase
}

func (j PistonJoint) SetParam(parameter int, value float64) {
	C.dJointSetPistonParam(j.c(), C.int(parameter), C.dReal(value))
}

func (j PistonJoint) Param(parameter int) float64 {
	return float64(C.dJointGetPistonParam(j.c(), C.int(parameter)))
}

func (j PistonJoint) SetAnchor(pt Vector3) {
	C.dJointSetPistonAnchor(j.c(), C.dReal(pt[0]), C.dReal(pt[1]), C.dReal(pt[2]))
}

func (j PistonJoint) SetAnchorOffset(pt, delta Vector3) {
	C.dJointSetPistonAnchorOffset(j.c(), C.dReal(pt[0]), C.dReal(pt[1]), C.dReal(pt[2]),
		C.dReal(delta[0]), C.dReal(delta[1]), C.dReal(delta[2]))
}

func (j PistonJoint) Anchor2() Vector3 {
	pt := NewVector3()
	C.dJointGetPistonAnchor2(j.c(), (*C.dReal)(&pt[0]))
	return pt
}

func (j PistonJoint) SetAxis(axis Vector3) {
	C.dJointSetPistonAxis(j.c(), C.dReal(axis[0]), C.dReal(axis[1]), C.dReal(axis[2]))
}

func (j PistonJoint) Axis() Vector3 {
	axis := NewVector3()
	C.dJointGetPistonAxis(j.c(), (*C.dReal)(&axis[0]))
	return axis
}

func (j PistonJoint) Position() float64 {
	return float64(C.dJointGetPistonPosition(j.c()))
}

func (j PistonJoint) PositionRate() float64 {
	return float64(C.dJointGetPistonPositionRate(j.c()))
}

func (j PistonJoint) Angle() float64 {
	return float64(C.dJointGetPistonAngle(j.c()))
}

func (j PistonJoint) AngleRate() float64 {
	return float64(C.dJointGetPistonAngleRate(j.c()))
}

func (j PistonJoint) AddForce(force float64) {
	C.dJointAddPistonForce(j.c(), C.dReal(force))
}

// DBallJoint

type DBallJoint struct {
	JointBase
}

func (j DBallJoint) SetParam(parameter int, value float64) {
	C.dJointSetDBallParam(j.c(), C.int(parameter), C.dReal(value))
}

func (j DBallJoint) Param(parameter int) float64 {
	return float64(C.dJointGetDBallParam(j.c(), C.int(parameter)))
}

func (j DBallJoint) SetAnchor1(pt Vector3) {
	C.dJointSetDBallAnchor1(j.c(), C.dReal(pt[0]), C.dReal(pt[1]), C.dReal(pt[2]))
}

func (j DBallJoint) Anchor1() Vector3 {
	pt := NewVector3()
	C.dJointGetDBallAnchor1(j.c(), (*C.dReal)(&pt[0]))
	return pt
}

func (j DBallJoint) SetAnchor2(pt Vector3) {
	C.dJointSetDBallAnchor2(j.c(), C.dReal(pt[0]), C.dReal(pt[1]), C.dReal(pt[2]))
}

func (j DBallJoint) Anchor2() Vector3 {
	pt := NewVector3()
	C.dJointGetDBallAnchor2(j.c(), (*C.dReal)(&pt[0]))
	return pt
}

func (j DBallJoint) Distance() float64 {
	return float64(C.dJointGetDBallDistance(j.c()))
}

// DHingeJoint

type DHingeJoint struct {
	JointBase
}

func (j DHingeJoint) SetParam(parameter int, value float64) {
	C.dJointSetDHingeParam(j.c(), C.int(parameter), C.dReal(value))
}

func (j DHingeJoint) Param(parameter int) float64 {
	return float64(C.dJointGetDHingeParam(j.c(), C.int(parameter)))
}

func (j DHingeJoint) SetAxis(axis Vector3) {
	C.dJointSetDHingeAxis(j.c(), C.dReal(axis[0]), C.dReal(axis[1]), C.dReal(axis[2]))
}

func (j DHingeJoint) Axis() Vector3 {
	axis := NewVector3()
	C.dJointGetDHingeAxis(j.c(), (*C.dReal)(&axis[0]))
	return axis
}

func (j DHingeJoint) SetAnchor1(pt Vector3) {
	C.dJointSetDHingeAnchor1(j.c(), C.dReal(pt[0]), C.dReal(pt[1]), C.dReal(pt[2]))
}

func (j DHingeJoint) Anchor1() Vector3 {
	pt := NewVector3()
	C.dJointGetDHingeAnchor1(j.c(), (*C.dReal)(&pt[0]))
	return pt
}

func (j DHingeJoint) SetAnchor2(pt Vector3) {
	C.dJointSetDHingeAnchor2(j.c(), C.dReal(pt[0]), C.dReal(pt[1]), C.dReal(pt[2]))
}

func (j DHingeJoint) Anchor2() Vector3 {
	pt := NewVector3()
	C.dJointGetDHingeAnchor2(j.c(), (*C.dReal)(&pt[0]))
	return pt
}

func (j DHingeJoint) Distance() float64 {
	return float64(C.dJointGetDHingeDistance(j.c()))
}

// TransmissionJoint

type TransmissionJoint struct {
	JointBase
}

func (j TransmissionJoint) SetParam(parameter int, value float64) {
	C.dJointSetTransmissionParam(j.c(), C.int(parameter), C.dReal(value))
}

func (j TransmissionJoint) Param(parameter int) float64 {
	return float64(C.dJointGetTransmissionParam(j.c(), C.int(parameter)))
}

func (j TransmissionJoint) SetAxis(axis Vector3) {
	C.dJointSetTransmissionAxis(j.c(), C.dReal(axis[0]), C.dReal(axis[1]), C.dReal(axis[2]))
}

func (j TransmissionJoint) Axis() Vector3 {
	axis := NewVector3()
	C.dJointGetTransmissionAxis(j.c(), (*C.dReal)(&axis[0]))
	return axis
}

func (j TransmissionJoint) SetAxis1(axis Vector3) {
	C.dJointSetTransmissionAxis1(j.c(), C.dReal(axis[0]), C.dReal(axis[1]), C.dReal(axis[2]))
}

func (j TransmissionJoint) Axis1() Vector3 {
	axis := NewVector3()
	C.dJointGetTransmissionAxis1(j.c(), (*C.dReal)(&axis[0]))
	return axis
}

func (j TransmissionJoint) SetAxis2(axis Vector3) {
	C.dJointSetTransmissionAxis2(j.c(), C.dReal(axis[0]), C.dReal(axis[1]), C.dReal(axis[2]))
}

func (j TransmissionJoint) Axis2() Vector3 {
	axis := NewVector3()
	C.dJointGetTransmissionAxis2(j.c(), (*C.dReal)(&axis[0]))
	return axis
}

func (j TransmissionJoint) SetAnchor1(pt Vector3) {
	C.dJointSetTransmissionAnchor1(j.c(), C.dReal(pt[0]), C.dReal(pt[1]), C.dReal(pt[2]))
}

func (j TransmissionJoint) Anchor1() Vector3 {
	pt := NewVector3()
	C.dJointGetTransmissionAnchor1(j.c(), (*C.dReal)(&pt[0]))
	return pt
}

func (j TransmissionJoint) SetAnchor2(pt Vector3) {
	C.dJointSetTransmissionAnchor2(j.c(), C.dReal(pt[0]), C.dReal(pt[1]), C.dReal(pt[2]))
}

func (j TransmissionJoint) Anchor2() Vector3 {
	pt := NewVector3()
	C.dJointGetTransmissionAnchor2(j.c(), (*C.dReal)(&pt[0]))
	return pt
}

func (j TransmissionJoint) ContactPoint1() Vector3 {
	pt := NewVector3()
	C.dJointGetTransmissionContactPoint1(j.c(), (*C.dReal)(&pt[0]))
	return pt
}

func (j TransmissionJoint) ContactPoint2() Vector3 {
	pt := NewVector3()
	C.dJointGetTransmissionContactPoint2(j.c(), (*C.dReal)(&pt[0]))
	return pt
}

func (j TransmissionJoint) Angle1() float64 {
	return float64(C.dJointGetTransmissionAngle1(j.c()))
}

func (j TransmissionJoint) Angle2() float64 {
	return float64(C.dJointGetTransmissionAngle2(j.c()))
}

func (j TransmissionJoint) SetRadius1(radius float64) {
	C.dJointSetTransmissionRadius1(j.c(), C.dReal(radius))
}

func (j TransmissionJoint) Radius1() float64 {
	return float64(C.dJointGetTransmissionRadius1(j.c()))
}

func (j TransmissionJoint) SetRadius2(radius float64) {
	C.dJointSetTransmissionRadius2(j.c(), C.dReal(radius))
}

func (j TransmissionJoint) Radius2() float64 {
	return float64(C.dJointGetTransmissionRadius2(j.c()))
}

func (j TransmissionJoint) SetMode(mode int) {
	C.dJointSetTransmissionMode(j.c(), C.int(mode))
}

func (j TransmissionJoint) Mode() int {
	return int(C.dJointGetTransmissionMode(j.c()))
}

func (j TransmissionJoint) SetRatio(ratio float64) {
	C.dJointSetTransmissionRatio(j.c(), C.dReal(ratio))
}

func (j TransmissionJoint) Ratio() float64 {
	return float64(C.dJointGetTransmissionRatio(j.c()))
}

func (j TransmissionJoint) SetBacklash(backlash float64) {
	C.dJointSetTransmissionBacklash(j.c(), C.dReal(backlash))
}

func (j TransmissionJoint) Backlash() float64 {
	return float64(C.dJointGetTransmissionBacklash(j.c()))
}
