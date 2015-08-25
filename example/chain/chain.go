package main

import (
	"fmt"
	"math"
	"os"

	"github.com/ianremmler/ode"
	"gopkg.in/qml.v1"
	"gopkg.in/qml.v1/gl/1.0"
)

const (
	numSpheres   = 10
	sideLen      = 0.2
	sphereMass   = 1
	sphereRadius = 0.1732
)

var (
	world  ode.World
	space  ode.Space
	body   []ode.Body
	joint  []ode.BallJoint
	ctGrp  ode.JointGroup
	sphere []ode.Sphere
	mass   *ode.Mass
	angle  float64
)

func cb(data interface{}, obj1, obj2 ode.Geom) {
	contact := ode.NewContact()
	body1, body2 := obj1.Body(), obj2.Body()
	if body1 != 0 && body2 != 0 && body1.Connected(body2) {
		return
	}
	contact.Surface.Mode = 0
	contact.Surface.Mu = 0.1
	contact.Surface.Mu2 = 0
	cts := obj1.Collide(obj2, 1, 0)
	if len(cts) > 0 {
		contact.Geom = cts[0]
		ct := world.NewContactJoint(ctGrp, contact)
		ct.Attach(body1, body2)
	}
}

type Sim struct {
	qml.Object
}

func (s *Sim) Iter() {
	angle += 0.05
	body[len(body)-1].AddForce(ode.V3(0, 0, 1.5*(math.Sin(angle)+1)))
	space.Collide(0, cb)
	world.Step(0.05)
	ctGrp.Empty()
}

func (s *Sim) Paint(p *qml.Painter) {
	gl := GL.API(p)
	diffuseColor := []float32{0.5, 1, 1, 0}
	diffusePos := []float32{0, 0, 1, 0}

	gl.Rotated(-45, 1, 0, 0)

	gl.Lightfv(GL.LIGHT0, GL.DIFFUSE, diffuseColor)
	gl.Lightfv(GL.LIGHT0, GL.POSITION, diffusePos)
	gl.Enable(GL.LIGHT0)
	gl.Enable(GL.LIGHTING)
	gl.Enable(GL.DEPTH_TEST)
	gl.Enable(GL.NORMALIZE)

	gl.ClearColor(0, 0, 0, 0)
	gl.Clear(GL.COLOR_BUFFER_BIT | GL.DEPTH_BUFFER_BIT)

	width := float64(s.Int("width"))
	height := float64(s.Int("height"))

	gl.PushMatrix()
	scale := width / 5
	gl.Translated(0.5*width, 0.5*height, 0)
	gl.Scaled(scale, scale, scale)
	for i := range body {
		pos := body[i].Position()
		gl.PushMatrix()
		gl.Translated(pos[0], pos[1], pos[2])
		gl.Scaled(sphereRadius, sphereRadius, sphereRadius)
		drawSphere(gl, 16, 16)
		gl.PopMatrix()
	}
	gl.PopMatrix()
}

func run() error {
	qml.RegisterTypes("GoExtensions", 1, 0, []qml.TypeSpec{{
		Init: func(s *Sim, obj qml.Object) { s.Object = obj },
	}})

	engine := qml.NewEngine()
	component, err := engine.LoadFile("chain.qml")
	if err != nil {
		return err
	}

	win := component.CreateWindow(nil)
	win.Show()
	win.Wait()

	return nil
}

func drawSphere(gl *GL.GL, latSegs, lonSegs int) {
	for i := 0; i < latSegs; i++ {
		latFrac0, latFrac1 := float64(i)/float64(latSegs), float64(i+1)/float64(latSegs)
		latAngle0, latAngle1 := math.Pi*(latFrac0-0.5), math.Pi*(latFrac1-0.5)
		z0, z1 := math.Sin(latAngle0), math.Sin(latAngle1)
		r0, r1 := math.Cos(latAngle0), math.Cos(latAngle1)
		gl.Begin(GL.QUAD_STRIP)
		for j := 0; j <= lonSegs; j++ {
			lonFrac := float64(j) / float64(lonSegs)
			lonAngle := 2 * math.Pi * lonFrac
			x0, x1 := r0*math.Cos(lonAngle), r1*math.Cos(lonAngle)
			y0, y1 := r0*math.Sin(lonAngle), r1*math.Sin(lonAngle)
			gl.Normal3d(x0, y0, z0)
			gl.Vertex3d(x0, y0, z0)
			gl.Normal3d(x1, y1, z1)
			gl.Vertex3d(x1, y1, z1)
		}
		gl.End()
	}
}

func main() {
	ode.Init(0, ode.AllAFlag)

	world = ode.NewWorld()
	space = ode.NilSpace().NewHashSpace()
	body = make([]ode.Body, numSpheres)
	joint = make([]ode.BallJoint, numSpheres-1)
	ctGrp = ode.NewJointGroup(1000000)
	sphere = make([]ode.Sphere, numSpheres)
	mass = ode.NewMass()

	world.SetGravity(ode.V3(0, 0, -0.5))
	space.NewPlane(ode.V4(0, 0, 1, 0))

	for i := 0; i < numSpheres; i++ {
		k := float64(i) * sideLen
		body[i] = world.NewBody()
		body[i].SetPosition(ode.V3(k, k, k+0.4))
		mass.SetBox(1, ode.V3(sideLen, sideLen, sideLen))
		mass.Adjust(sphereMass)
		body[i].SetMass(mass)
		sphere[i] = space.NewSphere(sphereRadius)
		sphere[i].SetBody(body[i])
	}

	for i := 0; i < numSpheres-1; i++ {
		joint[i] = world.NewBallJoint(ode.JointGroup(0))
		joint[i].Attach(body[i], body[i+1])
		k := (float64(i) + 0.5) * sideLen
		joint[i].SetAnchor(ode.V3(k, k, k+0.4))
	}

	if err := qml.Run(run); err != nil {
		fmt.Fprintf(os.Stderr, "error: %v\n", err)
		os.Exit(1)
	}
}
