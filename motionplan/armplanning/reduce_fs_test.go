package armplanning

import (
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/test"

	"go.viam.com/rdk/motionplan"
	frame "go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// makeDualArmFS creates a FrameSystem with two 1-DOF translational "arms" for testing.
// Structure:
//
//	world -> leftOffset -> leftArm -> leftGripper
//	world -> rightOffset -> rightArm -> rightGripper
func makeDualArmFS(t *testing.T) *frame.FrameSystem {
	t.Helper()
	fs := frame.NewEmptyFrameSystem("dual_arm")

	// Left arm chain
	leftOffset, err := frame.NewStaticFrame("leftOffset", spatialmath.NewPoseFromPoint(r3.Vector{X: -500}))
	test.That(t, err, test.ShouldBeNil)
	test.That(t, fs.AddFrame(leftOffset, fs.World()), test.ShouldBeNil)

	leftArm, err := frame.NewTranslationalFrame("leftArm", r3.Vector{0, 0, 1}, frame.Limit{Min: -1000, Max: 1000})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, fs.AddFrame(leftArm, leftOffset), test.ShouldBeNil)

	leftBox, err := spatialmath.NewBox(spatialmath.NewZeroPose(), r3.Vector{50, 50, 50}, "leftGripperGeom")
	test.That(t, err, test.ShouldBeNil)
	leftGripper, err := frame.NewStaticFrameWithGeometry("leftGripper",
		spatialmath.NewPoseFromPoint(r3.Vector{Z: 100}), leftBox)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, fs.AddFrame(leftGripper, leftArm), test.ShouldBeNil)

	// Right arm chain
	rightOffset, err := frame.NewStaticFrame("rightOffset", spatialmath.NewPoseFromPoint(r3.Vector{X: 500}))
	test.That(t, err, test.ShouldBeNil)
	test.That(t, fs.AddFrame(rightOffset, fs.World()), test.ShouldBeNil)

	rightArm, err := frame.NewTranslationalFrame("rightArm", r3.Vector{0, 0, 1}, frame.Limit{Min: -1000, Max: 1000})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, fs.AddFrame(rightArm, rightOffset), test.ShouldBeNil)

	rightBox, err := spatialmath.NewBox(spatialmath.NewZeroPose(), r3.Vector{50, 50, 50}, "rightGripperGeom")
	test.That(t, err, test.ShouldBeNil)
	rightGripper, err := frame.NewStaticFrameWithGeometry("rightGripper",
		spatialmath.NewPoseFromPoint(r3.Vector{Z: 100}), rightBox)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, fs.AddFrame(rightGripper, rightArm), test.ShouldBeNil)

	return fs
}

func testZeroInputs(fs *frame.FrameSystem) frame.FrameSystemInputs {
	return frame.NewZeroInputs(fs)
}

func TestReduceToMovingOnly_ReducesFS(t *testing.T) {
	fs := makeDualArmFS(t)
	startInputs := testZeroInputs(fs)

	// Goal: move leftGripper relative to world. Only leftArm is in the motion chain.
	req := &PlanRequest{
		FrameSystem: fs,
		Goals: []*PlanState{
			{poses: frame.FrameSystemPoses{
				"leftGripper": frame.NewPoseInFrame(frame.World, spatialmath.NewPoseFromPoint(r3.Vector{X: -500, Z: 200})),
			}},
		},
		StartState:     NewPlanState(nil, startInputs),
		WorldState:     frame.NewEmptyWorldState(),
		PlannerOptions: NewBasicPlannerOptions(),
	}

	newReq, restore, err := reduceToMovingOnly(req)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, restore, test.ShouldNotBeNil)

	// The reduced FS should contain: leftOffset, leftArm, leftGripper (and world).
	// It should NOT contain: rightOffset, rightArm, rightGripper.
	reducedFS := newReq.FrameSystem
	test.That(t, reducedFS.Frame("leftOffset"), test.ShouldNotBeNil)
	test.That(t, reducedFS.Frame("leftArm"), test.ShouldNotBeNil)
	test.That(t, reducedFS.Frame("leftGripper"), test.ShouldNotBeNil)
	test.That(t, reducedFS.Frame("rightArm"), test.ShouldBeNil)
	test.That(t, reducedFS.Frame("rightGripper"), test.ShouldBeNil)

	// StartState should only have frames from the reduced FS.
	for fName := range newReq.StartState.Configuration() {
		test.That(t, reducedFS.Frame(fName), test.ShouldNotBeNil)
	}
	_, hasRight := newReq.StartState.Configuration()["rightArm"]
	test.That(t, hasRight, test.ShouldBeFalse)
}

func TestReduceToMovingOnly_CrystallizesGeometries(t *testing.T) {
	fs := makeDualArmFS(t)
	startInputs := testZeroInputs(fs)

	req := &PlanRequest{
		FrameSystem: fs,
		Goals: []*PlanState{
			{poses: frame.FrameSystemPoses{
				"leftGripper": frame.NewPoseInFrame(frame.World, spatialmath.NewPoseFromPoint(r3.Vector{X: -500, Z: 200})),
			}},
		},
		StartState:     NewPlanState(nil, startInputs),
		WorldState:     frame.NewEmptyWorldState(),
		PlannerOptions: NewBasicPlannerOptions(),
	}

	newReq, _, err := reduceToMovingOnly(req)
	test.That(t, err, test.ShouldBeNil)

	// The right arm's gripper geometry should be crystallized as a world-frame obstacle.
	obstacles := newReq.WorldState.Obstacles()
	test.That(t, len(obstacles), test.ShouldBeGreaterThan, 0)

	// Find the crystallized geometries.
	var foundCrystallized bool
	for _, gf := range obstacles {
		for _, g := range gf.Geometries() {
			if g.Label() == "rightGripperGeom" {
				foundCrystallized = true
			}
		}
	}
	test.That(t, foundCrystallized, test.ShouldBeTrue)
}

func TestReduceToMovingOnly_AllFramesMoving(t *testing.T) {
	fs := makeDualArmFS(t)
	startInputs := testZeroInputs(fs)

	// Both arms have goals, so all DOF frames are moving.
	req := &PlanRequest{
		FrameSystem: fs,
		Goals: []*PlanState{
			{poses: frame.FrameSystemPoses{
				"leftGripper":  frame.NewPoseInFrame(frame.World, spatialmath.NewPoseFromPoint(r3.Vector{X: -500, Z: 200})),
				"rightGripper": frame.NewPoseInFrame(frame.World, spatialmath.NewPoseFromPoint(r3.Vector{X: 500, Z: 200})),
			}},
		},
		StartState:     NewPlanState(nil, startInputs),
		WorldState:     frame.NewEmptyWorldState(),
		PlannerOptions: NewBasicPlannerOptions(),
	}

	newReq, restore, err := reduceToMovingOnly(req)
	test.That(t, err, test.ShouldBeNil)
	// No reduction needed — restoreInfo should be nil.
	test.That(t, restore, test.ShouldBeNil)
	// The request should be returned unchanged.
	test.That(t, newReq.FrameSystem, test.ShouldEqual, fs)
}

func TestReduceToMovingOnly_ConfigOnlyGoal(t *testing.T) {
	fs := makeDualArmFS(t)
	startInputs := testZeroInputs(fs)

	// Configuration-only goals have no poses, so no motion chain is computed.
	// This should result in no reduction.
	req := &PlanRequest{
		FrameSystem: fs,
		Goals: []*PlanState{
			{structuredConfiguration: frame.FrameSystemInputs{
				"leftArm": {100},
			}},
		},
		StartState:     NewPlanState(nil, startInputs),
		WorldState:     frame.NewEmptyWorldState(),
		PlannerOptions: NewBasicPlannerOptions(),
	}

	newReq, restore, err := reduceToMovingOnly(req)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, restore, test.ShouldBeNil)
	test.That(t, newReq.FrameSystem, test.ShouldEqual, fs)
}

func TestExpandTrajectory(t *testing.T) {
	fs := makeDualArmFS(t)
	startInputs := testZeroInputs(fs)
	startInputs["rightArm"] = []float64{42}

	restore := &restoreInfo{
		originalFS:         fs,
		originalStartState: NewPlanState(nil, startInputs),
	}

	// Simulate a reduced trajectory with only leftArm.
	step1 := frame.NewLinearInputs()
	step1.Put("leftArm", []float64{10})
	step1.Put("leftOffset", []float64{})
	step1.Put("leftGripper", []float64{})

	step2 := frame.NewLinearInputs()
	step2.Put("leftArm", []float64{20})
	step2.Put("leftOffset", []float64{})
	step2.Put("leftGripper", []float64{})

	traj := []*frame.LinearInputs{step1, step2}

	expanded := restore.expandTrajectory(traj)
	test.That(t, len(expanded), test.ShouldEqual, 2)

	for _, step := range expanded {
		// rightArm should be filled in from the original start state.
		rightInputs := step.Get("rightArm")
		test.That(t, rightInputs, test.ShouldNotBeNil)
		test.That(t, len(rightInputs), test.ShouldEqual, 1)
		test.That(t, rightInputs[0], test.ShouldEqual, 42.0)
	}

	// leftArm should have the trajectory values.
	left1 := expanded[0].Get("leftArm")
	test.That(t, left1[0], test.ShouldEqual, 10.0)
	left2 := expanded[1].Get("leftArm")
	test.That(t, left2[0], test.ShouldEqual, 20.0)
}

func TestReduceToMovingOnly_PreservesExistingObstacles(t *testing.T) {
	fs := makeDualArmFS(t)
	startInputs := testZeroInputs(fs)

	// Create an existing obstacle.
	existingBox, err := spatialmath.NewBox(
		spatialmath.NewPoseFromPoint(r3.Vector{X: 300, Y: 0, Z: 300}),
		r3.Vector{100, 100, 100}, "existingObstacle",
	)
	test.That(t, err, test.ShouldBeNil)
	existingObstacles := []*frame.GeometriesInFrame{
		frame.NewGeometriesInFrame(frame.World, []spatialmath.Geometry{existingBox}),
	}
	ws, err := frame.NewWorldState(existingObstacles, nil)
	test.That(t, err, test.ShouldBeNil)

	req := &PlanRequest{
		FrameSystem: fs,
		Goals: []*PlanState{
			{poses: frame.FrameSystemPoses{
				"leftGripper": frame.NewPoseInFrame(frame.World, spatialmath.NewPoseFromPoint(r3.Vector{X: -500, Z: 200})),
			}},
		},
		StartState:     NewPlanState(nil, startInputs),
		WorldState:     ws,
		PlannerOptions: NewBasicPlannerOptions(),
	}

	newReq, _, err := reduceToMovingOnly(req)
	test.That(t, err, test.ShouldBeNil)

	// The existing obstacle should still be present, plus crystallized geometries.
	var foundExisting bool
	for _, gf := range newReq.WorldState.Obstacles() {
		for _, g := range gf.Geometries() {
			if g.Label() == "existingObstacle" {
				foundExisting = true
			}
		}
	}
	test.That(t, foundExisting, test.ShouldBeTrue)
}

func TestReduceToMovingOnly_MultiWaypointUnion(t *testing.T) {
	fs := makeDualArmFS(t)
	startInputs := testZeroInputs(fs)

	// Two separate goals, each moving a different arm. The union should include both arms.
	req := &PlanRequest{
		FrameSystem: fs,
		Goals: []*PlanState{
			{poses: frame.FrameSystemPoses{
				"leftGripper": frame.NewPoseInFrame(frame.World, spatialmath.NewPoseFromPoint(r3.Vector{X: -500, Z: 200})),
			}},
			{poses: frame.FrameSystemPoses{
				"rightGripper": frame.NewPoseInFrame(frame.World, spatialmath.NewPoseFromPoint(r3.Vector{X: 500, Z: 200})),
			}},
		},
		StartState:     NewPlanState(nil, startInputs),
		WorldState:     frame.NewEmptyWorldState(),
		PlannerOptions: NewBasicPlannerOptions(),
	}

	newReq, restore, err := reduceToMovingOnly(req)
	test.That(t, err, test.ShouldBeNil)
	// Both arms are part of motion chains, so no reduction.
	test.That(t, restore, test.ShouldBeNil)
	test.That(t, newReq.FrameSystem, test.ShouldEqual, fs)
}

func TestReduceToMovingOnly_SingleArmRobot(t *testing.T) {
	// Single arm robot: nothing to reduce.
	fs := frame.NewEmptyFrameSystem("single_arm")

	arm, err := frame.NewTranslationalFrame("arm", r3.Vector{0, 0, 1}, frame.Limit{Min: -1000, Max: 1000})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, fs.AddFrame(arm, fs.World()), test.ShouldBeNil)

	startInputs := testZeroInputs(fs)
	req := &PlanRequest{
		FrameSystem: fs,
		Goals: []*PlanState{
			{poses: frame.FrameSystemPoses{
				"arm": frame.NewPoseInFrame(frame.World, spatialmath.NewPoseFromPoint(r3.Vector{Z: 200})),
			}},
		},
		StartState:     NewPlanState(nil, startInputs),
		WorldState:     frame.NewEmptyWorldState(),
		PlannerOptions: NewBasicPlannerOptions(),
	}

	newReq, restore, err := reduceToMovingOnly(req)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, restore, test.ShouldBeNil)
	test.That(t, newReq.FrameSystem, test.ShouldEqual, fs)
}

func TestReduceToMovingOnly_DescendantsRemoved(t *testing.T) {
	// Build a FS where the non-moving arm has descendants (a tool attached to the gripper).
	fs := makeDualArmFS(t)

	rightTool, err := frame.NewStaticFrame("rightTool", spatialmath.NewPoseFromPoint(r3.Vector{Z: 50}))
	test.That(t, err, test.ShouldBeNil)
	test.That(t, fs.AddFrame(rightTool, fs.Frame("rightGripper")), test.ShouldBeNil)

	startInputs := testZeroInputs(fs)
	req := &PlanRequest{
		FrameSystem: fs,
		Goals: []*PlanState{
			{poses: frame.FrameSystemPoses{
				"leftGripper": frame.NewPoseInFrame(frame.World, spatialmath.NewPoseFromPoint(r3.Vector{X: -500, Z: 200})),
			}},
		},
		StartState:     NewPlanState(nil, startInputs),
		WorldState:     frame.NewEmptyWorldState(),
		PlannerOptions: NewBasicPlannerOptions(),
	}

	newReq, restore, err := reduceToMovingOnly(req)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, restore, test.ShouldNotBeNil)

	// rightTool should also be removed since it's a descendant of rightArm.
	test.That(t, newReq.FrameSystem.Frame("rightTool"), test.ShouldBeNil)
	test.That(t, newReq.FrameSystem.Frame("rightArm"), test.ShouldBeNil)
}

func TestReduceToMovingOnly_NonMovingStaticFrameKept(t *testing.T) {
	// Ensure that 0-DOF frames not descended from a non-moving DOF frame are kept.
	fs := makeDualArmFS(t)

	// Add a camera directly under world (0 DOF, not a descendant of any arm).
	camera, err := frame.NewStaticFrame("camera", spatialmath.NewPoseFromPoint(r3.Vector{Y: 1000}))
	test.That(t, err, test.ShouldBeNil)
	test.That(t, fs.AddFrame(camera, fs.World()), test.ShouldBeNil)

	startInputs := testZeroInputs(fs)
	req := &PlanRequest{
		FrameSystem: fs,
		Goals: []*PlanState{
			{poses: frame.FrameSystemPoses{
				"leftGripper": frame.NewPoseInFrame(frame.World, spatialmath.NewPoseFromPoint(r3.Vector{X: -500, Z: 200})),
			}},
		},
		StartState:     NewPlanState(nil, startInputs),
		WorldState:     frame.NewEmptyWorldState(),
		PlannerOptions: NewBasicPlannerOptions(),
	}

	newReq, restore, err := reduceToMovingOnly(req)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, restore, test.ShouldNotBeNil)

	// The camera should be kept (0 DOF, not a descendant of a removed frame).
	test.That(t, newReq.FrameSystem.Frame("camera"), test.ShouldNotBeNil)
	// rightOffset is 0-DOF but is the parent of rightArm. Since rightArm is removed,
	// rightOffset itself should still be kept (it's not a descendant of rightArm).
	test.That(t, newReq.FrameSystem.Frame("rightOffset"), test.ShouldNotBeNil)
}

func TestReduceToMovingOnly_ReducedFSDOFCount(t *testing.T) {
	// Verify the reduced FS has fewer total DOF.
	fs := makeDualArmFS(t)
	startInputs := testZeroInputs(fs)

	req := &PlanRequest{
		FrameSystem: fs,
		Goals: []*PlanState{
			{poses: frame.FrameSystemPoses{
				"leftGripper": frame.NewPoseInFrame(frame.World, spatialmath.NewPoseFromPoint(r3.Vector{X: -500, Z: 200})),
			}},
		},
		StartState:     NewPlanState(nil, startInputs),
		WorldState:     frame.NewEmptyWorldState(),
		PlannerOptions: NewBasicPlannerOptions(),
	}

	newReq, _, err := reduceToMovingOnly(req)
	test.That(t, err, test.ShouldBeNil)

	// Count total DOF in original FS.
	origDOF := 0
	for _, name := range fs.FrameNames() {
		origDOF += len(fs.Frame(name).DoF())
	}

	// Count total DOF in reduced FS.
	reducedDOF := 0
	for _, name := range newReq.FrameSystem.FrameNames() {
		reducedDOF += len(newReq.FrameSystem.Frame(name).DoF())
	}

	// Original has 2 DOF (leftArm + rightArm), reduced should have 1 DOF (leftArm only).
	test.That(t, origDOF, test.ShouldEqual, 2)
	test.That(t, reducedDOF, test.ShouldEqual, 1)
}

func TestReduceAndExpandRoundTrip(t *testing.T) {
	fs := makeDualArmFS(t)
	startInputs := testZeroInputs(fs)
	startInputs["rightArm"] = []float64{42}
	startInputs["leftArm"] = []float64{5}

	req := &PlanRequest{
		FrameSystem: fs,
		Goals: []*PlanState{
			{poses: frame.FrameSystemPoses{
				"leftGripper": frame.NewPoseInFrame(frame.World, spatialmath.NewPoseFromPoint(r3.Vector{X: -500, Z: 200})),
			}},
		},
		StartState:     NewPlanState(nil, startInputs),
		WorldState:     frame.NewEmptyWorldState(),
		PlannerOptions: NewBasicPlannerOptions(),
	}

	newReq, restore, err := reduceToMovingOnly(req)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, restore, test.ShouldNotBeNil)

	// Simulate a 2-step trajectory in the reduced FS.
	step0 := newReq.StartState.LinearConfiguration()
	step1 := frame.NewLinearInputs()
	for k, v := range step0.Items() {
		step1.Put(k, v)
	}
	step1.Put("leftArm", []float64{100})

	traj := []*frame.LinearInputs{step0, step1}
	expanded := restore.expandTrajectory(traj)

	test.That(t, len(expanded), test.ShouldEqual, 2)

	// Check that the expanded trajectory has both arms' values.
	// Step 0: leftArm=5, rightArm=42
	left0 := expanded[0].Get("leftArm")
	test.That(t, left0, test.ShouldNotBeNil)
	test.That(t, left0[0], test.ShouldEqual, 5.0)
	right0 := expanded[0].Get("rightArm")
	test.That(t, right0, test.ShouldNotBeNil)
	test.That(t, right0[0], test.ShouldEqual, 42.0)

	// Step 1: leftArm=100 (from trajectory), rightArm=42 (from start state)
	left1 := expanded[1].Get("leftArm")
	test.That(t, left1, test.ShouldNotBeNil)
	test.That(t, left1[0], test.ShouldEqual, 100.0)
	right1 := expanded[1].Get("rightArm")
	test.That(t, right1, test.ShouldNotBeNil)
	test.That(t, right1[0], test.ShouldEqual, 42.0)

	// Verify expanded trajectory is valid against the original FS.
	for _, step := range expanded {
		_, err := step.ComputePoses(fs)
		test.That(t, err, test.ShouldBeNil)
	}
}

func TestReduceToMovingOnly_PlannerOptionsCarriedThrough(t *testing.T) {
	fs := makeDualArmFS(t)
	startInputs := testZeroInputs(fs)

	opts := NewBasicPlannerOptions()
	opts.Timeout = 42
	opts.LockNonmovingJoints = true

	constraints := &motionplan.Constraints{
		LinearConstraint: []motionplan.LinearConstraint{{LineToleranceMm: 1}},
	}

	req := &PlanRequest{
		FrameSystem: fs,
		Goals: []*PlanState{
			{poses: frame.FrameSystemPoses{
				"leftGripper": frame.NewPoseInFrame(frame.World, spatialmath.NewPoseFromPoint(r3.Vector{X: -500, Z: 200})),
			}},
		},
		StartState:     NewPlanState(nil, startInputs),
		WorldState:     frame.NewEmptyWorldState(),
		PlannerOptions: opts,
		Constraints:    constraints,
	}

	newReq, _, err := reduceToMovingOnly(req)
	test.That(t, err, test.ShouldBeNil)

	// Verify that planner options and constraints are preserved.
	test.That(t, newReq.PlannerOptions.Timeout, test.ShouldEqual, 42.0)
	test.That(t, newReq.PlannerOptions.LockNonmovingJoints, test.ShouldBeTrue)
	test.That(t, newReq.Constraints, test.ShouldEqual, constraints)
}
