//go:build !windows

package motionplan

import (
	"fmt"
	"context"
	"math/rand"
	"testing"
	//~ "math"

	"github.com/edaniels/golog"
	"github.com/golang/geo/r3"
	"go.viam.com/test"

	//~ "go.viam.com/rdk/motionplan/tpspace"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

const testTurnRad = 0.3

func TestPtgRrt(t *testing.T) {
	logger := golog.NewTestLogger(t)
	roverGeom, err := spatialmath.NewBox(spatialmath.NewZeroPose(), r3.Vector{10, 10, 10}, "")
	test.That(t, err, test.ShouldBeNil)
	geometries := []spatialmath.Geometry{roverGeom}

	ackermanFrame, err := NewPTGFrameFromTurningRadius(
		"ackframe",
		logger,
		300.,
		testTurnRad,
		0,
		geometries,
	)
	test.That(t, err, test.ShouldBeNil)

	goalPos := spatialmath.NewPose(r3.Vector{X: 100, Y: 700, Z: 0}, &spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 180})

	opt := newBasicPlannerOptions()
	opt.SetGoalMetric(NewPositionOnlyMetric(goalPos))
	opt.DistanceFunc = SquaredNormNoOrientSegmentMetric
	//~ opt.SetGoalMetric(NewSquaredNormMetric(goalPos))
	//~ opt.DistanceFunc = SquaredNormSegmentMetric
	opt.GoalThreshold = 5.
	mp, err := newTPSpaceMotionPlanner(ackermanFrame, rand.New(rand.NewSource(42)), logger, opt)
	test.That(t, err, test.ShouldBeNil)
	tp, ok := mp.(*tpSpaceRRTMotionPlanner)
	test.That(t, ok, test.ShouldBeTrue)

	plan, err := tp.plan(context.Background(), goalPos, nil)
	for _, wp := range plan {
		fmt.Println(wp.Q())
	}
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(plan), test.ShouldBeGreaterThanOrEqualTo, 2)
}

func TestPtgWithObstacle(t *testing.T) {
	logger := golog.NewTestLogger(t)
	roverGeom, err := spatialmath.NewBox(spatialmath.NewZeroPose(), r3.Vector{10, 10, 10}, "")
	test.That(t, err, test.ShouldBeNil)
	geometries := []spatialmath.Geometry{roverGeom}
	ackermanFrame, err := NewPTGFrameFromTurningRadius(
		"ackframe",
		logger,
		300.,
		testTurnRad,
		0,
		geometries,
	)
	test.That(t, err, test.ShouldBeNil)

	ctx := context.Background()

	goalPos := spatialmath.NewPoseFromPoint(r3.Vector{X: 5000, Y: 0, Z: 0})

	fs := referenceframe.NewEmptyFrameSystem("test")
	fs.AddFrame(ackermanFrame, fs.World())

	opt := newBasicPlannerOptions()
	opt.SetGoalMetric(NewPositionOnlyMetric(goalPos))
	opt.DistanceFunc = SquaredNormNoOrientSegmentMetric
	//~ opt.SetGoalMetric(NewSquaredNormMetric(goalPos))
	//~ opt.DistanceFunc = SquaredNormSegmentMetric
	opt.GoalThreshold = 5.
	// obstacles
	obstacle1, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{2500, -500, 0}), r3.Vector{180, 1800, 1}, "")
	test.That(t, err, test.ShouldBeNil)
	obstacle2, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{2500, 2000, 0}), r3.Vector{180, 1800, 1}, "")
	test.That(t, err, test.ShouldBeNil)
	obstacle3, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{2500, -1400, 0}), r3.Vector{50000, 30, 1}, "")
	test.That(t, err, test.ShouldBeNil)
	obstacle4, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{2500, 2400, 0}), r3.Vector{50000, 30, 1}, "")
	test.That(t, err, test.ShouldBeNil)

	geoms := []spatialmath.Geometry{obstacle1, obstacle2, obstacle3, obstacle4}

	//~ fmt.Println("type,X,Y")
	//~ for _, geom := range geoms {
	//~ pts := geom.ToPoints(1.)
	//~ for _, pt := range pts {
	//~ if math.Abs(pt.Z) < 0.1 {
	//~ fmt.Printf("OBS,%f,%f\n", pt.X, pt.Y)
	//~ }
	//~ }
	//~ }

	worldState, err := referenceframe.NewWorldState(
		[]*referenceframe.GeometriesInFrame{referenceframe.NewGeometriesInFrame(referenceframe.World, geoms)},
		nil,
	)
	test.That(t, err, test.ShouldBeNil)
	sf, err := newSolverFrame(fs, ackermanFrame.Name(), referenceframe.World, nil)
	test.That(t, err, test.ShouldBeNil)
	collisionConstraints, err := createAllCollisionConstraints(sf, fs, worldState, referenceframe.StartPositions(fs), nil)
	test.That(t, err, test.ShouldBeNil)

	for name, constraint := range collisionConstraints {
		opt.AddStateConstraint(name, constraint)
	}

	mp, err := newTPSpaceMotionPlanner(ackermanFrame, rand.New(rand.NewSource(42)), logger, opt)
	test.That(t, err, test.ShouldBeNil)
	tp, _ := mp.(*tpSpaceRRTMotionPlanner)

	plan, err := tp.plan(ctx, goalPos, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(plan), test.ShouldBeGreaterThan, 2)
	for _, wp := range plan {
		fmt.Println(wp.Q())
	}
	
	//~ allPtgs := ackermanFrame.(tpspace.PTGProvider).PTGs()
	//~ lastPose := spatialmath.NewZeroPose()
	
	//~ for _, mynode := range plan {
		//~ trajPts, _ := allPtgs[int(mynode.Q()[0].Value)].Trajectory(mynode.Q()[1].Value, mynode.Q()[2].Value)
		//~ for i, pt := range trajPts {
			//~ fmt.Println("pt", pt)
			//~ intPose := spatialmath.Compose(lastPose, pt.Pose)
			//~ if i == 0 {
				//~ fmt.Printf("WP,%f,%f\n", intPose.Point().X, intPose.Point().Y)
			//~ }
			//~ fmt.Printf("FINALPATH,%f,%f\n", intPose.Point().X, intPose.Point().Y)
			//~ if i == len(trajPts) - 1 {
				//~ lastPose = spatialmath.Compose(lastPose, pt.Pose)
				//~ break
			//~ }
		//~ }
	//~ }
}


func TestIKPtgRrt(t *testing.T) {
	logger := golog.NewTestLogger(t)
	roverGeom, err := spatialmath.NewBox(spatialmath.NewZeroPose(), r3.Vector{10, 10, 10}, "")
	test.That(t, err, test.ShouldBeNil)
	geometries := []spatialmath.Geometry{roverGeom}

	ackermanFrame, err := NewPTGFrameFromTurningRadius(
		"ackframe",
		logger,
		300.,
		testTurnRad,
		0,
		geometries,
	)
	test.That(t, err, test.ShouldBeNil)

	goalPos := spatialmath.NewPose(r3.Vector{X: 50, Y: 10, Z: 0}, &spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 180})

	opt := newBasicPlannerOptions()
	opt.SetGoalMetric(NewPositionOnlyMetric(goalPos))
	opt.DistanceFunc = SquaredNormNoOrientSegmentMetric
	opt.GoalThreshold = 10.
	mp, err := newTPSpaceMotionPlanner(ackermanFrame, rand.New(rand.NewSource(42)), logger, opt)
	test.That(t, err, test.ShouldBeNil)
	tp, ok := mp.(*tpSpaceRRTMotionPlanner)
	test.That(t, ok, test.ShouldBeTrue)

	plan, err := tp.plan(context.Background(), goalPos, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(plan), test.ShouldBeGreaterThanOrEqualTo, 2)
}
