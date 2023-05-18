package referenceframe

import (
	"fmt"
	"math"
	"strings"

	"github.com/golang/geo/r3"
	pb "go.viam.com/api/component/arm/v1"

	spatial "go.viam.com/rdk/spatialmath"
)


type TimeFrame struct {
	*baseFrame
	pose spatial.Pose
	geometry spatial.Geometry
}

// NewTimeFrame instantiates a frame that has forwards linear velocity, yaw angular velocity, and time that moves from some pose
// mm/s, rad/s, s
func NewTimeFrame(name string, limits []Limit, geometry spatial.Geometry, pose spatial.Pose) (Frame, error) {
	if len(limits) != 3 {
		return nil, fmt.Errorf("cannot create a %d dof time frame, only support 3 dimensions currently", len(limits))
	}
	return &TimeFrame{baseFrame: &baseFrame{name: name, limits: limits}, pose: pose, geometry: geometry}, nil
}

func (tf *TimeFrame) Transform(input []Input) (spatial.Pose, error) {
	
	linvel := input[0].Value
	angvel := input[1].Value
	secs := input[2].Value
	
	// if linear velocity is 0, we spin
	if linvel == 0 {
		return spatial.Compose(tf.pose,spatial.NewPoseFromOrientation(&spatial.OrientationVector{OZ: 1, Theta: angvel * secs})), nil
	}
	// if angular velocity is 0, we move straight. Straight for bases is Y
	if angvel == 0 {
		return spatial.Compose(tf.pose, spatial.NewPoseFromPoint(r3.Vector{Y: linvel * secs})), nil
	}
	// Otherwise we go some amount of the way along some circle whose radius is determined by angvel, linvel, and time
	secsFor360 := 2./angvel // seconds to complete one ful rotation
	circumference := secsFor360 * linvel // circumference of circle
	dist := math.Mod((secs * linvel), circumference)
	theta := (dist/circumference) * 2 * math.Pi // ending angle about our circle
	radius := circumference/(2 * math.Pi)
	// positive angvel turns us towards -x
	cirCenter := r3.Vector{X: math.Copysign(1, angvel) * -1 * radius}
	// Our ending point on the unit circle
	unitCirPt := r3.Vector{math.Copysign(math.Cos(theta), angvel), math.Sin(theta), 0}
	framePose := spatial.NewPose(unitCirPt.Mul(radius).Add(cirCenter), &spatial.OrientationVector{OZ: 1, Theta: math.Copysign(theta, angvel)})
	return spatial.Compose(tf.pose, framePose), nil
}

// InputFromProtobuf converts pb.JointPosition to inputs.
func (tf *TimeFrame) InputFromProtobuf(jp *pb.JointPositions) []Input {
	n := make([]Input, len(jp.Values))
	for idx, d := range jp.Values {
		n[idx] = Input{d}
	}
	return n
}

// ProtobufFromInput converts inputs to pb.JointPosition.
func (tf *TimeFrame) ProtobufFromInput(input []Input) *pb.JointPositions {
	n := make([]float64, len(input))
	for idx, a := range input {
		n[idx] = a.Value
	}
	return &pb.JointPositions{Values: n}
}

func (tf *TimeFrame) Geometries(input []Input) (*GeometriesInFrame, error) {
	if tf.geometry == nil {
		return NewGeometriesInFrame(tf.Name(), nil), nil
	}
	pose, err := tf.Transform(input)
	if pose == nil || (err != nil && !strings.Contains(err.Error(), OOBErrString)) {
		return nil, err
	}
	return NewGeometriesInFrame(tf.name, []spatial.Geometry{tf.geometry.Transform(pose)}), err
}

func (tf *TimeFrame) MarshalJSON() ([]byte, error) {
	return nil, fmt.Errorf("MarshalJSON not implemented for type %T", tf)
}

func (tf *TimeFrame) AlmostEquals(otherFrame Frame) bool {
	other, ok := otherFrame.(*TimeFrame)
	return ok && tf.baseFrame.AlmostEquals(other.baseFrame)
}

func (tf *TimeFrame) AtNewPose(pose spatial.Pose) *TimeFrame {
	return &TimeFrame{baseFrame: tf.baseFrame, pose: pose, geometry: tf.geometry}
}
