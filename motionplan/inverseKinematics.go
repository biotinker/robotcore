package motionplan

import (
	"context"

	"go.viam.com/rdk/referenceframe"
)

// InverseKinematics defines an interface which, provided with seed inputs and a Metric to minimize to zero, will output all found
// solutions to the provided channel until cancelled or otherwise completes.
type InverseKinematics interface {
	// Solve receives a context, the goal arm position, and current joint angles.
	Solve(context.Context, chan<- *IKSolution, []referenceframe.Input, StateMetric, int) error
}

// IKSolution is the struct returned from an IK solver. It contains the solution configuration, the score of the solution, and a flag
// indicating whether that configuration and score met the solution criteria requested by the caller.
type IKSolution struct {
	Configuration []referenceframe.Input
	Score         float64
	Partial       bool
}

func limitsToArrays(limits []referenceframe.Limit) ([]float64, []float64) {
	var min, max []float64
	for _, limit := range limits {
		min = append(min, limit.Min)
		max = append(max, limit.Max)
	}
	return min, max
}
