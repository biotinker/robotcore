package armplanning

import (
	"fmt"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// restoreInfo holds the information needed to expand a reduced-DOF trajectory back to full DOF.
type restoreInfo struct {
	originalFS         *referenceframe.FrameSystem
	originalStartState *PlanState
}

// expandTrajectory takes a trajectory planned against a reduced FrameSystem and expands each step
// to include non-moving frame values from the original start state.
func (ri *restoreInfo) expandTrajectory(traj []*referenceframe.LinearInputs) []*referenceframe.LinearInputs {
	expanded := make([]*referenceframe.LinearInputs, len(traj))
	origLinear := ri.originalStartState.LinearConfiguration()

	for i, step := range traj {
		full := referenceframe.NewLinearInputs()
		// First, fill in all non-moving frames from the original start state.
		for k, v := range origLinear.Items() {
			full.Put(k, v)
		}
		// Then overwrite with moving frame values from this trajectory step.
		for k, v := range step.Items() {
			full.Put(k, v)
		}
		expanded[i] = full
	}
	return expanded
}

// reduceToMovingOnly replaces the FrameSystem in the PlanRequest with a reduced one containing
// only the frames involved in motion chains. Non-moving frames with nonzero DOF have their
// geometries crystallized at the start position and added as world-frame obstacles.
// Returns the modified request and restoreInfo for trajectory expansion, or nil restoreInfo if
// no reduction was needed.
func reduceToMovingOnly(req *PlanRequest) (*PlanRequest, *restoreInfo, error) {
	fullFS := req.FrameSystem

	// 1. Determine moving frames from all goals' motion chains.
	movingSet := map[string]bool{}
	for _, goal := range req.Goals {
		if len(goal.poses) == 0 {
			// Configuration-only goals don't define motion chains.
			continue
		}
		for frame, pif := range goal.poses {
			chain, err := motionChainFromGoal(fullFS, frame, pif.Parent())
			if err != nil {
				return nil, nil, fmt.Errorf("reduceToMovingOnly: %w", err)
			}
			for _, f := range chain.frames {
				movingSet[f.Name()] = true
			}
		}
	}

	// If no pose goals, nothing to reduce.
	if len(movingSet) == 0 {
		return req, nil, nil
	}

	// Also mark all ancestors of moving frames as moving (they stay in the reduced FS).
	ancestorSet := map[string]bool{}
	for name := range movingSet {
		frame := fullFS.Frame(name)
		if frame == nil {
			continue
		}
		traceback, err := fullFS.TracebackFrame(frame)
		if err != nil {
			return nil, nil, fmt.Errorf("reduceToMovingOnly traceback: %w", err)
		}
		for _, f := range traceback {
			ancestorSet[f.Name()] = true
		}
	}

	// 2. Determine which frames to remove: frames with DoF > 0 that are NOT moving and NOT ancestors of moving frames.
	// Also collect all descendants of removed frames for removal.
	removeSet := map[string]bool{}
	for _, frameName := range fullFS.FrameNames() {
		frame := fullFS.Frame(frameName)
		if frame == nil {
			continue
		}
		if movingSet[frameName] || ancestorSet[frameName] {
			continue
		}
		if len(frame.DoF()) > 0 {
			removeSet[frameName] = true
		}
	}

	// If nothing to remove, return the request unchanged.
	if len(removeSet) == 0 {
		return req, nil, nil
	}

	// Mark all descendants of removed frames for removal too.
	// FrameNames() is in BFS order, so parents appear before children.
	for _, frameName := range fullFS.FrameNames() {
		if removeSet[frameName] {
			continue
		}
		frame := fullFS.Frame(frameName)
		if frame == nil {
			continue
		}
		parent, err := fullFS.Parent(frame)
		if err != nil {
			continue
		}
		if parent != nil && removeSet[parent.Name()] {
			removeSet[frameName] = true
		}
	}

	// 3. Crystallize geometries of removed frames.
	startInputs := req.StartState.Configuration()
	fsGeometries, err := referenceframe.FrameSystemGeometries(fullFS, startInputs)
	if err != nil {
		return nil, nil, fmt.Errorf("reduceToMovingOnly geometry computation: %w", err)
	}

	var crystallizedGeoms []spatialmath.Geometry
	for frameName := range removeSet {
		if gif, ok := fsGeometries[frameName]; ok {
			crystallizedGeoms = append(crystallizedGeoms, gif.Geometries()...)
		}
	}

	// Merge crystallized geometries into existing WorldState.
	ws := req.WorldState
	existingObstacles := ws.Obstacles()
	existingTransforms := ws.Transforms()

	if len(crystallizedGeoms) > 0 {
		// Label geometries to avoid duplicate name issues.
		for i, g := range crystallizedGeoms {
			if g.Label() == "" {
				g.SetLabel(fmt.Sprintf("crystallized_%d", i))
			}
		}
		crystallizedGIF := referenceframe.NewGeometriesInFrame(referenceframe.World, crystallizedGeoms)
		allObstacles := append(existingObstacles, crystallizedGIF)
		ws, err = referenceframe.NewWorldState(allObstacles, existingTransforms)
		if err != nil {
			return nil, nil, fmt.Errorf("reduceToMovingOnly world state: %w", err)
		}
	}

	// 4. Build reduced FrameSystem.
	// FrameNames() returns BFS order (parents before children), so we can add frames in order.
	reducedFS := referenceframe.NewEmptyFrameSystem(fullFS.Name() + "_reduced")
	for _, frameName := range fullFS.FrameNames() {
		if removeSet[frameName] {
			continue
		}
		frame := fullFS.Frame(frameName)
		parent, err := fullFS.Parent(frame)
		if err != nil {
			return nil, nil, fmt.Errorf("reduceToMovingOnly parent lookup for %s: %w", frameName, err)
		}
		// Map the parent: if it's the original world, use the new world.
		var newParent referenceframe.Frame
		if parent.Name() == referenceframe.World {
			newParent = reducedFS.World()
		} else {
			newParent = reducedFS.Frame(parent.Name())
		}
		if newParent == nil {
			// Parent was removed; this frame should have been removed too. Skip.
			continue
		}
		if err := reducedFS.AddFrame(frame, newParent); err != nil {
			return nil, nil, fmt.Errorf("reduceToMovingOnly add frame %s: %w", frameName, err)
		}
	}

	// 5. Filter StartState to only frames present in reduced FS.
	reducedConfig := referenceframe.FrameSystemInputs{}
	for fName, inputs := range req.StartState.Configuration() {
		if reducedFS.Frame(fName) != nil {
			reducedConfig[fName] = inputs
		}
	}
	reducedStartState := NewPlanState(req.StartState.Poses(), reducedConfig)

	// 6. Build modified request.
	newReq := &PlanRequest{
		FrameSystem:    reducedFS,
		Goals:          req.Goals,
		StartState:     reducedStartState,
		WorldState:     ws,
		Constraints:    req.Constraints,
		PlannerOptions: req.PlannerOptions,
		myTestOptions:  req.myTestOptions,
	}

	restore := &restoreInfo{
		originalFS:         fullFS,
		originalStartState: req.StartState,
	}

	return newReq, restore, nil
}
