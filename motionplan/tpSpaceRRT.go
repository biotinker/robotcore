//go:build !windows

package motionplan

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/rand"

	"github.com/edaniels/golog"
	"github.com/golang/geo/r3"
	"go.viam.com/utils"

	"go.viam.com/rdk/motionplan/tpspace"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

const (
	defaultGoalCheck = 5  // Check if the goal is reachable every this many iterations
	defaultAutoBB    = 0.3 // Automatic bounding box on driveable area as a multiple of start-goal distance
	// Note: while fully holonomic planners can use the limits of the frame as implicit boundaries, with non-holonomic motion
	// this is not the case, and the total workspace available to the planned frame is not directly related to the motion available
	// from a single set of inputs.

	// whether to add intermediate waypoints.
	defaultAddInt = true
	// Add a subnode every this many mm along a valid trajectory. Large values run faster, small gives better paths
	// Meaningless if the above is false.
	defaultAddNodeEvery = 100.

	// Don't add new RRT tree nodes if there is an existing node within this distance.
	// Consider nodes on trees to be connected if they are within this distance.
	defaultIdenticalNodeDistance = 1.

	defaultMaxReseeds = 50
)

type tpspaceOptions struct {
	*rrtOptions
	goalCheck int // Check if goal is reachable every this many iters

	// TODO: base this on frame limits?
	autoBB float64 // Automatic bounding box on driveable area as a multiple of start-goal distance

	addIntermediate bool // whether to add intermediate waypoints.
	// Add a subnode every this many mm along a valid trajectory. Large values run faster, small gives better paths
	// Meaningless if the above is false.
	addNodeEvery float64

	
	// If the squared norm between two poses is less than this, consider them equal
	poseSolveDist float64

	// Cached functions for calculating TP-space distances for each PTG
	distOptions map[tpspace.PTG]*plannerOptions
	invertDistOptions map[tpspace.PTG]*plannerOptions
}

// candidate is putative node which could be added to an RRT tree. It includes a distance score, the new node and its future parent.
type candidate struct {
	dist       float64
	treeNode   node
	newNode    node
	err        error
	lastInTraj bool
}

type nodeAndError struct {
	node
	error
}

// tpSpaceRRTMotionPlanner.
type tpSpaceRRTMotionPlanner struct {
	*planner
	algOpts *tpspaceOptions
	tpFrame tpspace.PTGProvider
}

// newTPSpaceMotionPlanner creates a newTPSpaceMotionPlanner object with a user specified random seed.
func newTPSpaceMotionPlanner(
	frame referenceframe.Frame,
	seed *rand.Rand,
	logger golog.Logger,
	opt *plannerOptions,
) (motionPlanner, error) {
	if opt == nil {
		return nil, errNoPlannerOptions
	}
	mp, err := newPlanner(frame, seed, logger, opt)
	if err != nil {
		return nil, err
	}

	tpFrame, ok := mp.frame.(tpspace.PTGProvider)
	if !ok {
		return nil, fmt.Errorf("frame %v must be a PTGProvider", mp.frame)
	}

	tpPlanner := &tpSpaceRRTMotionPlanner{
		planner: mp,
		tpFrame: tpFrame,
	}
	tpPlanner.setupTPSpaceOptions()
	return tpPlanner, nil
}

// TODO: seed is not immediately useful for TP-space.
func (mp *tpSpaceRRTMotionPlanner) plan(ctx context.Context,
	goal spatialmath.Pose,
	seed []referenceframe.Input,
) ([]node, error) {
	solutionChan := make(chan *rrtPlanReturn, 1)

	seedPos := spatialmath.NewZeroPose()

	startNode := &basicNode{q: make([]referenceframe.Input, len(mp.frame.DoF())), cost: 0, pose: seedPos, corner: false}
	goalNode := &basicNode{q: make([]referenceframe.Input, len(mp.frame.DoF())), cost: 0, pose: goal, corner: false}

	utils.PanicCapturingGo(func() {
		mp.planRunner(ctx, seed, &rrtParallelPlannerShared{
			&rrtMaps{
				startMap: map[node]node{startNode: nil},
				goalMap:  map[node]node{goalNode: nil},
			},
			nil,
			solutionChan,
		})
	})
	select {
	case <-ctx.Done():
		return nil, ctx.Err()
	case plan := <-solutionChan:
		if plan != nil {
			return plan.steps, plan.err()
		}
		return nil, errors.New("nil tp-space plan returned, unable to complete plan")
	}
}

// planRunner will execute the plan. Plan() will call planRunner in a separate thread and wait for results.
// Separating this allows other things to call planRunner in parallel allowing the thread-agnostic Plan to be accessible.
func (mp *tpSpaceRRTMotionPlanner) planRunner(
	ctx context.Context,
	_ []referenceframe.Input, // TODO: this may be needed for smoothing
	rrt *rrtParallelPlannerShared,
) {
	defer close(rrt.solutionChan)

	// get start and goal poses
	var startPose spatialmath.Pose
	var goalPose spatialmath.Pose
	for k, v := range rrt.maps.startMap {
		if v == nil {
			if k.Pose() != nil {
				startPose = k.Pose()
			} else {
				rrt.solutionChan <- &rrtPlanReturn{planerr: fmt.Errorf("node %v must provide a Pose", k)}
				return
			}
			break
		}
	}
	for k, v := range rrt.maps.goalMap {
		if v == nil {
			if k.Pose() != nil {
				goalPose = k.Pose()
			} else {
				rrt.solutionChan <- &rrtPlanReturn{planerr: fmt.Errorf("node %v must provide a Pose", k)}
				return
			}
			break
		}
	}
	
	bidirectional := true

	m1chan := make(chan *nodeAndError, 1)
	m2chan := make(chan *nodeAndError, 1)
	defer close(m1chan)
	defer close(m2chan)

	dist := math.Sqrt(mp.planOpts.DistanceFunc(&Segment{StartPosition: startPose, EndPosition: goalPose}))
	midPt := spatialmath.Interpolate(startPose, goalPose, 0.5) // used for initial seed

	var randPos spatialmath.Pose
	//~ for iter := 0; iter < mp.algOpts.PlanIter; iter++ {
	for iter := 0; iter < 1; iter++ {
		fmt.Println("iter", iter)
		if ctx.Err() != nil {
			mp.logger.Debugf("TP Space RRT timed out after %d iterations", iter)
			rrt.solutionChan <- &rrtPlanReturn{planerr: fmt.Errorf("TP Space RRT timeout %w", ctx.Err()), maps: rrt.maps}
			return
		}
		// Get random cartesian configuration
		rDist := dist * (mp.algOpts.autoBB + float64(iter)/10.)
		randPosX := float64(mp.randseed.Intn(int(rDist)))
		randPosY := float64(mp.randseed.Intn(int(rDist)))
		randPosTheta := math.Pi * (mp.randseed.Float64() - 0.5)
		randPos = spatialmath.NewPose(
			r3.Vector{midPt.Point().X + (randPosX - rDist/2.), midPt.Point().Y + (randPosY - rDist/2.), 0},
			&spatialmath.OrientationVector{OZ: 1, Theta: randPosTheta},
		)
		if iter == 0 {
			randPos = midPt
		}
		if !bidirectional && iter % 5 == 0 {
			randPos = goalPose
		}
		randPosNode := &basicNode{pose: randPos}

		utils.PanicCapturingGo(func() {
			m1chan <- mp.attemptExtension(ctx, nil, randPosNode, rrt.maps.startMap, false)
		})
		utils.PanicCapturingGo(func() {
			m2chan <- mp.attemptExtension(ctx, nil, randPosNode, rrt.maps.goalMap, true)
			
		})
		seedMapReached := <-m1chan
		goalMapReached := <-m2chan
		
		seedMapNode := seedMapReached.node
		goalMapNode := goalMapReached.node
		err := errors.Join(seedMapReached.error, goalMapReached.error)
		
		if err != nil {
			rrt.solutionChan <- &rrtPlanReturn{planerr: err, maps: rrt.maps}
			return
		}
		
		
		if err != nil {
			rrt.solutionChan <- &rrtPlanReturn{planerr: err, maps: rrt.maps}
			return
		}
		
		if seedMapNode != nil && goalMapNode != nil {
		
			seedReached := mp.attemptExtension(ctx, nil, goalMapNode, rrt.maps.startMap, false)
			if seedReached.error != nil {
				rrt.solutionChan <- &rrtPlanReturn{planerr: seedReached.error, maps: rrt.maps}
				return
			}
			if seedReached.node == nil {
				continue
			}
			goalReached := mp.attemptExtension(ctx, nil, seedReached.node, rrt.maps.goalMap, true)
			if goalReached.error != nil {
				rrt.solutionChan <- &rrtPlanReturn{planerr: goalReached.error, maps: rrt.maps}
				return
			}
			if goalReached.node == nil {
				continue
			}
			reachedDelta := mp.planOpts.DistanceFunc(&Segment{StartPosition: seedReached.node.Pose(), EndPosition: goalReached.node.Pose()})
			fmt.Println("reached seed", spatialmath.PoseToProtobuf(seedReached.node.Pose()))
			fmt.Println("reached goal", spatialmath.PoseToProtobuf(goalReached.node.Pose()))
			if reachedDelta <= mp.algOpts.poseSolveDist {
				// If we've reached the goal, extract the path from the RRT trees and return
				path := extractPath(rrt.maps.startMap, rrt.maps.goalMap, &nodePair{a: seedReached.node, b: goalReached.node}, false)
				rrt.solutionChan <- &rrtPlanReturn{steps: path, maps: rrt.maps}
				return
			}
		}
	}
	rrt.solutionChan <- &rrtPlanReturn{maps: rrt.maps, planerr: errors.New("tpspace RRT unable to create valid path")}
}

// getExtensionCandidate will return either nil, or the best node on a valid PTG to reach the desired random node and its RRT tree parent.
func (mp *tpSpaceRRTMotionPlanner) getExtensionCandidate(
	ctx context.Context,
	randPosNode node,
	ptgNum int,
	curPtg tpspace.PTG,
	rrt rrtMap,
	nearest node,
	planOpts *plannerOptions,
	invert bool,
) (*candidate, error) {
	nm := &neighborManager{nCPU: mp.planOpts.NumThreads/len(mp.tpFrame.PTGs())}
	nm.parallelNeighbors = 10

	var successNode node
	// Get the distance function that will find the nearest RRT map node in TP-space of *this* PTG
	ptgDistOpt := mp.algOpts.distOptions[curPtg]
	if invert {
		ptgDistOpt = mp.algOpts.invertDistOptions[curPtg]
	}

	if nearest == nil {
		// Get nearest neighbor to rand config in tree using this PTG
		nearest = nm.nearestNeighbor(ctx, ptgDistOpt, randPosNode, rrt)
		if nearest == nil {
			return nil, nil
		}
	}

	// Get cartesian distance from NN to rand
	var targetFunc func(spatialmath.Pose) float64
	if invert {
		sqMet := NewSquaredNormMetric(randPosNode.Pose())
		targetFunc = func(pose spatialmath.Pose) float64 {
			return sqMet(&State{Position: spatialmath.Compose(nearest.Pose(), spatialmath.PoseInverse(pose))})
		}
		
	} else {
		relPose := spatialmath.Compose(spatialmath.PoseInverse(nearest.Pose()), randPosNode.Pose())
		sqMet := NewSquaredNormMetric(relPose)
		targetFunc = func(pose spatialmath.Pose) float64 {
			return sqMet(&State{Position: pose})
		}
	}
	
	// Convert cartesian distance to tp-space using inverse curPtg, yielding TP-space coordinates goalK and goalD
	bestNode, err := curPtg.CToTP(ctx, targetFunc)
	if err != nil || bestNode == nil {
		return nil, err
	}
	bestDist := targetFunc(bestNode.Pose)
	goalAlpha := bestNode.Alpha
	goalD := bestNode.Dist
	
	// Check collisions along this traj and get the longest distance viable
	trajK, err := curPtg.Trajectory(goalAlpha, goalD)
	if err != nil {
		return nil, err
	}
	finalTrajNode := trajK[len(trajK) - 1]

	arcStartPose := nearest.Pose()
	if invert {
		arcStartPose = spatialmath.Compose(arcStartPose, spatialmath.PoseInverse(finalTrajNode.Pose))
	}

	sinceLastCollideCheck := 0.
	lastDist := 0.
	var nodePose spatialmath.Pose
	// Check each point along the trajectory to confirm constraints are met
	for i := 0; i < len(trajK); i++ {
		trajPt := trajK[i]
		if invert {
			// Start at known-good map point and extend
			// For the goal tree this means iterating backwards
			trajPt = trajK[(len(trajK)-1)-i]
		}

		sinceLastCollideCheck += math.Abs(trajPt.Dist - lastDist)
		trajState := &State{Position: spatialmath.Compose(arcStartPose, trajPt.Pose), Frame: mp.frame}
		nodePose = trajState.Position // This will get rewritten later for inverted trees
		if sinceLastCollideCheck > planOpts.Resolution {
			ok, _ := planOpts.CheckStateConstraints(trajState)
			if !ok {
				return nil, nil
			}
			sinceLastCollideCheck = 0.
		}

		lastDist = trajPt.Dist
	}
	
	isLastNode := math.Abs(finalTrajNode.Dist - curPtg.RefDistance()) < 1e-3
	
	// add the last node in trajectory
	successNode = &basicNode{
		q:      referenceframe.FloatsToInputs([]float64{float64(ptgNum), goalAlpha, finalTrajNode.Dist}),
		cost:   finalTrajNode.Dist,
		pose:   nodePose,
		corner: false,
	}

	cand := &candidate{dist: bestDist, treeNode: nearest, newNode: successNode, lastInTraj: isLastNode}

	return cand, nil
}

// attemptExtension will attempt to extend the rrt map towards the goal node, and will return the candidate added to the map that is
// closest to that goal.
func (mp *tpSpaceRRTMotionPlanner) attemptExtension(
	ctx context.Context,
	seedNode,
	goalNode node,
	rrt rrtMap,
	invert bool,
) *nodeAndError {
	var reseedCandidate *candidate
	maxReseeds := 1 // Will be updated as necessary
	for i := 0; i <= maxReseeds; i++ {
		select {
		case <-ctx.Done():
			return &nodeAndError{nil, ctx.Err()}
		default:
		}
		candidates := []*candidate{}
		
		candChan := make(chan *candidate, len(mp.tpFrame.PTGs()))
		defer close(candChan)
		for ptgNum, curPtg := range mp.tpFrame.PTGs() {
			// Find the best traj point for each traj family, and store for later comparison
			ptgNumPar, curPtgPar := ptgNum, curPtg
			utils.PanicCapturingGo(func() {
				
				cand, err := mp.getExtensionCandidate(ctx, goalNode, ptgNumPar, curPtgPar, rrt, seedNode, mp.planOpts, invert)
				
				if err != nil {
					candChan <- nil
					return
				}
				if cand != nil {
					if cand.err == nil {
						candChan <- cand
						return
					}
				}
				candChan <- nil
			})
		}
		
		for i := 0; i < len(mp.tpFrame.PTGs()); i++ {
			select {
			case <-ctx.Done():
				return &nodeAndError{nil, ctx.Err()}
			case cand := <-candChan:
				if cand != nil {
					candidates = append(candidates, cand)
				}
			}
		}
		var err error
		reseedCandidate, err = mp.extendMap(ctx, candidates, rrt, invert)
		if err != nil {
			return &nodeAndError{nil, err}
		}
		if reseedCandidate == nil {
			return &nodeAndError{nil, nil}
		}
		dist := mp.planOpts.DistanceFunc(&Segment{StartPosition: reseedCandidate.newNode.Pose(), EndPosition: goalNode.Pose()})
		if dist < mp.algOpts.poseSolveDist || !reseedCandidate.lastInTraj {
			// Reached the goal position, or otherwise failed to fully extend to the end of a trajectory
			return &nodeAndError{reseedCandidate.newNode, nil}
		}
		if i == 0 {
			dist = math.Sqrt(dist)
			// TP-space distance is NOT the same thing as cartesian distance, but they track sufficiently well that this is valid to do.
			maxReseeds = int(math.Min(float64(defaultMaxReseeds), math.Ceil(dist / reseedCandidate.newNode.Q()[2].Value) + 2))
			
		}
		seedNode = reseedCandidate.newNode
	}
	return &nodeAndError{reseedCandidate.newNode, nil}
}

// extendMap grows the rrt map to the best candidate node, returning the added candidate.
func (mp *tpSpaceRRTMotionPlanner) extendMap(
	ctx context.Context,
	candidates []*candidate,
	rrt rrtMap,
	invert bool,
) (*candidate, error) {
	if len(candidates) == 0 {
		return nil, nil
	}
	var addedNode node
	// If we found any valid nodes that we can extend to, find the very best one and add that to the tree
	bestDist := math.Inf(1)
	var bestCand *candidate
	for _, cand := range candidates {
		if cand.dist < bestDist {
			bestCand = cand
			bestDist = cand.dist
		}
	}
	treeNode := bestCand.treeNode // The node already in the tree to which we are parenting
	newNode := bestCand.newNode // The node we are adding because it was the best extending PTG

	ptgNum := int(newNode.Q()[0].Value)
	randAlpha := newNode.Q()[1].Value
	randDist := newNode.Q()[2].Value

	//~ fmt.Println("__ADDINGtraj", ptgNum, randAlpha, randDist)
	trajK, err := mp.tpFrame.PTGs()[ptgNum].Trajectory(randAlpha, randDist)
	if err != nil {
		return nil, err
	}

	arcStartPose := treeNode.Pose()
	if invert {
		arcStartPose = spatialmath.Compose(arcStartPose, spatialmath.PoseInverse(trajK[len(trajK) - 1].Pose))
	}
	lastDist := 0.
	sinceLastNode := 0.

	var trajState *State
	if mp.algOpts.addIntermediate {
		for i := 0; i < len(trajK); i++ {
			trajPt := trajK[i]
			if invert {
				trajPt = trajK[(len(trajK)-1)-i]
			}
			if ctx.Err() != nil {
				return nil, ctx.Err()
			}
			trajState = &State{Position: spatialmath.Compose(arcStartPose, trajPt.Pose)}
			if !invert {
				fmt.Printf("FINALPATH1,%f,%f\n", trajState.Position.Point().X, trajState.Position.Point().Y)
			}else{
				fmt.Printf("FINALPATH2,%f,%f\n", trajState.Position.Point().X, trajState.Position.Point().Y)
			}
			sinceLastNode += (trajPt.Dist - lastDist)

			// Optionally add sub-nodes along the way. Will make the final path a bit better
			// Intermediate nodes currently disabled on the goal map because they do not invert nicely
			if sinceLastNode > mp.algOpts.addNodeEvery && !invert {
				// add the last node in trajectory
				addedNode = &basicNode{
					q:      referenceframe.FloatsToInputs([]float64{float64(ptgNum), randAlpha, trajPt.Dist}),
					cost:   trajPt.Dist,
					pose:   trajState.Position,
					corner: false,
				}
				rrt[addedNode] = treeNode
				sinceLastNode = 0.
			}
			lastDist = trajPt.Dist
		}
		fmt.Printf("WPI,%f,%f\n", trajState.Position.Point().X, trajState.Position.Point().Y)
	}
	rrt[newNode] = treeNode
	return bestCand, nil
}

func (mp *tpSpaceRRTMotionPlanner) setupTPSpaceOptions() {
	tpOpt := &tpspaceOptions{
		rrtOptions: newRRTOptions(),
		goalCheck:  defaultGoalCheck,
		autoBB:     defaultAutoBB,

		addIntermediate: defaultAddInt,
		addNodeEvery:    defaultAddNodeEvery,

		poseSolveDist: defaultIdenticalNodeDistance,
		
		distOptions: map[tpspace.PTG]*plannerOptions{},
		invertDistOptions: map[tpspace.PTG]*plannerOptions{},
	}

	for _, curPtg := range mp.tpFrame.PTGs() {
		tpOpt.distOptions[curPtg] = mp.make2DTPSpaceDistanceOptions(curPtg, false)
		tpOpt.invertDistOptions[curPtg] = mp.make2DTPSpaceDistanceOptions(curPtg, true)
	}

	mp.algOpts = tpOpt
}

// make2DTPSpaceDistanceOptions will create a plannerOptions object with a custom DistanceFunc constructed such that
// distances can be computed in TP space using the given PTG.
func (mp *tpSpaceRRTMotionPlanner) make2DTPSpaceDistanceOptions(ptg tpspace.PTG, invert bool) *plannerOptions {
	opts := newBasicPlannerOptions()

	segMetric := func(seg *Segment) float64 {
		// When running NearestNeighbor:
		// StartPosition is the seed/query
		// EndPosition is the pose already in the RRT tree
		if seg.StartPosition == nil || seg.EndPosition == nil {
			return math.Inf(1)
		}
		var targetFunc func(spatialmath.Pose) float64
		if invert {
			sqMet := NewSquaredNormMetric(seg.StartPosition)
			targetFunc = func(pose spatialmath.Pose) float64 {
				return sqMet(&State{Position: spatialmath.Compose(seg.EndPosition, spatialmath.PoseInverse(pose))})
			}
		} else {
			
			relPose := spatialmath.Compose(spatialmath.PoseInverse(seg.EndPosition), seg.StartPosition)
			sqMet := NewSquaredNormMetric(relPose)
			targetFunc = func(pose spatialmath.Pose) float64 {
				return sqMet(&State{Position: pose})
			}
		}
		closeNode, err := ptg.CToTP(context.Background(), targetFunc)
		if err != nil || closeNode == nil {
			return math.Inf(1)
		}
		return targetFunc(closeNode.Pose)
	}
	opts.DistanceFunc = segMetric
	return opts
}

func (mp *tpSpaceRRTMotionPlanner) smoothPath(ctx context.Context, path []node) []node {
	toIter := int(math.Min(float64(len(path)*len(path)), float64(mp.planOpts.SmoothIter)))*5
	currCost := sumCosts(path)
	for i := 0; i < toIter; i++ {
		select {
		case <-ctx.Done():
			return path
		default:
		}
		// get start node of first edge. Cannot be either the last or second-to-last node.
		// Intn will return an int in the half-open interval half-open interval [0,n)
		firstEdge := mp.randseed.Intn(len(path) - 2)
		secondEdge := firstEdge + 2 + mp.randseed.Intn((len(path)-2)-firstEdge)
		mp.logger.Debugf("checking shortcut between nodes %d and %d", firstEdge, secondEdge)
		//~ fmt.Printf("checking shortcut between nodes %d and %d \n", firstEdge, secondEdge)
		
		newInputSteps, err := mp.attemptSmooth(ctx, path, firstEdge, secondEdge)
		if err != nil || newInputSteps == nil {
			continue
		}
		newCost := sumCosts(newInputSteps)
		if newCost >= currCost {
			continue
		}
		// Re-connect to the final goal
		newInputSteps = append(newInputSteps, path[len(path)-1])
		goalInputSteps, err := mp.attemptSmooth(ctx, newInputSteps, len(newInputSteps)-3, len(newInputSteps)-1)
		if err != nil || goalInputSteps == nil {
			continue
		}
		goalInputSteps = append(goalInputSteps, path[len(path)-1])
		
		
		path = goalInputSteps
		currCost = sumCosts(path)
	}
	return path
}

// firstEdge and secondEdge must not be adjacent
func (mp *tpSpaceRRTMotionPlanner) attemptSmooth(ctx context.Context, path []node, firstEdge, secondEdge int) ([]node, error) {
	startMap := map[node]node{}
	var parent node
	parentPose := spatialmath.NewZeroPose()
	
	for j := 0; j <= firstEdge; j++ {
		pathNode := path[j]
		startMap[pathNode] = parent
		for _, adj := range []float64{0.25, 0.5, 0.75} {
			fullQ := pathNode.Q()
			newQ := []referenceframe.Input{fullQ[0], fullQ[1], referenceframe.Input{fullQ[2].Value * adj}}
			trajK, err := mp.tpFrame.PTGs()[int(math.Round(newQ[0].Value))].Trajectory(newQ[1].Value, newQ[2].Value)
			if err != nil {
				continue
			}
			
			intNode := &basicNode{
				q:      newQ,
				cost:   pathNode.Cost() - (pathNode.Q()[2].Value * (1 - adj)),
				pose:   spatialmath.Compose(parentPose, trajK[len(trajK)-1].Pose),
				corner: false,
			}
			startMap[intNode] = parent
		}
		parent = pathNode
		parentPose = parent.Pose()
	}
	
	reached := mp.attemptExtension(ctx, nil, path[secondEdge], startMap, false)
	if reached.error != nil || reached.node == nil {
		return nil, errors.New("could not extend to smoothing destination")
	}
	
	reachedDelta := mp.planOpts.DistanceFunc(&Segment{StartPosition: reached.Pose(), EndPosition: path[secondEdge].Pose()})
	// If we tried the goal and have a close-enough XY location, check if the node is good enough to be a final goal
	if reachedDelta > mp.algOpts.poseSolveDist {
		return nil, errors.New("could not precisely reach smoothing destination")
	}
	//~ fmt.Println("matched poses", reached.Pose().Point(), path[secondEdge].Pose().Point())
	//~ fmt.Println("reached", reached.node)

	newInputSteps := extractPath(startMap, nil, &nodePair{a: reached.node, b: nil}, false)
	
	if secondEdge < len(path) - 1 {
		//~ fmt.Println("appending")
		newInputSteps = append(newInputSteps, path[secondEdge+1:]...)
	}
	return newInputSteps, nil
}
