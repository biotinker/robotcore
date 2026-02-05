package armplanning

import (
	"math"
	"math/rand"
	"runtime"
	"sort"
	"sync"
)

// vpNode is a node in a VP-tree. Each node stores a vantage point,
// a median distance threshold, and left/right children.
type vpNode[T any] struct {
	point  T
	median float64
	left   *vpNode[T] // points with dist <= median
	right  *vpNode[T] // points with dist > median
	isLeaf bool
	items  []T // only used for small leaf nodes
}

// VPTree is a Vantage Point Tree for efficient nearest-neighbor
// queries in arbitrary metric spaces. It only requires a distance
// function satisfying the triangle inequality.
type VPTree[T any] struct {
	root *vpNode[T]
	dist func(T, T) float64
	size int
}

const vpLeafSize = 16

// vpParallelThreshold is the minimum slice size to parallelize distance computation.
const vpParallelThreshold = 50000

// NewVPTree builds a VP-tree from the given points using the provided
// distance function. The distance function must satisfy the triangle
// inequality. Build time is O(n log n) on average.
func NewVPTree[T any](points []T, dist func(T, T) float64) *VPTree[T] {
	if len(points) == 0 {
		return &VPTree[T]{dist: dist}
	}

	// Work on a copy so we don't mutate the caller's slice.
	items := make([]T, len(points))
	copy(items, points)

	rng := rand.New(rand.NewSource(42))
	// Single scratch buffer for distances, reused across same-level operations.
	dists := make([]float64, len(items))

	tree := &VPTree[T]{
		dist: dist,
		size: len(items),
	}
	tree.root = tree.build(items, dists, rng)
	return tree
}

func (vp *VPTree[T]) build(items []T, dists []float64, rng *rand.Rand) *vpNode[T] {
	if len(items) == 0 {
		return nil
	}

	if len(items) <= vpLeafSize {
		return &vpNode[T]{
			isLeaf: true,
			items:  items,
		}
	}

	// Pick a random vantage point and swap to front.
	idx := rng.Intn(len(items))
	items[0], items[idx] = items[idx], items[0]

	node := &vpNode[T]{
		point: items[0],
	}

	rest := items[1:]
	ds := dists[1 : 1+len(rest)]

	// Compute distances from vantage point to all other points.
	vp.computeDistances(node.point, rest, ds)

	// Quickselect to find median and partition in O(n).
	mid := len(rest) / 2
	quickselect(rest, ds, 0, len(rest)-1, mid, rng)
	node.median = ds[mid]

	// At large sizes, build subtrees in parallel since they
	// operate on non-overlapping slices. Each subtree gets its
	// own distance scratch buffer since they can't share.
	if len(rest) >= vpParallelThreshold {
		leftDists := make([]float64, mid+1)
		rightDists := make([]float64, len(rest)-mid-1)

		var wg sync.WaitGroup
		wg.Add(2)
		// Use separate RNGs so there's no contention.
		rngL := rand.New(rand.NewSource(rng.Int63()))
		rngR := rand.New(rand.NewSource(rng.Int63()))
		go func() {
			defer wg.Done()
			node.left = vp.build(rest[:mid+1], leftDists, rngL)
		}()
		go func() {
			defer wg.Done()
			node.right = vp.build(rest[mid+1:], rightDists, rngR)
		}()
		wg.Wait()
	} else {
		node.left = vp.build(rest[:mid+1], ds[:mid+1], rng)
		node.right = vp.build(rest[mid+1:], ds[mid+1:], rng)
	}

	return node
}

// computeDistances fills ds[i] = dist(vantage, items[i]) in parallel
// when the slice is large enough, otherwise sequentially.
func (vp *VPTree[T]) computeDistances(vantage T, items []T, ds []float64) {
	if len(items) < vpParallelThreshold {
		for i := range items {
			ds[i] = vp.dist(vantage, items[i])
		}
		return
	}

	numWorkers := runtime.NumCPU()
	chunkSize := (len(items) + numWorkers - 1) / numWorkers

	var wg sync.WaitGroup
	for w := range numWorkers {
		lo := w * chunkSize
		if lo >= len(items) {
			break
		}
		hi := lo + chunkSize
		if hi > len(items) {
			hi = len(items)
		}
		wg.Add(1)
		go func() {
			defer wg.Done()
			for i := lo; i < hi; i++ {
				ds[i] = vp.dist(vantage, items[i])
			}
		}()
	}
	wg.Wait()
}

// quickselect partially sorts items and dists so that dists[k] is the
// k-th smallest distance, with all smaller distances before it and all
// larger distances after it. Items are rearranged to stay paired with dists.
func quickselect[T any](items []T, dists []float64, lo, hi, k int, rng *rand.Rand) {
	for lo < hi {
		pivotIdx := lo + rng.Intn(hi-lo+1)
		pivotIdx = partition(items, dists, lo, hi, pivotIdx)

		if k == pivotIdx {
			return
		} else if k < pivotIdx {
			hi = pivotIdx - 1
		} else {
			lo = pivotIdx + 1
		}
	}
}

func partition[T any](items []T, dists []float64, lo, hi, pivotIdx int) int {
	pivotDist := dists[pivotIdx]
	// Move pivot to end.
	items[pivotIdx], items[hi] = items[hi], items[pivotIdx]
	dists[pivotIdx], dists[hi] = dists[hi], dists[pivotIdx]

	store := lo
	for i := lo; i < hi; i++ {
		if dists[i] < pivotDist {
			items[i], items[store] = items[store], items[i]
			dists[i], dists[store] = dists[store], dists[i]
			store++
		}
	}
	// Move pivot to final position.
	items[store], items[hi] = items[hi], items[store]
	dists[store], dists[hi] = dists[hi], dists[store]
	return store
}

type vpCandidate[T any] struct {
	point T
	dist  float64
}

// vpHeap is a max-heap of candidates, ordered by distance.
// We use a max-heap so we can efficiently drop the farthest
// candidate when we find a closer one.
type vpHeap[T any] struct {
	items []vpCandidate[T]
	k     int
}

func newVPHeap[T any](k int) *vpHeap[T] {
	return &vpHeap[T]{
		items: make([]vpCandidate[T], 0, k+1),
		k:     k,
	}
}

func (h *vpHeap[T]) maxDist() float64 {
	if len(h.items) < h.k {
		return math.MaxFloat64
	}
	return h.items[0].dist
}

func (h *vpHeap[T]) add(point T, dist float64) {
	if len(h.items) < h.k {
		h.items = append(h.items, vpCandidate[T]{point, dist})
		h.siftUp(len(h.items) - 1)
		return
	}
	if dist >= h.items[0].dist {
		return
	}
	// Replace the max element.
	h.items[0] = vpCandidate[T]{point, dist}
	h.siftDown(0)
}

func (h *vpHeap[T]) siftUp(i int) {
	for i > 0 {
		parent := (i - 1) / 2
		if h.items[i].dist > h.items[parent].dist {
			h.items[i], h.items[parent] = h.items[parent], h.items[i]
			i = parent
		} else {
			break
		}
	}
}

func (h *vpHeap[T]) siftDown(i int) {
	n := len(h.items)
	for {
		largest := i
		left := 2*i + 1
		right := 2*i + 2
		if left < n && h.items[left].dist > h.items[largest].dist {
			largest = left
		}
		if right < n && h.items[right].dist > h.items[largest].dist {
			largest = right
		}
		if largest == i {
			break
		}
		h.items[i], h.items[largest] = h.items[largest], h.items[i]
		i = largest
	}
}

func (h *vpHeap[T]) results() []vpCandidate[T] {
	out := make([]vpCandidate[T], len(h.items))
	copy(out, h.items)
	sort.Slice(out, func(i, j int) bool {
		return out[i].dist < out[j].dist
	})
	return out
}

// NearestK returns the k nearest points to the query, sorted by
// distance (closest first). If k > tree size, returns all points.
func (vp *VPTree[T]) NearestK(query T, k int) []vpCandidate[T] {
	if vp.root == nil || k <= 0 {
		return nil
	}

	heap := newVPHeap[T](k)
	vp.search(vp.root, query, heap)
	return heap.results()
}

func (vp *VPTree[T]) search(node *vpNode[T], query T, heap *vpHeap[T]) {
	if node == nil {
		return
	}

	if node.isLeaf {
		for _, item := range node.items {
			d := vp.dist(query, item)
			heap.add(item, d)
		}
		return
	}

	d := vp.dist(query, node.point)
	heap.add(node.point, d)

	// Decide which subtree to search first (the one the query falls into).
	if d <= node.median {
		// Query is inside: search left first.
		vp.search(node.left, query, heap)
		// Only search right if the farthest candidate is beyond the median boundary.
		if d+heap.maxDist() > node.median {
			vp.search(node.right, query, heap)
		}
	} else {
		// Query is outside: search right first.
		vp.search(node.right, query, heap)
		// Only search left if the farthest candidate crosses into the inside.
		if d-heap.maxDist() <= node.median {
			vp.search(node.left, query, heap)
		}
	}
}

// Size returns the number of points in the tree.
func (vp *VPTree[T]) Size() int {
	return vp.size
}
