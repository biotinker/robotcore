package armplanning

import (
	"math"
	"math/rand"
	"sort"
)

// vpNode is a node in a VP-tree. Each node stores a vantage point,
// a median distance threshold, and left/right children.
type vpNode[T any] struct {
	point    T
	median   float64
	left     *vpNode[T] // points with dist <= median
	right    *vpNode[T] // points with dist > median
	isLeaf   bool
	leafData []T // only used for small leaf nodes
}

// VPTree is a Vantage Point Tree for efficient nearest-neighbor
// queries in arbitrary metric spaces. It only requires a distance
// function satisfying the triangle inequality.
type VPTree[T any] struct {
	root *vpNode[T]
	dist func(T, T) float64
	size int
}

const vpLeafSize = 8

// NewVPTree builds a VP-tree from the given points using the provided
// distance function. The distance function must satisfy the triangle
// inequality. Build time is O(n log n) on average.
func NewVPTree[T any](points []T, dist func(T, T) float64) *VPTree[T] {
	if len(points) == 0 {
		return &VPTree[T]{dist: dist}
	}

	// Make a copy so we don't mutate the caller's slice.
	items := make([]T, len(points))
	copy(items, points)

	rng := rand.New(rand.NewSource(42))

	tree := &VPTree[T]{
		dist: dist,
		size: len(items),
	}
	tree.root = tree.build(items, rng)
	return tree
}

func (vp *VPTree[T]) build(items []T, rng *rand.Rand) *vpNode[T] {
	if len(items) == 0 {
		return nil
	}

	if len(items) <= vpLeafSize {
		return &vpNode[T]{
			isLeaf:   true,
			leafData: items,
		}
	}

	// Pick a random vantage point by swapping it to the front.
	idx := rng.Intn(len(items))
	items[0], items[idx] = items[idx], items[0]

	node := &vpNode[T]{
		point: items[0],
	}

	rest := items[1:]
	if len(rest) == 0 {
		node.isLeaf = true
		node.leafData = items[:1]
		return node
	}

	// Compute distances from vantage point to all other points.
	type distItem struct {
		item T
		dist float64
	}
	dists := make([]distItem, len(rest))
	for i, item := range rest {
		dists[i] = distItem{item, vp.dist(node.point, item)}
	}

	// Sort by distance to find the median.
	sort.Slice(dists, func(i, j int) bool {
		return dists[i].dist < dists[j].dist
	})

	medianIdx := len(dists) / 2
	node.median = dists[medianIdx].dist

	// Split into left (<=median) and right (>median).
	leftItems := make([]T, 0, medianIdx+1)
	rightItems := make([]T, 0, len(dists)-medianIdx-1)

	for _, d := range dists {
		if d.dist <= node.median {
			leftItems = append(leftItems, d.item)
		} else {
			rightItems = append(rightItems, d.item)
		}
	}

	node.left = vp.build(leftItems, rng)
	node.right = vp.build(rightItems, rng)

	return node
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
		// Bubble up.
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
		for _, item := range node.leafData {
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
