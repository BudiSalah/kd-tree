package kdtree

import (
	"math"
	"sort"
)

// Represents a K-Dimensional data point.
type KDPoint[T any] interface {
	GetDimensionValue(n int) T //Returns the value of the nth dimension
	Dimensions() int           //Returns the number of dimensions
}

type KDistanceCalculator[T any] func(a, b KDPoint[T], dim int) float64

// a node in our tree
type Node[T any] struct {
	Point KDPoint[T] //Holds the data point.

	//Pointers to child nodes.
	Left  *Node[T]
	Right *Node[T]
}

type KDTree[T any] struct {
	Root  *Node[T]
	Size  int
	dstFn KDistanceCalculator[T] //Function to calculate the distance between two points at a given dimension.
}

func NewKDTree[T any](points []KDPoint[T], dstFn KDistanceCalculator[T]) *KDTree[T] {
	if dstFn == nil {
		panic("dstFn cannot be nil")
	}

	return &KDTree[T]{dstFn: dstFn, Root: buildTree(points, dstFn), Size: len(points)}
}

func buildTree[T any](points []KDPoint[T], dstFn KDistanceCalculator[T]) *Node[T] {
	sort.Slice(points, func(i, j int) bool {
		return dstFn(points[i], points[j], 0) < 0
	})

	return treeMaker(points, dstFn, 0)
}

func (t *KDTree[T]) SearchNearest(target KDPoint[T]) KDPoint[T] {
	return treeNearest(target, t.Root, t.dstFn, 0, nil, nil)
}

func (t *KDTree[T]) Insert(p KDPoint[T]) {
	treeInsert(p, t.Root, t.dstFn, 0)
}

func (t *KDTree[T]) ForEach(fn func(*Node[T], int)) {
	traverse(t.Root, 0, fn)
}

// ------------------------------
type Point[T any] struct {
	values [2]T
}

func (p Point[T]) GetDimensionValue(n int) (returnValue T) {
	return p.values[n]
}

func (p Point[T]) Dimensions() int {
	return len(p.values)
}

func treeMaker[T any](points []KDPoint[T], dstFn KDistanceCalculator[T], depth int) *Node[T] {
	if len(points) == 0 {
		return nil
	}

	if len(points) == 1 {
		return &Node[T]{
			Point: points[0],
			Left:  nil,
			Right: nil,
		}
	}

	dim := depth % points[0].Dimensions()

	n := len(points)
	mid := int(math.Floor(float64(n) / 2))

	var leftPoints []KDPoint[T]
	var rightPoints []KDPoint[T]

	for i := 0; i < len(points); i++ {
		if i == mid {
			continue
		}

		if dstFn(points[mid], points[i], dim) > 0 {
			leftPoints = append(leftPoints, points[i])
		} else {
			rightPoints = append(rightPoints, points[i])
		}
	}

	node := &Node[T]{}

	node.Point = points[mid]

	node.Left = treeMaker[T](leftPoints, dstFn, depth+1)

	node.Right = treeMaker[T](rightPoints, dstFn, depth+1)

	return node
}

func treeNearest[T any](target KDPoint[T], node *Node[T], dstFn KDistanceCalculator[T], depth int, nearestNode *Node[T], bestDistance *float64) KDPoint[T] {
	noLeftAndRight := node.Left == nil && node.Right == nil

	if noLeftAndRight {
		distance := distance[T](target, node.Point, dstFn)

		if bestDistance != nil && *bestDistance < distance && noLeftAndRight {
			return nearestNode.Point
		}

		if bestDistance == nil || distance < *bestDistance {
			nearestNode = node
			bestDistance = &distance
		}

		return nearestNode.Point
	}

	distance := distance[T](target, node.Point, dstFn)

	if bestDistance != nil && *bestDistance < distance && noLeftAndRight {
		return nearestNode.Point
	}

	if bestDistance == nil || distance < *bestDistance {
		nearestNode = node
		bestDistance = &distance
	}

	dim := depth % node.Point.Dimensions()

	if dstFn(target, node.Point, dim) > 0 {
		// go right
		if node.Right == nil {
			return treeNearest(target, node.Left, dstFn, depth+1, nearestNode, bestDistance)
		}

		return treeNearest(target, node.Right, dstFn, depth+1, nearestNode, bestDistance)
	} else {
		// go left
		if node.Left == nil {
			return treeNearest(target, node.Right, dstFn, depth+1, nearestNode, bestDistance)
		}

		return treeNearest(target, node.Left, dstFn, depth+1, nearestNode, bestDistance)
	}
}

func treeInsert[T any](p KDPoint[T], node *Node[T], dstFn KDistanceCalculator[T], depth int) {
	dim := depth % node.Point.Dimensions()

	if dstFn(p, node.Point, dim) > 0 {
		// go right
		if node.Right == nil {
			node.Right = &Node[T]{Point: p}
			return
		}

		treeInsert(p, node.Right, dstFn, depth+1)
	} else {
		// go left
		if node.Left == nil {
			node.Left = &Node[T]{Point: p}
			return
		}

		treeInsert(p, node.Left, dstFn, depth+1)
	}
}
