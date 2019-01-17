package polygon

import "math"
import "fmt"
import "sort"

var (
	inf = float64(math.Pow(2, 64))
	eps = math.Pow(10, -3)
)

func IntersectStrict(e1, e2 Edge) (Vertex, bool) {
	e1 = e1.Order()
	e2 = e2.Order()
	if e1.IsEqual(e2) {
		return Vertex{0, 0, 0}, false
	}
	alpha1, b1, nVert1 := e1.Equation()
	alpha2, b2, nVert2 := e2.Equation()
	if nVert1 && nVert2 {
		xint := (b2 - b1) / (alpha1 - alpha2)
		if xint > e1.Begin().X+eps && xint < e1.End().X-eps && xint > e2.Begin().X+eps && xint < e2.End().X-eps {
			yint := alpha1*xint + b1
			return Vertex{0, xint, yint}, true
		}
	} else if !nVert1 && !nVert2 {
		return Vertex{0, 0, 0}, false
	} else if !nVert1 {
		m := e2.ValueIn(e1.Begin().X)
		if m > e1.Begin().Y && m < e1.End().Y && e1.Begin().X > e2.Begin().X && e1.Begin().X < e2.End().X {
			return Vertex{0, e1.Begin().X, m}, true
		}
	} else if !nVert2 {
		m := e1.ValueIn(e2.Begin().X)
		if m > e2.Begin().Y && m < e2.End().Y && e2.Begin().X > e1.Begin().X && e2.Begin().X < e1.End().X {
			return Vertex{0, e2.Begin().X, m}, true
		}
	}
	return Vertex{0, 0, 0}, false
}

func IntersectLarge(e1, e2 Edge) (Vertex, bool) {
	e1 = e1.Order()
	e2 = e2.Order()
	if e1.IsEqual(e2) {
		return Vertex{0, 0, 0}, false
	}
	alpha1, b1, nVert1 := e1.Equation()
	alpha2, b2, nVert2 := e2.Equation()
	xint := (b2 - b1) / (alpha1 - alpha2)
	if nVert1 && nVert2 {
		if xint > e1.Begin().X-eps && xint < e1.End().X+eps && xint > e2.Begin().X-eps && xint < e2.End().X+eps {
			yint := alpha1*xint + b1
			return Vertex{0, xint, yint}, true
		}
	} else if !nVert1 && !nVert2 {
		return Vertex{0, 0, 0}, false
	} else if !nVert1 {
		m := e2.ValueIn(e1.Begin().X)
		if m >= e1.Begin().Y && m <= e1.End().Y && e1.Begin().X >= e2.Begin().X && e1.Begin().X <= e2.End().X {
			return Vertex{0, e1.Begin().X, m}, true
		}
	} else if !nVert2 {
		m := e1.ValueIn(e2.Begin().X)
		if m >= e2.Begin().Y && m <= e2.End().Y && e2.Begin().X >= e1.Begin().X && e2.Begin().X <= e1.End().X {
			return Vertex{0, e2.Begin().X, m}, true
		}
	}
	return Vertex{0, 0, 0}, false
}

func ListOfEdges(spaceConf *Space) *[]Edge {
	l := []Edge{}
	for _, pol := range spaceConf.Polygons {
		begin := GetOneVertice(&pol)
		first := begin
		second := pol.Edges[first][0]
		keepgoing := true
		for keepgoing {
			tmp := second
			l = append(l, Edge{pol.Vertices[first], pol.Vertices[second]})
			second = FindGoodEdge(&pol, second, first)
			first = tmp
			if first == begin {
				keepgoing = false
			}
		}
	}
	return &l
}
func FasterListOfEdges(spaceConf *Space) *[]Edge {
	l := []Edge{}
	for _, pol := range spaceConf.Polygons {
		begin := GetOneVertice(&pol)
		first := begin
		second := pol.Edges[first][0]
		keepgoing := true
		for keepgoing {
			tmp := second
			if !(Norm(Vertex{0, pol.Vertices[first].X - pol.Vertices[second].X, 0}) < 1) {
				l = append(l, Edge{pol.Vertices[first], pol.Vertices[second]})
			}
			second = FindGoodEdge(&pol, second, first)
			first = tmp
			if first == begin {
				keepgoing = false
			}
		}
	}
	return &l
}

func EraseDoublonsOfSorted(listOfVert []Vertex, lastSeen Vertex) []Vertex {
	if len(listOfVert) == 0 {
		return []Vertex{}
	}
	diff := Norm(Vertex{0, listOfVert[0].X - lastSeen.X, listOfVert[0].Y - lastSeen.Y})
	if listOfVert[0].IsEqual(lastSeen) || diff < 5 {
		return EraseDoublonsOfSorted(listOfVert[1:], lastSeen)
	} else {
		return append([]Vertex{listOfVert[0]}, EraseDoublonsOfSorted(listOfVert[1:], listOfVert[0])...)
	}
	return []Vertex{}
}

func (e Edge) Split(listOfEdges *[]Edge) *[]Edge {
	e = e.Order()
	l := []Vertex{}
	for _, edge := range *listOfEdges {
		v, b := IntersectLarge(e, edge)
		if b {
			l = append(l, v)
		}
	}
	sort.Slice(l, func(i, j int) bool {
		di := math.Pow(e.Begin().X-l[i].X, 2) + math.Pow(e.Begin().Y-l[i].Y, 2)
		dj := math.Pow(e.Begin().X-l[j].X, 2) + math.Pow(e.Begin().Y-l[j].Y, 2)
		return di < dj
	})
	l = append([]Vertex{e.Begin()}, l...)
	l = append(l, e.End())
	l2 := EraseDoublonsOfSorted(l, Vertex{0, 0, 0})
	l3 := []Edge{}
	for i := 1; i < len(l2); i++ {
		l3 = append(l3, Edge{l2[i-1], l2[i]})
	}

	return &l3
}

func recEraseDouble(l []Edge, e Edge) []Edge {
	if len(l) == 0 {
		return l
	}
	if e.IsEqual(l[0]) {
		return recEraseDouble(l[1:], e)
	} else {
		return append(recEraseDouble(l[1:], l[0]), l[0])
	}
}

func EraseDouble(l []Edge) []Edge {
	if len(l) == 0 {
		return l
	}
	return append(recEraseDouble(l, l[0]), l[0])
}

func OrderListOfEdge(l []Edge) []Edge {
	if len(l) == 0 {
		return l
	}
	return append(OrderListOfEdge(l[1:]), l[0].Order())
}

func BorderEdges(spaceConf *Space) *[]Edge {
	lptr := ListOfEdges(spaceConf)
	l := []Edge{}
	for _, e1 := range *lptr {
		ltmp := e1.Split(lptr)
		for _, e := range *ltmp {
			v := Vertex{0, (e.Begin().X + e.End().X) / 2, (e.Begin().Y + e.End().Y) / 2}
			vdir := Vertex{0, e.Begin().X - e.End().X, e.Begin().Y - e.End().Y}
			coeff := math.Pow(10, -5) / Norm(vdir)
			vorth := Vertex{0, coeff * vdir.Y, coeff * (-vdir.X)}
			v1 := Vertex{0, v.X + vorth.X, v.Y + vorth.Y}
			v2 := Vertex{0, v.X - vorth.X, v.Y - vorth.Y}
			if !v1.IsInsideOneOf(&(spaceConf.Polygons)) || !v2.IsInsideOneOf(&(spaceConf.Polygons)) {
				l = append(l, e)
			}
		}
	}
	l = OrderListOfEdge(l)
	sort.Slice(l, func(i, j int) bool {
		if l[i].Begin().IsEqual(l[j].Begin()) {
			if l[i].End().X == l[j].End().X {
				return l[i].End().Y < l[j].End().Y
			} else {
				return l[i].End().X < l[j].End().X
			}
		} else {
			if l[i].Begin().X == l[j].Begin().X {
				return l[i].Begin().Y < l[j].Begin().Y
			} else {
				return l[i].Begin().X < l[j].Begin().X
			}
		}
	})
	l = EraseDouble(l)
	return &l
}

func FasterBorderEdges(spaceConf *Space) *[]Edge {
	lptr := FasterListOfEdges(spaceConf)
	l := []Edge{}
	for _, e1 := range *lptr {
		ltmp := e1.Split(lptr)
		for _, e := range *ltmp {
			v := Vertex{0, (e.Begin().X + e.End().X) / 2, (e.Begin().Y + e.End().Y) / 2}
			vdir := Vertex{0, e.Begin().X - e.End().X, e.Begin().Y - e.End().Y}
			coeff := math.Pow(10, -5) / Norm(vdir)
			vorth := Vertex{0, coeff * vdir.Y, coeff * (-vdir.X)}
			v1 := Vertex{0, v.X + vorth.X, v.Y + vorth.Y}
			v2 := Vertex{0, v.X - vorth.X, v.Y - vorth.Y}
			if !v1.IsInsideOneOf(&(spaceConf.Polygons)) || !v2.IsInsideOneOf(&(spaceConf.Polygons)) {
				l = append(l, e)
			}
		}
	}
	l = OrderListOfEdge(l)
	sort.Slice(l, func(i, j int) bool {
		if l[i].Begin().IsEqual(l[j].Begin()) {
			if l[i].End().X == l[j].End().X {
				return l[i].End().Y < l[j].End().Y
			} else {
				return l[i].End().X < l[j].End().X
			}
		} else {
			if l[i].Begin().X == l[j].Begin().X {
				return l[i].Begin().Y < l[j].Begin().Y
			} else {
				return l[i].Begin().X < l[j].Begin().X
			}
		}
	})
	l = EraseDouble(l)
	return &l
}

func FindFollowing(first, second Vertex, lEdges *[]Edge, lUsed *[]bool) Vertex {
	vres := Vertex{0, 0, 0}
	d := float64(6)
	icho := 0
	b := false
	for i, e := range *lEdges {
		if !(*lUsed)[i] {
			if e.Begin().IsEqualRange(second, 5) && !e.End().IsEqualRange(first, 3) {
				b = true
				n := Norm(Vertex{0, e.Begin().X - second.X, e.Begin().Y - second.Y})
				if n < d {
					d = n
					vres = e.End()
					icho = i
				}
			} else if e.End().IsEqualRange(second, 5) && !e.Begin().IsEqualRange(first, 3) {
				b = true
				n := Norm(Vertex{0, e.End().X - second.X, e.End().Y - second.Y})
				if n < d {
					d = n
					vres = e.Begin()
					icho = i
				}
			}
		}
	}
	if b {
		(*lUsed)[icho] = true
	}
	return vres
}

func IsAllTrue(lUsed *[]bool) bool {
	for _, b := range *lUsed {
		if !b {
			return false
		}
	}
	return true
}

func FirstFalse(lUsed *[]bool) int {
	for i, b := range *lUsed {
		if !b {
			return i
		}
	}
	return 0
}

func FromEdgesToPoly(lEdges *[]Edge) *[]Polygon {
	lres := []Polygon{}
	lUsed := make([]bool, len(*lEdges))
	id := uint32(0)
	for i, _ := range lUsed {
		lUsed[i] = false
	}
	for !IsAllTrue(&lUsed) {
		m := FirstFalse(&lUsed)
		lUsed[m] = true
		pol := Polygon{map[uint32]Vertex{}, map[uint32]([]uint32){}}
		begin, bsecond := (*lEdges)[m].Begin(), (*lEdges)[m].End()
		second := bsecond
		first := begin
		bid := id
		tid := id
		pol.Vertices[id] = first
		pol.Vertices[id+1] = second
		pol.Edges[id] = append(pol.Edges[id], id+1)
		pol.Edges[id+1] = append(pol.Edges[id+1], id)
		id = id + 2
		SecondTime := false
		keepgoing := true
		for keepgoing {
			tmp := second
			second = FindFollowing(first, second, lEdges, &lUsed)
			first = tmp
			if second.IsEqual(Vertex{0, 0, 0}) && !SecondTime {
				second = FindFollowing(bsecond, begin, lEdges, &lUsed)
				first = begin
				begin = tmp
				bid = id - 1
				SecondTime = true
				fmt.Printf("Hello")
			}
			if second.IsEqual(Vertex{0, 0, 0}) && SecondTime {
				keepgoing = false
				pol.Edges[bid] = append(pol.Edges[bid], tid)
				pol.Edges[tid] = append(pol.Edges[tid], bid)
			}
			if second.IsEqualRange(begin, 5) {
				keepgoing = false
				pol.Edges[bid] = append(pol.Edges[bid], id-1)
				pol.Edges[id-1] = append(pol.Edges[id-1], bid)
			} else if keepgoing {
				if SecondTime {
					pol.Vertices[id] = second
					pol.Edges[tid] = append(pol.Edges[tid], id)
					pol.Edges[id] = append(pol.Edges[id], tid)
					tid = id
				} else {
					pol.Vertices[id] = second
					pol.Edges[id-1] = append(pol.Edges[id-1], id)
					pol.Edges[id] = append(pol.Edges[id], id-1)
				}
				id++
			}
		}
		if !(len(pol.Vertices) < 3) {
			lres = append(lres, pol)
		}
	}
	return &lres
}
