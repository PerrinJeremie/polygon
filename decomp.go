package polygon

import "sort"
import "mygo-avl"
import "mathint"
import "fmt"

const (
	NONEADD   = 1
	UPDOWNADD = 2
	UPDOWN    = 3
	NONEDEL   = 4
	UP        = 5
	DOWN      = 6
)

type Edges interface {
	Begin() Vertex
	End() Vertex
	ValueIn(x int32) int32
	Order()
	Compare(v avl.AvlTreeValue) uint8
}

type Edge struct {
	V1 Vertex
	V2 Vertex
}

func (e Edge) IntersectsOne(l *[]Edge) bool {
	for _, v := range *l {
		_, inter := IntersectStrict(e, v)
		if inter {
			return true
		}
	}
	return false
}

func (e Edge) Begin() Vertex {
	return e.V1
}

func (e Edge) End() Vertex {
	return e.V2
}

func (e Edge) Equation() (float64, float64, bool) {
	e = e.Order()
	if e.Begin().X != e.End().X {
		alpha := (e.End().Y - e.Begin().Y) / (e.End().X - e.Begin().X)
		b := e.Begin().Y - alpha*e.Begin().X
		return alpha, b, true
	}
	return 0, 0, false

}

func (e Edge) ValueIn(x float64) float64 {
	e = e.Order()
	if e.Begin().X == e.End().X {
		return (e.Begin().Y + e.End().Y) / 2
	} else {
		alpha := (e.End().Y - e.Begin().Y) / (e.End().X - e.Begin().X)
		return e.Begin().Y + alpha*(x-e.Begin().X)
	}
}

func (e Edge) Order() Edge {
	if e.V1.X > e.V2.X {
		return Edge{e.V2, e.V1}
	} else if e.V1.X == e.V2.X && e.V1.Y > e.V2.Y {
		return Edge{e.V2, e.V1}
	}
	return e
}

func (e Edge) IsEqual(ep Edge) bool {
	e = e.Order()
	ep = ep.Order()
	b := e.Begin().IsEqual(ep.Begin())
	return b && e.End().IsEqual(ep.End())
}

func (a Edge) Compare(v avl.AvlTreeValue) uint8 {
	b := v.(Edge)
	a = a.Order()
	b = b.Order()
	aBX, aBY, aEX, aEY := a.Begin().X, a.Begin().Y, a.End().X, a.End().Y
	bBX, bBY, bEX, bEY := b.Begin().X, b.Begin().Y, b.End().X, b.End().Y
	if aBX == bBX && aBY == bBY && aEX == bEX && aEY == bEY {
		return avl.EQ
	}
	if aBX >= bBX && aBX <= bEX {
		val := b.ValueIn(aBX)
		if aBY > val {
			return avl.LT
		} else if aBY < val {
			return avl.GT
		} else {
			if b.ValueIn(aEX) > aEY {
				return avl.GT
			} else {
				return avl.LT
			}
		}
	}
	if aEX >= bBX && aEX <= bEX {
		val := b.ValueIn(aEX)
		if aEY > val {
			return avl.LT
		} else if aEY < val {
			return avl.GT
		} else {
			if b.ValueIn(aBX) > aBY {
				return avl.GT
			} else {
				return avl.LT
			}
		}
	}
	if bBX >= aBX && bBX <= aEX {
		switch b.Compare(a) {
		case avl.LT:
			return avl.GT
			break
		case avl.GT:
			return avl.LT
			break
		default:
			return avl.EQ
		}
	}
	return avl.EQ
}

type GLink struct {
	Id uint32
	V  Vertex
}

type GNode struct {
	Pol *Polygon
	Ids []uint32
}

type Graph struct {
	Cells   map[uint32](GNode)
	AdjList map[uint32]([]GLink)
	Id      uint32
}

func (g Graph) FindNodeOfVertRec(vert Vertex, id uint32, lSeen *(map[uint32]bool)) (uint32, bool) {
	(*lSeen)[id] = true
	if vert.IsInside(g.Cells[id].Pol) {
		return id, true
	} else {
		for _, link := range g.AdjList[id] {
			if !(*lSeen)[link.Id] {
				resId, b := g.FindNodeOfVertRec(vert, link.Id, lSeen)
				if b {
					return resId, b
				}
			}
		}
	}
	return 0, false
}

func (g Graph) FindNodeOfVert(vert Vertex) (uint32, bool) {
	lSeen := map[uint32]bool{}
	for k, _ := range g.Cells {
		lSeen[k] = false
	}
	for k, _ := range g.Cells {
		if !lSeen[k] {
			id, found := g.FindNodeOfVertRec(vert, k, &lSeen)
			if found {
				return id, found
			}
		}
	}
	return 0, false
}

func (g Graph) FindPathRec(curId, idToFind uint32, lSeen *(map[uint32]bool)) ([]Vertex, bool) {
	(*lSeen)[curId] = true
	for _, link := range g.AdjList[curId] {
		if !(*lSeen)[link.Id] {
			if link.Id == idToFind {
				(*lSeen)[idToFind] = true
				return []Vertex{link.V}, true
			} else {
				l, b := g.FindPathRec(link.Id, idToFind, lSeen)
				if b {
					return append([]Vertex{link.V}, l...), b
				}
			}
		}
	}
	return []Vertex{}, false

}

func (g Graph) FindPath(deb, fin Vertex) ([]Vertex, bool) {
	lSeen := map[uint32]bool{}
	ideb, foundDeb := g.FindNodeOfVert(deb)
	ifin, foundFin := g.FindNodeOfVert(fin)
	if !foundDeb || !foundFin {
		return []Vertex{}, false
	}
	for k, _ := range g.Cells {
		lSeen[k] = false
	}
	lres, b := g.FindPathRec(ideb, ifin, &lSeen)
	if b {
		return append([]Vertex{deb}, append(lres, fin)...), true
	}
	return []Vertex{}, false

}

func (g Graph) AddNode(pol *Polygon, ids []uint32) *Graph {
	node := GNode{pol, ids}
	leftUp, foundUp := g.FindNode(ids[0])
	leftDown, foundDown := g.FindNode(ids[3])
	if !foundUp && !foundDown {
		g.Cells[g.Id] = node
		g.AdjList[g.Id] = []GLink{}
		g.Id = g.Id + 1
		return &g
	}
	if leftUp == leftDown && foundDown && foundUp {
		fmt.Printf("\nFOUND SAME: %v\n", g.Id)
		nodeD := g.Cells[leftDown]
		DVert := nodeD.Pol.Vertices
		v1 := DVert[nodeD.Ids[1]]
		v2 := DVert[nodeD.Ids[2]]
		l1 := GLink{g.Id, Vertex{0, (v1.X + v2.X) / 2, (v1.Y + v2.Y) / 2}}
		l2 := GLink{leftDown, Vertex{0, (v1.X + v2.X) / 2, (v1.Y + v2.Y) / 2}}
		g.AdjList[leftDown] = append(g.AdjList[leftDown], l1)
		g.AdjList[g.Id] = []GLink{l2}
		g.Cells[g.Id] = node
		g.Id = g.Id + 1
		return &g
	} else if foundDown && foundUp {
		nodeD := g.Cells[leftDown]
		DVert := nodeD.Pol.Vertices
		v1 := DVert[nodeD.Ids[1]]
		v2 := DVert[nodeD.Ids[2]]
		l1 := GLink{g.Id, Vertex{0, (v1.X + v2.X) / 2, (v1.Y + v2.Y) / 2}}
		l2 := GLink{leftDown, Vertex{0, (v1.X + v2.X) / 2, (v1.Y + v2.Y) / 2}}
		g.AdjList[leftDown] = append(g.AdjList[leftDown], l1)
		g.AdjList[g.Id] = []GLink{l2}
		nodeU := g.Cells[leftUp]
		UVert := nodeU.Pol.Vertices
		v1 = UVert[nodeU.Ids[1]]
		v2 = UVert[nodeU.Ids[2]]
		l1 = GLink{g.Id, Vertex{0, (v1.X + v2.X) / 2, (v1.Y + v2.Y) / 2}}
		l2 = GLink{leftUp, Vertex{0, (v1.X + v2.X) / 2, (v1.Y + v2.Y) / 2}}
		g.AdjList[leftUp] = append(g.AdjList[leftUp], l1)
		g.AdjList[g.Id] = append(g.AdjList[g.Id], l2)
	} else if foundUp {
		v1 := pol.Vertices[ids[0]]
		v2 := pol.Vertices[ids[3]]
		l1 := GLink{g.Id, Vertex{0, (v1.X + v2.X) / 2, (v1.Y + v2.Y) / 2}}
		l2 := GLink{leftUp, Vertex{0, (v1.X + v2.X) / 2, (v1.Y + v2.Y) / 2}}
		g.AdjList[leftUp] = append(g.AdjList[leftUp], l1)
		if g.AdjList[g.Id] == nil {
			g.AdjList[g.Id] = []GLink{l2}
		} else {
			g.AdjList[g.Id] = append(g.AdjList[g.Id], l2)
		}
	} else if foundDown {
		v1 := pol.Vertices[ids[0]]
		v2 := pol.Vertices[ids[3]]
		l1 := GLink{g.Id, Vertex{0, (v1.X + v2.X) / 2, (v1.Y + v2.Y) / 2}}
		l2 := GLink{leftDown, Vertex{0, (v1.X + v2.X) / 2, (v1.Y + v2.Y) / 2}}
		g.AdjList[leftDown] = append(g.AdjList[leftDown], l1)
		if g.AdjList[g.Id] == nil {
			g.AdjList[g.Id] = []GLink{l2}
		} else {
			g.AdjList[g.Id] = append(g.AdjList[g.Id], l2)
		}
	}
	g.Cells[g.Id] = node
	g.Id = g.Id + 1
	return &g
}

func (g Graph) FindNode(id uint32) (uint32, bool) {
	for i, v := range g.Cells {
		if v.Ids[1] == id || v.Ids[2] == id {
			return i, true
		}
	}
	return 0, false
}

func (g Graph) AreEqual(id1, id2 uint32) bool {
	Ids1 := (g.Cells[id1]).Ids
	Ids2 := (g.Cells[id2]).Ids
	if len(Ids1) != len(Ids2) {
		return false
	}
	for i := 0; i < len(Ids1); i++ {
		if Ids1[i] != Ids2[i] {
			return false
		}
	}
	return true
}

func ListOf(pol *Polygon) []Vertex {
	l := []Vertex{}
	for _, v := range pol.Vertices {
		l = append(l, v)
	}
	return l
}

func SortPolygon(space *Space) ([]Vertex, []uint32) {
	l := []Vertex{}
	mins := []uint32{}
	for _, v := range space.Polygons {
		tmp := []Vertex{}
		for _, vl := range v.Vertices {
			tmp = append(tmp, vl)
		}
		sort.Slice(tmp, func(i, j int) bool {
			return tmp[i].X < tmp[j].X
		})
		l = append(l, tmp...)
		mins = append(mins, tmp[0].Id)
	}
	sort.Slice(l, func(i, j int) bool {
		return l[i].X < l[j].X
	})
	return l, mins
}

func AllZero(n int) []int {
	l := []int{}
	for i := 0; i < n; i++ {
		l = append(l, 0)
	}
	return l
}

func IsAbove(e Edge, x, y float64) bool {
	e = e.Order()
	v := e.ValueIn(x)
	vertex := Vertex{0, x, y}
	if x >= e.Begin().X && x <= e.End().X && v <= y && !vertex.IsEqual(e.Begin()) && !vertex.IsEqual(e.End()) {
		return true
	}
	return false
}

func IsInside(space *Space, id uint32) bool {
	s := 0
	vid := space.PolygonOfId[id].Vertices[id]
	for _, pol := range space.Polygons {
		vert := pol.Vertices
		beginning := GetOneVertice(&pol)
		_, ok := pol.Vertices[id]
		if !ok {
			first := beginning
			second := pol.Edges[first][0]
			run := true
			for run {
				e := Edge{vert[first], vert[second]}
				if IsAbove(e, vid.X, vid.Y) {
					fmt.Printf("\n%v : %v,%v \n", e, vid.X, vid.Y)
					s += 1
				}
				tmp := second
				second = FindGoodEdge(&pol, second, first)
				first = tmp
				run = !(first == beginning)
			}
		}
	}
	fmt.Printf("\n%v%v\n%v\n", vid.X, vid.Y, mathint.Mod(uint32(s), 2))
	return (mathint.Mod(uint32(s), 2) == 1)
}

func Above(node *avl.TreeNode, x, y float64) Edge {
	e := Edge{Vertex{0, 0, 0}, Vertex{0, 0, 0}}
	if node == nil {
		return e
	}
	k := node.Key().(Edge)
	op := k.ValueIn(x)
	if op > y {
		return Above(node.Right(), x, y)
	} else if node.Left() != nil {
		s := Above(node.Left(), x, y)
		if s == e {
			return k
		} else {
			return s
		}
	}
	return k
}

func Below(node *avl.TreeNode, x, y float64) Edge {
	e := Edge{Vertex{0, 0, 0}, Vertex{0, 0, 0}}
	if node == nil {
		return e
	}
	k := node.Key().(Edge)
	op := k.ValueIn(x)
	if op < y {
		return Below(node.Left(), x, y)
	} else if node.Right() != nil {
		s := Below(node.Right(), x, y)
		if s == e {
			return k
		} else {
			return s
		}
	}
	return k
}

func EdgesOfVert(node *avl.TreeNode, v Vertex) []Edge {
	if node == nil {
		return []Edge{}
	}
	l := []Edge{}
	k := node.Key().(Edge)
	k.Order()
	l = append(l, EdgesOfVert(node.Right(), v)...)
	if k.V2.IsEqual(v) {
		l = append(l, k)
	}
	l = append(l, EdgesOfVert(node.Left(), v)...)
	return l
}

func AdjustIn(c int, isI bool) int {
	if isI {
		switch c {
		case UP:
			return DOWN
			break
		case DOWN:
			return UP
			break
		case NONEADD:
			return UPDOWNADD
			break
		case UPDOWNADD:
			return NONEADD
			break
		case UPDOWN:
			return NONEDEL
			break
		case NONEDEL:
			return UPDOWN
			break
		}
	}
	return c
}

func NodeType(pol *Polygon, first, second, third uint32) int {
	v1 := pol.Vertices[first]
	v2 := pol.Vertices[second]
	v3 := pol.Vertices[third]
	fmt.Printf("\n%v\n", v2)
	e1 := Edge{v1, v2}
	e2 := Edge{v2, v3}
	if v1.X < v2.X && v2.X < v3.X {
		fmt.Printf("UP")
		return UP
	}
	if v1.X > v2.X && v2.X > v3.X {
		fmt.Printf("DOWN")
		return DOWN
	}
	op := e1.Compare(e2)
	if v1.X > v2.X {
		if op == avl.GT {
			fmt.Printf("NONEADD")
			return NONEADD
		}
		if op == avl.LT {
			fmt.Printf("UPDOWNADD")
			return UPDOWNADD
		}
	} else {
		if op == avl.GT {
			fmt.Printf("UPDOWN")
			return UPDOWN
		}
		if op == avl.LT {
			fmt.Printf("NONEDEL")
			return NONEDEL
		}
	}
	return NONEADD
}

func TypesOfNodes(space *Space, mins *[]uint32) map[uint32]int {
	m := map[uint32]int{}
	for i, pol := range space.Polygons {
		min := (*mins)[i]
		isI := IsInside(space, min)
		if !isI {
			m[min] = NONEADD
		} else {
			m[min] = UPDOWNADD
		}
		first := min
		second := uint32(0)
		vert := pol.Vertices
		e1 := Edge{vert[min], vert[pol.Edges[min][0]]}
		e2 := Edge{vert[min], vert[pol.Edges[min][1]]}
		if e1.Compare(e2) == avl.LT {
			second = pol.Edges[min][0]
		} else {
			second = pol.Edges[min][1]
		}
		for second != min {
			third := FindGoodEdge(&pol, second, first)
			m[second] = AdjustIn(NodeType(&pol, first, second, third),
				isI)
			first = second
			second = third
		}
	}
	return m
}

func EqVert(v1, v2 Vertex) bool {
	return v1.X == v2.X && v1.Y == v2.Y
}

func OrderTree(node *avl.TreeNode) {
	if node != nil {
		k := node.Key().(Edge)
		k.Order()
		node.ChangeKeyInto(k)
		OrderTree(node.Right())
		OrderTree(node.Left())
	}
}

func noRedundancy(ab, be Edge) *[]Vertex {
	ab = ab.Order()
	be = be.Order()
	tmpl := []Vertex{be.V1, be.V2}
	if !be.V2.IsEqual(ab.V2) {
		tmpl = append(tmpl, ab.V2)
	}
	if !be.V1.IsEqual(ab.V1) {
		tmpl = append(tmpl, ab.V1)
	}
	return &tmpl
}

func Decompose(space *Space) []*Polygon {
	root := avl.NewAvlTree()
	l, mins := SortPolygon(space)
	marks := TypesOfNodes(space, &mins)
	res := []*Polygon{}
	vert, edg := UnionOf(&space.Polygons)
	for _, v := range l {
		switch marks[v.Id] {
		case NONEADD:
			root = avl.InsertVal(
				Edge{vert[v.Id], vert[edg[v.Id][0]]}, root)
			root = avl.InsertVal(
				Edge{vert[v.Id], vert[edg[v.Id][1]]}, root)
			break
		case UPDOWNADD:
			ab := Above(root, v.X, v.Y)
			be := Below(root, v.X, v.Y)
			ab = ab.Order()
			be = be.Order()
			mab := Edge{ab.V1, Vertex{0, v.X, ab.ValueIn(v.X)}}
			mbe := Edge{be.V1, Vertex{0, v.X, be.ValueIn(v.X)}}

			res = append(res, PolygonOfList(noRedundancy(mab, mbe)))
			abm := Edge{Vertex{0, v.X, ab.ValueIn(v.X)}, ab.V2}
			bem := Edge{Vertex{0, v.X, be.ValueIn(v.X)}, be.V2}
			root = root.ModVal(ab, abm)
			root = root.ModVal(be, bem)
			root = avl.InsertVal(
				Edge{v, vert[edg[v.Id][0]]}, root)
			root = avl.InsertVal(
				Edge{v, vert[edg[v.Id][1]]}, root)
			break
		case UPDOWN:
			toDelete := EdgesOfVert(root, v)
			root, _ = avl.DeleteVal(toDelete[0], root)
			root, _ = avl.DeleteVal(toDelete[1], root)
			ab := Above(root, v.X, v.Y)
			be := Below(root, v.X, v.Y)
			mab := Edge{ab.V1, Vertex{0, v.X, ab.ValueIn(v.X)}}
			mbe := Edge{be.V1, Vertex{0, v.X, be.ValueIn(v.X)}}
			res = append(res,
				PolygonOfList(noRedundancy(mab, toDelete[0])))
			res = append(res,
				PolygonOfList(noRedundancy(toDelete[1], mbe)))
			abm := Edge{Vertex{0, v.X, ab.ValueIn(v.X)}, ab.V2}
			bem := Edge{Vertex{0, v.X, be.ValueIn(v.X)}, be.V2}
			root = root.ModVal(ab, abm)
			root = root.ModVal(be, bem)
			break
		case NONEDEL:
			toDelete := EdgesOfVert(root, v)
			root, _ = avl.DeleteVal(toDelete[0], root)
			root, _ = avl.DeleteVal(toDelete[1], root)
			res = append(res, PolygonOfList(
				noRedundancy(toDelete[0], toDelete[1])))
			break
		case UP:
			toDelete := EdgesOfVert(root, v)
			root, _ = avl.DeleteVal(toDelete[0], root)
			ab := Above(root, v.X, v.Y)
			abmp := Edge{ab.V1, Vertex{0, v.X, ab.ValueIn(v.X)}}
			res = append(res,
				PolygonOfList(noRedundancy(toDelete[0], abmp)))
			abm := Edge{Vertex{0, v.X, ab.ValueIn(v.X)}, ab.V2}
			root = root.ModVal(ab, abm)
			tmpv := vert[edg[v.Id][0]]
			if tmpv.X < v.X {
				root = avl.InsertVal(
					Edge{vert[v.Id],
						vert[edg[v.Id][1]]}, root)
			} else {
				root = avl.InsertVal(
					Edge{vert[v.Id], tmpv}, root)
			}
			break
		case DOWN:
			toDelete := EdgesOfVert(root, v)
			root, _ = avl.DeleteVal(toDelete[0], root)
			be := Below(root, v.X, v.Y)
			bemp := Edge{be.V1, Vertex{0, v.X, be.ValueIn(v.X)}}
			res = append(res,
				PolygonOfList(noRedundancy(toDelete[0], bemp)))
			bem := Edge{Vertex{0, v.X, be.ValueIn(v.X)}, be.V2}
			root = root.ModVal(be, bem)
			tmpv := vert[edg[v.Id][0]]
			if tmpv.X < v.X {
				root = avl.InsertVal(
					Edge{vert[v.Id],
						vert[edg[v.Id][1]]}, root)
			} else {
				root = avl.InsertVal(
					Edge{vert[v.Id], tmpv}, root)
			}
			break
		}
	}
	return res
}

func AllPositive(pol Polygon) bool {
	for _, v := range pol.Vertices {
		if v.X < -5 || v.Y < -5 {
			return false
		}
	}
	return true
}

func DeleteFirstPol(space *Space) {
	l, _ := SortPolygon(space)
	vmin := l[0]
	change := []Polygon{}
	for _, pol := range space.Polygons {
		_, ok := pol.Vertices[vmin.Id]
		if !ok && AllPositive(pol) {
			change = append(change, pol)
		}
	}
	space.Polygons = change
}

func NewGraph() Graph {
	return Graph{map[uint32]GNode{}, map[uint32]([]GLink){}, 0}
}

func DecomposeAndBuildGraph(space *Space) ([]*Polygon, Graph) {
	root := avl.NewAvlTree()
	l, mins := SortPolygon(space)
	marks := TypesOfNodes(space, &mins)
	res := []*Polygon{}
	grap := NewGraph()
	graph := &grap
	vert, edg := UnionOf(&space.Polygons)
	id := space.Id + 1
	for _, v := range l {
		switch marks[v.Id] {
		case NONEADD:
			root = avl.InsertVal(
				Edge{vert[v.Id], vert[edg[v.Id][0]]}, root)
			root = avl.InsertVal(
				Edge{vert[v.Id], vert[edg[v.Id][1]]}, root)
			break
		case UPDOWNADD:
			ab := Above(root, v.X, v.Y)
			be := Below(root, v.X, v.Y)
			ab = ab.Order()
			be = be.Order()
			mab := Edge{ab.V1, Vertex{id, v.X, ab.ValueIn(v.X)}}
			mbe := Edge{be.V1, Vertex{id + 1, v.X, be.ValueIn(v.X)}}
			nr := noRedundancy(mab, mbe)
			graph = graph.AddNode(PolygonOfListIds(nr), []uint32{mab.V1.Id, mab.V2.Id, mbe.V2.Id, mbe.V1.Id})
			res = append(res, PolygonOfList(nr))
			abm := Edge{Vertex{id, v.X, ab.ValueIn(v.X)}, ab.V2}
			bem := Edge{Vertex{id + 1, v.X, be.ValueIn(v.X)}, be.V2}
			root = root.ModVal(ab, abm)
			root = root.ModVal(be, bem)
			root = avl.InsertVal(
				Edge{v, vert[edg[v.Id][0]]}, root)
			root = avl.InsertVal(
				Edge{v, vert[edg[v.Id][1]]}, root)
			id = id + 2
			break
		case UPDOWN:
			toDelete := EdgesOfVert(root, v)
			root, _ = avl.DeleteVal(toDelete[0], root)
			root, _ = avl.DeleteVal(toDelete[1], root)
			ab := Above(root, v.X, v.Y)
			be := Below(root, v.X, v.Y)
			mab := Edge{ab.V1, Vertex{id, v.X, ab.ValueIn(v.X)}}
			mbe := Edge{be.V1, Vertex{id + 1, v.X, be.ValueIn(v.X)}}
			td0 := toDelete[0]
			td1 := toDelete[1]
			nr := noRedundancy(mab, td0)
			graph = graph.AddNode(PolygonOfListIds(nr), []uint32{mab.V1.Id, mab.V2.Id, td0.V2.Id, td0.V1.Id})
			res = append(res, PolygonOfList(nr))
			nr = noRedundancy(td1, mbe)
			graph = graph.AddNode(PolygonOfListIds(nr), []uint32{td1.V1.Id, td1.V2.Id, mbe.V2.Id, mbe.V1.Id})
			res = append(res, PolygonOfList(nr))
			abm := Edge{Vertex{id, v.X, ab.ValueIn(v.X)}, ab.V2}
			bem := Edge{Vertex{id + 1, v.X, be.ValueIn(v.X)}, be.V2}
			root = root.ModVal(ab, abm)
			root = root.ModVal(be, bem)
			id = id + 2
			break
		case NONEDEL:
			toDelete := EdgesOfVert(root, v)
			root, _ = avl.DeleteVal(toDelete[0], root)
			root, _ = avl.DeleteVal(toDelete[1], root)
			td0 := toDelete[0]
			td1 := toDelete[1]
			if td1.Compare(td0) == avl.GT {
				tmp := td0
				td0 = td1
				td1 = tmp
			}
			nr := noRedundancy(td0, td1)
			graph = graph.AddNode(PolygonOfListIds(nr), []uint32{td0.V1.Id, td0.V2.Id, td1.V2.Id, td1.V1.Id})
			res = append(res, PolygonOfList(nr))
			break
		case UP:
			toDelete := EdgesOfVert(root, v)
			root, _ = avl.DeleteVal(toDelete[0], root)
			ab := Above(root, v.X, v.Y)
			abmp := Edge{ab.V1, Vertex{id, v.X, ab.ValueIn(v.X)}}
			td0 := toDelete[0]
			nr := noRedundancy(toDelete[0], abmp)
			graph = graph.AddNode(PolygonOfListIds(nr), []uint32{abmp.V1.Id, abmp.V2.Id, td0.V2.Id, td0.V1.Id})
			res = append(res, PolygonOfList(nr))
			abm := Edge{Vertex{id, v.X, ab.ValueIn(v.X)}, ab.V2}
			root = root.ModVal(ab, abm)
			tmpv := vert[edg[v.Id][0]]
			if tmpv.X < v.X {
				root = avl.InsertVal(
					Edge{vert[v.Id],
						vert[edg[v.Id][1]]}, root)
			} else {
				root = avl.InsertVal(
					Edge{vert[v.Id], tmpv}, root)
			}
			id++
			break
		case DOWN:
			toDelete := EdgesOfVert(root, v)
			root, _ = avl.DeleteVal(toDelete[0], root)
			be := Below(root, v.X, v.Y)
			bemp := Edge{be.V1, Vertex{id, v.X, be.ValueIn(v.X)}}
			td0 := toDelete[0]
			nr := noRedundancy(td0, bemp)
			graph = graph.AddNode(PolygonOfListIds(nr), []uint32{td0.V1.Id, td0.V2.Id, bemp.V2.Id, bemp.V1.Id})
			res = append(res, PolygonOfList(nr))
			bem := Edge{Vertex{id, v.X, be.ValueIn(v.X)}, be.V2}
			root = root.ModVal(be, bem)
			tmpv := vert[edg[v.Id][0]]
			if tmpv.X < v.X {
				root = avl.InsertVal(
					Edge{vert[v.Id],
						vert[edg[v.Id][1]]}, root)
			} else {
				root = avl.InsertVal(
					Edge{vert[v.Id], tmpv}, root)
			}
			id++
			break
		}
	}
	fmt.Printf("\nNUMBER OF CELLS IN GRAPH : %v\n", graph.Id)
	return res, *graph
}
