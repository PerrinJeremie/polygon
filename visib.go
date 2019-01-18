package polygon

import "fmt"
import "math"

import "container/heap"

var (
	whereis map[uint32]int
)

type VGraph struct {
	Vertices map[uint32]Vertex
	AdjList  map[uint32]([]uint32)
}

func (g VGraph) NewGraph() VGraph {
	return VGraph{map[uint32]Vertex{}, map[uint32]([]uint32){}}
}

func ListToMap(l []Vertex) map[uint32]Vertex {
	m := map[uint32]Vertex{}
	for _, v := range l {
		m[v.Id] = v
	}
	return m
}

func MaxKey(m map[uint32]Vertex) uint32 {
	max := uint32(0)
	for k, _ := range m {
		if k > max {
			max = k
		} else if k == max {
			fmt.Printf("\nTwo Have Same Id\n")
		}
	}
	return max
}

func VisibilityGraphFrom(spaceAcc *Space, spaceConf *Space) VGraph {
	g := VGraph{}.NewGraph()
	lEdges := spaceAcc.ListOfEdges()
	fmt.Printf("\n%v\n", lEdges)
	lVertices := spaceAcc.ListOfVertices()
	g.Vertices = ListToMap(lVertices)
	for _, pol := range spaceAcc.Polygons {
		for k, _ := range pol.Vertices {
			for _, i := range pol.Edges[k] {
				g.AdjList[k] = append(g.AdjList[k], i)
				g.AdjList[i] = append(g.AdjList[i], k)
			}
		}
	}
	for i := 0; i < len(lVertices)-1; i++ {
		for j := i + 1; j < len(lVertices); j++ {
			b := false
			/*jtoi := Vertex{0, lVertices[i].X - lVertices[j].X, lVertices[i].Y - lVertices[j].Y}
			norm := Norm(jtoi)
			jtoi = Vertex{0, jtoi.X / norm, jtoi.Y / norm}
			vip := Vertex{0, lVertices[i].X - jtoi.X, lVertices[i].Y - jtoi.Y}
			vjp := Vertex{0, lVertices[j].X + jtoi.X, lVertices[j].Y + jtoi.Y}*/
			e := Edge{lVertices[i], lVertices[j]}
			for _, edge := range lEdges {
				eps = 2
				v, b1 := IntersectLarge(edge, e)
				eps = math.Pow(10, -3)
				if !v.IsEqualRange(lVertices[i], 1) && !v.IsEqualRange(lVertices[j], 1) {
					b = b1 || b
				}
			}
			if !b {
				b2 := false
				v := Vertex{0, (lVertices[i].X + lVertices[j].X) / 2, (lVertices[i].Y + lVertices[j].Y) / 2}
				vdir := Vertex{0, e.Begin().X - e.End().X, e.Begin().Y - e.End().Y}
				coeff := 1 / Norm(vdir)
				vorth := Vertex{0, coeff * vdir.Y, coeff * (-vdir.X)}
				v1 := Vertex{0, v.X + vorth.X, v.Y + vorth.Y}
				v2 := Vertex{0, v.X - vorth.X, v.Y - vorth.Y}
				if v1.IsInsideOneOf(&(spaceConf.Polygons)) && v2.IsInsideOneOf(&(spaceConf.Polygons)) {
					b2 = true
				}

				if !b2 {
					vidi := lVertices[i].Id
					vidj := lVertices[j].Id
					g.AdjList[vidi] = append(g.AdjList[vidi], vidj)
					g.AdjList[vidj] = append(g.AdjList[vidj], vidi)
				}
			}
		}
	}
	return g
}

func (g VGraph) AddBeginEnd(vBegin, vEnd *Vertex, spaceAcc, spaceConf *Space) VGraph {
	lEdges := spaceAcc.ListOfEdges()
	nId := MaxKey(g.Vertices)
	vBegin.Id = nId + 1
	vEnd.Id = nId + 2
	g.Vertices[nId+1] = *vBegin
	g.Vertices[nId+2] = *vEnd
	for i := uint32(1); i <= uint32(2); i++ {
		for k, v := range g.Vertices {
			if k < nId+i {
				e := Edge{g.Vertices[nId+i], v}
				b := false
				for _, edge := range lEdges {
					_, b1 := IntersectStrict(e, edge)
					b = b || b1
				}
				if !b {
					vmid := Vertex{0, (g.Vertices[nId+i].X + v.X) / 2, (g.Vertices[nId+i].Y + v.Y) / 2}
					b2 := false
					for _, pol := range spaceConf.Polygons {
						if vmid.IsInside(&pol) {
							b2 = true
							break
						}
					}
					if !b2 {
						vidi := nId + i
						vidj := v.Id
						g.AdjList[vidi] = append(g.AdjList[vidi], vidj)
						g.AdjList[vidj] = append(g.AdjList[vidj], vidi)
					}
				}
			}
		}
	}
	return g
}

func (g VGraph) PathRec(currId, eId uint32, seen *map[uint32]bool) []Vertex {
	if currId == eId {
		(*seen)[eId] = true
		return []Vertex{g.Vertices[eId]}
	} else {
		(*seen)[currId] = true
		for _, v := range g.AdjList[currId] {
			if !(*seen)[v] {
				l := g.PathRec(v, eId, seen)
				if len(l) != 0 {
					return append([]Vertex{g.Vertices[v]}, l...)
				}
			}
		}
	}
	return []Vertex{}
}

func (g VGraph) Path(bId, eId uint32) []Vertex {
	seen := map[uint32]bool{}
	for k, _ := range g.Vertices {
		seen[k] = false
	}
	l := append([]Vertex{g.Vertices[bId]}, g.PathRec(bId, eId, &seen)...)
	fmt.Printf("\n%v\n", l)
	return l
}

type Item struct {
	Id       uint32
	Distance float64
	Index    int
}

type UFHeap []*Item

func (h UFHeap) Len() int {
	return len(h)
}

func (h UFHeap) Less(i, j int) bool {
	return h[i].Distance < h[j].Distance
}

func (h UFHeap) Swap(i, j int) {
	h[i], h[j] = h[j], h[i]
	h[i].Index = j
	h[j].Index = i
	whereis[h[i].Id] = i
	whereis[h[j].Id] = j
}

func (h *UFHeap) Push(x interface{}) {
	n := len(*h)
	item := x.(*Item)
	item.Index = n
	whereis[item.Id] = n
	*h = append(*h, item)
}

func (h *UFHeap) Pop() interface{} {
	old := *h
	n := len(old)
	item := old[n-1]
	item.Index = -1
	whereis[item.Id] = -1
	*h = old[0 : n-1]
	return item
}

func (h *UFHeap) Update(item *Item, distance float64, id uint32) {
	item.Id = id
	item.Distance = distance
	heap.Fix(h, item.Index)
}

func (h *UFHeap) InitHeap(l map[uint32]Vertex, bId uint32) {
	i := 0
	whereis = map[uint32]int{}
	for id, _ := range l {
		if id != bId {
			whereis[id] = i
			(*h)[i] = &Item{
				Id:       id,
				Distance: inf,
				Index:    i}
			i++
		}
	}
	heap.Init(h)
}

func (g VGraph) EDist(id1, id2 uint32) float64 {
	v1 := g.Vertices[id1]
	v2 := g.Vertices[id2]
	return Norm(Vertex{0, v1.X - v2.X, v1.Y - v2.Y})
}

func (g VGraph) SPDijkstra(bId, eId uint32) []Vertex {
	res := []Vertex{}
	father := map[uint32]uint32{}
	h := make(UFHeap, len(g.Vertices)-1)
	h.InitHeap(g.Vertices, bId)
	for _, nId := range g.AdjList[bId] {
		h.Update(h[whereis[nId]], g.EDist(bId, nId), nId)
		father[nId] = bId
	}

	item := heap.Pop(&h).(*Item)
	for item.Id != eId && item.Distance != inf {
		fmt.Printf("\n%v\n", whereis)
		for _, vid := range g.AdjList[item.Id] {
			ind := whereis[vid]
			if ind >= 0 && vid != item.Id && vid != bId {
				nItem := h[ind]
				nDist := g.EDist(vid, item.Id) + item.Distance
				nItem.Index = ind
				fmt.Printf("\nindex : %v\n", nItem.Index)
				if nItem.Distance > nDist {
					h.Update(nItem, nDist, vid)
					father[vid] = item.Id
				}
			}
		}
		item = heap.Pop(&h).(*Item)
	}

	if h[0].Distance == inf {
		fmt.Printf("Ok")
		return res
	}

	currentId := eId
	for currentId != bId {
		res = append([]Vertex{g.Vertices[currentId]}, res...)
		currentId = father[currentId]
	}
	fmt.Printf("\n%v\n", father)
	return append([]Vertex{g.Vertices[bId]}, res...)

}
