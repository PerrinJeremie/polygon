package polygon

import "fmt"
import "mathint"
import "sort"
import "math"

type Vertex struct {
	Id uint32
	X  float64
	Y  float64
}

type Polygon struct {
	Vertices map[uint32]Vertex
	Edges    map[uint32]([]uint32)
}

type Space struct {
	Polygons    []Polygon
	PolygonOfId map[uint32]*Polygon
	Id          uint32
}

func (s Space) ListOfEdges() []Edge {
	l := []Edge{}
	for _, pol := range s.Polygons {
		begin := GetOneVertice(&pol)
		first := begin
		second := pol.Edges[first][0]
		l = append(l, Edge{pol.Vertices[first], pol.Vertices[second]})
		keepgoing := true
		for keepgoing {
			tmp := second
			second = FindGoodEdge(&pol, second, first)
			first = tmp
			l = append(l, Edge{pol.Vertices[first], pol.Vertices[second]})
			keepgoing = !(second == begin)
		}
	}
	return l
}

func (pol Polygon) ListOfVertices() []Vertex {
	l := []Vertex{}
	for _, v := range pol.Vertices {
		l = append(l, v)
	}
	return l
}

func (s Space) ListOfVertices() []Vertex {
	l := []Vertex{}
	for _, pol := range s.Polygons {
		l = append(l, pol.ListOfVertices()...)
	}
	return l
}

func (v Vertex) SameX(l *[]Vertex) bool {
	for _, v2 := range *l {
		if Norm(Vertex{0, v.X - v2.X, 0}) < 5 {
			return true
		}
	}
	return false
}

func (v Vertex) IsEqual(v1 Vertex) bool {
	diff := Norm(Vertex{0, v1.X - v.X, v1.Y - v.Y})
	return (diff < 0.5)
}

func (v Vertex) IsEqualRange(v1 Vertex, npix float64) bool {
	diff := Norm(Vertex{0, v1.X - v.X, v1.Y - v.Y})
	return (diff < npix)
}

func (v Vertex) IsInside(polptr *Polygon) bool {
	keepgoing := true
	begin := GetOneVertice(polptr)
	first := begin
	second := polptr.Edges[first][0]
	vert := polptr.Vertices
	sign := SignOfVProd(v, vert[first], vert[first], vert[second])
	for keepgoing {
		tmp := second
		second = FindGoodEdge(polptr, second, first)
		first = tmp
		if first == begin {
			keepgoing = false
		}
		s := SignOfVProd(v, vert[first], vert[first], vert[second])
		if keepgoing && s != sign {
			return false
		}
	}
	return true
}

func (v Vertex) IsInsideOneOf(lpol *[]Polygon) bool {
	for _, polptr := range *lpol {
		if v.IsInside(&polptr) {
			return true
		}
	}
	return false
}

func NewEmptyPolygon() Polygon {
	return Polygon{map[uint32]Vertex{}, map[uint32]([]uint32){}}
}

func NewEmptySpace() Space {
	return Space{[]Polygon{}, map[uint32]*Polygon{}, 0}
}

func MapVertexId(m *map[uint32]uint32, s *[]uint32) *[]uint32 {
	rs := []uint32{}
	for _, k := range *s {
		rs = append(rs, (*m)[k])
	}
	return &rs
}

func GetIdsOfSpace(sp *Space) *[]uint32 {
	l := []uint32{}
	for _, pol := range sp.Polygons {
		for _, v := range pol.Vertices {
			l = append(l, v.Id)
		}
	}
	return &l
}

func UnionOf(pols *[]Polygon) (map[uint32]Vertex, map[uint32]([]uint32)) {
	l := map[uint32]Vertex{}
	m := map[uint32]([]uint32){}
	for _, pol := range *pols {
		for k, v := range pol.Vertices {
			l[k] = v
		}
		for k, v := range pol.Edges {
			m[k] = v
		}
	}
	return l, m
}

func GetVertexOfId(id uint32, sp *Space) Vertex {
	return (sp.PolygonOfId[id]).Vertices[id]
}

func GetSortedIdsOfSpace(sp *Space) *[]uint32 {
	l := GetIdsOfSpace(sp)
	sort.Slice(*l, func(i, j int) bool {
		vi := GetVertexOfId((*l)[i], sp)
		vj := GetVertexOfId((*l)[j], sp)
		return vi.X < vj.X
	})
	return l
}

func ModId(vptr *Vertex, id uint32) {
	vptr.Id = id
}

func renamePolygonForSpace(poly *Polygon, sp *Space) {
	newMap := map[uint32]uint32{}
	newVert := map[uint32]Vertex{}
	newEdges := map[uint32]([]uint32){}

	for k, v := range poly.Vertices {
		newMap[k] = sp.Id
		v.Id = sp.Id
		newVert[sp.Id] = Vertex{sp.Id, v.X, v.Y}
		sp.Id = sp.Id + 1
	}

	for k, v := range poly.Edges {
		newEdges[newMap[k]] = *(MapVertexId(&newMap, &v))
	}

	poly.Vertices = newVert
	poly.Edges = newEdges
}

func AddPolygonToSpace(poly *Polygon, sp *Space) {
	renamePolygonForSpace(poly, sp)
	sp.Polygons = append(sp.Polygons, *poly)
	polptr := sp.Polygons[len(sp.Polygons)-1]
	for _, v := range poly.Vertices {
		sp.PolygonOfId[v.Id] = &polptr
	}
}

func PolygonOfList(listOfVertices *[]Vertex) *Polygon {
	vert := map[uint32]Vertex{}
	edges := map[uint32]([]uint32){}
	n := uint32(len(*listOfVertices))
	for i, k := range *listOfVertices {
		j := uint32(i)
		k.Id = j
		vert[j] = k
		edges[j] = []uint32{mathint.Mod(j+1, n), mathint.Mod(n+j-1, n)}
	}
	pol := Polygon{vert, edges}
	return &pol
}

func PolygonOfListIds(listOfVertices *[]Vertex) *Polygon {
	vert := map[uint32]Vertex{}
	edges := map[uint32]([]uint32){}
	n := uint32(len(*listOfVertices))
	for i, k := range *listOfVertices {
		j := uint32(i)
		vert[k.Id] = k
		edges[k.Id] = []uint32{(*listOfVertices)[mathint.Mod(j+1, n)].Id, (*listOfVertices)[mathint.Mod(n+j-1, n)].Id}
	}
	pol := Polygon{vert, edges}
	return &pol
}

func AddListToSpaceAsPoly(listOfVertices *[]Vertex, sp *Space) {
	vert := map[uint32]Vertex{}
	edges := map[uint32]([]uint32){}
	L := uint32(len(*listOfVertices))
	for i, k := range *listOfVertices {
		j := uint32(i)
		vert[j] = k
		edges[j] = []uint32{mathint.Mod(j+1, L), mathint.Mod(L+j-1, L)}
	}
	pol := Polygon{vert, edges}
	AddPolygonToSpace(&pol, sp)
}

func main() {

	v := []Vertex{}

	for k := 0; k < 10; k++ {
		v = append(v, Vertex{uint32(k) + 1, float64(k), -float64(k)})
	}

	vertices := map[uint32]Vertex{}
	edges := map[uint32]([]uint32){}

	for _, k := range v {
		vertices[k.Id] = k
		edges[k.Id] = []uint32{1 + uint32(math.Mod(float64(k.Id), 10.0))}
	}
	pol := Polygon{vertices, edges}
	s := NewEmptySpace()
	//fmt.Printf("%v\n", pol)
	AddPolygonToSpace(&pol, &s)
	//fmt.Printf("%v\n", pol)
	//fmt.Printf("%v\n", *s.PolygonOfVertex[0])
	fmt.Printf("%v\n", s)
	fmt.Printf("%v\n", GetSortedIdsOfSpace(&s))
}
