package polygon

import "errors"
import "mathint"
import "math"
import "fmt"

func VectProd(v1 Vertex, v2 Vertex) float64 {
	return v1.X*v2.Y - v1.Y*v2.X
}

func VProd(v1 Vertex, v2 Vertex, v3 Vertex, v4 Vertex) float64 {
	dX1 := v2.X - v1.X
	dY1 := v2.Y - v1.Y
	dX2 := v4.X - v3.X
	dY2 := v4.Y - v3.Y
	return dX1*dY2 - dY1*dX2
}

func Norm(v Vertex) float64 {
	return math.Pow(math.Pow(v.X, 2)+math.Pow(v.Y, 2), 0.5)
}

func TotalLength(lpath []Vertex) float64 {
	s := float64(0)
	for i := 0; i < len(lpath)-1; i++ {
		s = s + Norm(Vertex{0, lpath[i].X - lpath[i+1].X, lpath[i].Y - lpath[i+1].Y})
	}
	return s
}

func SplitIntoPixels(v1, v2 Vertex, k int) []Vertex {
	n := Norm(Vertex{0, v2.X - v1.X, v2.Y - v1.Y}) - 1
	nInt := int(n)
	vInc := Vertex{0, float64(k) * (v2.X - v1.X) / n, float64(k) * (v2.Y - v1.Y) / n}
	l := []Vertex{v1}
	for i := 1; i < nInt/k; i++ {
		tmp := l[i-1]
		l = append(l, Vertex{0, tmp.X + vInc.X, tmp.Y + vInc.Y})
	}
	return append(l, v2)

}

func SplitPathIntoPixels(lpath []Vertex, k int) []Vertex {
	l := []Vertex{}
	for i := 0; i < len(lpath)-1; i++ {
		l = append(l, SplitIntoPixels(lpath[i], lpath[i+1], k)...)
	}
	return l
}

func SignOfVProd(v1 Vertex, v2 Vertex, v3 Vertex, v4 Vertex) int32 {
	det := VProd(v1, v2, v3, v4)
	v := Norm(Vertex{0, v2.X - v1.X, v2.Y - v1.Y})
	vp := Norm(Vertex{0, v4.X - v3.X, v4.Y - v3.Y})
	if v < 2 || vp < 2 {
		return 0
	}
	M := det / (v * vp)
	if M > math.Pow(10, -5) && M < math.Pow(10, -5) {
		return 0
	}
	if det > 0 {
		return 1
	} else {
		return -1
	}
}

func SignOfVProdId(i uint32, j uint32, k uint32, l uint32, polptr *Polygon) int32 {
	return SignOfVProd(polptr.Vertices[i], polptr.Vertices[j], polptr.Vertices[k], polptr.Vertices[l])
}

func GetOneVertice(polptr *Polygon) uint32 {
	one := uint32(0)
	for k := range polptr.Vertices {
		one = k
		break
	}
	return one
}

func FindGoodEdge(polptr *Polygon, j uint32, first uint32) uint32 {
	if polptr.Edges[j][0] == first {
		return polptr.Edges[j][1]
	} else {
		return polptr.Edges[j][0]
	}
}

func IsConvex(polptr *Polygon) bool {
	if polptr.Vertices == nil {
		return false
	}
	first := GetOneVertice(polptr)
	i := first
	j := polptr.Edges[i][0]
	k := FindGoodEdge(polptr, j, first)
	sign := SignOfVProdId(i, j, j, k, polptr)
	b := true
	for j != first {
		i = j
		j = k
		k = FindGoodEdge(polptr, j, i)
		b = b && (SignOfVProdId(i, j, j, k, polptr) == sign)
		if !b {
			break
		}
	}
	return b
}

func GetOrderListPol(polptr *Polygon) []Vertex {
	first := GetOneVertice(polptr)
	l := []Vertex{}
	i := first
	j := polptr.Edges[i][0]
	k := FindGoodEdge(polptr, j, i)
	if SignOfVProdId(i, j, j, k, polptr) == 1 {
		i, j, k = k, j, i
		first = i
	}
	l = append(l, polptr.Vertices[i], polptr.Vertices[j])
	for k != first {
		l = append(l, polptr.Vertices[k])
		i, j, k = j, k, FindGoodEdge(polptr, k, j)
	}
	return l
}

func SetFirstPos(listOfVert []Vertex) []Vertex {
	zero := Vertex{1, 0, 0}
	one := Vertex{1, 1, 0}
	first := 0
	n := uint32(len(listOfVert))
	for i, k := range listOfVert {
		if VProd(zero, one, k, listOfVert[mathint.Mod(uint32(i+1), n)]) < 0 {
			first = i
			fmt.Printf("%v\n\n", i)
			break
		}
	}
	return append(listOfVert[first:], listOfVert[:first]...)
}

func SetFirstRot(listOfVert []Vertex) []Vertex {
	zero := Vertex{1, 0, 0}
	one := Vertex{1, -1, 0}
	first := 0
	n := uint32(len(listOfVert))
	for i, k := range listOfVert {
		if VProd(zero, one, k, listOfVert[mathint.Mod(uint32(i+1), n)]) < 0 {
			first = i
			break
		}
	}
	return append(listOfVert[first:], listOfVert[:first]...)
}

func GetVects(listOfVert []Vertex) []Vertex {
	l := []Vertex{}
	n := uint32(len(listOfVert))
	for i, k := range listOfVert {
		j := listOfVert[mathint.Mod(uint32(i+1), n)]
		v := Vertex{0, j.X - k.X, j.Y - k.Y}
		l = append(l, v)
	}
	return l
}

func RotVect(listOfVert []Vertex) []Vertex {
	l := []Vertex{}
	for _, k := range listOfVert {
		v := Vertex{0, -k.X, -k.Y}
		l = append(l, v)
	}
	return l
}

func Rec(l1 []Vertex, l2 []Vertex) []Vertex {
	if len(l1) == 0 {
		return l2
	} else if len(l2) == 0 {
		return l1
	} else {
		if VectProd(l1[0], l2[0]) <= 0 {
			return append([]Vertex{l1[0]}, Rec(l1[1:], l2)...)
		} else {
			return append([]Vertex{l2[0]}, Rec(l1, l2[1:])...)
		}
	}
}

func recEraseDoublons(l []Vertex, lastSeen Vertex) []Vertex {
	if len(l) == 0 {
		return l
	} else {
		if l[0].IsEqual(lastSeen) {
			return recEraseDoublons(l[1:], lastSeen)
		} else {
			return append([]Vertex{l[0]}, recEraseDoublons(l[1:], l[0])...)
		}
	}
}

func EraseDoublons(l []Vertex) []Vertex {
	if len(l) == 0 {
		return l
	} else {
		return append([]Vertex{l[0]}, recEraseDoublons(l[1:], l[0])...)
	}
}

func SumVertVect(v Vertex, vect Vertex) Vertex {
	return Vertex{0, v.X + vect.X, v.Y + vect.Y}
}

func TotalPol(v Vertex, listOfVects []Vertex) []Vertex {
	if len(listOfVects) == 0 {
		return []Vertex{}
	} else {
		return append([]Vertex{v}, TotalPol(SumVertVect(v, listOfVects[0]), listOfVects[1:])...)
	}
}

func GetFirstVect(listOfVert []Vertex) []Vertex {
	return GetVects(SetFirstPos(SetFirstRot(listOfVert)))
}

func MinkowskiConv(pol1 *Polygon, pol2 *Polygon) (*Polygon, error) {
	if !(IsConvex(pol1) && IsConvex(pol2)) {
		return pol1, errors.New("need convexity")
	}
	l2 := GetOrderListPol(pol2)
	//fmt.Printf("***%v\n\n", l2)
	l2 = SetFirstPos(l2)
	//fmt.Printf("%v\n\n", l2)
	l2 = SetFirstRot(l2)
	//fmt.Printf("%v\n\n***", l2)

	//fmt.Printf("%v\n\n", SetFirstPos(SetFirstRot(GetOrderListPol(pol1))))
	l1 := SetFirstPos(SetFirstRot(GetOrderListPol(pol1)))
	//fmt.Printf("%v\n\n", l1)
	l3 := GetVects(l2)
	l4 := RotVect(GetVects(l1))
	ltot := Rec(l4, l3)
	totalPol := TotalPol(l2[0], ltot)
	//fmt.Printf("\nLONGUEURR : %v\n", len(totalPol))
	return PolygonOfList(&totalPol), nil
}

func FirstVertex(pol *Polygon) Vertex {
	l := SetFirstPos(SetFirstRot(GetOrderListPol(pol)))
	return l[0]
}

func Translate(pol *Polygon, vect Vertex) {
	for k, v := range pol.Vertices {
		pol.Vertices[k] = Vertex{v.Id, v.X + vect.X, v.Y + vect.Y}
	}
}

func MinkowskiGeneral(beginV Vertex, rob *[]*Polygon, obstacles *Space) *[]*Polygon {
	l := []*Polygon{}
	for _, rpol := range *rob {
		fvert := FirstVertex(rpol)
		vectToBegin := Vertex{0, beginV.X - fvert.X, beginV.Y - fvert.Y}
		for _, opol := range obstacles.Polygons {
			fmt.Printf("\n**%v\n", opol)
			res, err := MinkowskiConv(rpol, &opol)
			if err == nil {
				Translate(res, vectToBegin)
				l = append(l, res)
			}
		}
	}
	return &l
}
