package ode

type CGOMap struct {
	index    int
	pointers map[int]interface{}
}

func NewCGOMap() CGOMap {
	var m CGOMap
	m.pointers = make(map[int]interface{})
	return m
}

func (m CGOMap) Get(index int) interface{} {
	p := m.pointers[index]
	if p == nil {
		panic("couldn't retrieve the pointer")
	}
	return p
}

func (m CGOMap) Set(p interface{}) int {
	m.index += 1
	m.pointers[m.index] = p
	return m.index
}

func (m CGOMap) Delete(index int) {
	delete(m.pointers, index)
}
