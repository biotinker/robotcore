// Code generated by the FlatBuffers compiler. DO NOT EDIT.

package tflite

import (
	flatbuffers "github.com/google/flatbuffers/go"
)

type CustomQuantization struct {
	_tab flatbuffers.Table
}

func GetRootAsCustomQuantization(buf []byte, offset flatbuffers.UOffsetT) *CustomQuantization {
	n := flatbuffers.GetUOffsetT(buf[offset:])
	x := &CustomQuantization{}
	x.Init(buf, n+offset)
	return x
}

func GetSizePrefixedRootAsCustomQuantization(buf []byte, offset flatbuffers.UOffsetT) *CustomQuantization {
	n := flatbuffers.GetUOffsetT(buf[offset+flatbuffers.SizeUint32:])
	x := &CustomQuantization{}
	x.Init(buf, n+offset+flatbuffers.SizeUint32)
	return x
}

func (rcv *CustomQuantization) Init(buf []byte, i flatbuffers.UOffsetT) {
	rcv._tab.Bytes = buf
	rcv._tab.Pos = i
}

func (rcv *CustomQuantization) Table() flatbuffers.Table {
	return rcv._tab
}

func (rcv *CustomQuantization) Custom(j int) byte {
	o := flatbuffers.UOffsetT(rcv._tab.Offset(4))
	if o != 0 {
		a := rcv._tab.Vector(o)
		return rcv._tab.GetByte(a + flatbuffers.UOffsetT(j*1))
	}
	return 0
}

func (rcv *CustomQuantization) CustomLength() int {
	o := flatbuffers.UOffsetT(rcv._tab.Offset(4))
	if o != 0 {
		return rcv._tab.VectorLen(o)
	}
	return 0
}

func (rcv *CustomQuantization) CustomBytes() []byte {
	o := flatbuffers.UOffsetT(rcv._tab.Offset(4))
	if o != 0 {
		return rcv._tab.ByteVector(o + rcv._tab.Pos)
	}
	return nil
}

func (rcv *CustomQuantization) MutateCustom(j int, n byte) bool {
	o := flatbuffers.UOffsetT(rcv._tab.Offset(4))
	if o != 0 {
		a := rcv._tab.Vector(o)
		return rcv._tab.MutateByte(a+flatbuffers.UOffsetT(j*1), n)
	}
	return false
}

func CustomQuantizationStart(builder *flatbuffers.Builder) {
	builder.StartObject(1)
}
func CustomQuantizationAddCustom(builder *flatbuffers.Builder, custom flatbuffers.UOffsetT) {
	builder.PrependUOffsetTSlot(0, flatbuffers.UOffsetT(custom), 0)
}
func CustomQuantizationStartCustomVector(builder *flatbuffers.Builder, numElems int) flatbuffers.UOffsetT {
	return builder.StartVector(1, numElems, 1)
}
func CustomQuantizationEnd(builder *flatbuffers.Builder) flatbuffers.UOffsetT {
	return builder.EndObject()
}
