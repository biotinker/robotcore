// Code generated by the FlatBuffers compiler. DO NOT EDIT.

package tflite

import (
	flatbuffers "github.com/google/flatbuffers/go"
)

type CastOptions struct {
	_tab flatbuffers.Table
}

func GetRootAsCastOptions(buf []byte, offset flatbuffers.UOffsetT) *CastOptions {
	n := flatbuffers.GetUOffsetT(buf[offset:])
	x := &CastOptions{}
	x.Init(buf, n+offset)
	return x
}

func GetSizePrefixedRootAsCastOptions(buf []byte, offset flatbuffers.UOffsetT) *CastOptions {
	n := flatbuffers.GetUOffsetT(buf[offset+flatbuffers.SizeUint32:])
	x := &CastOptions{}
	x.Init(buf, n+offset+flatbuffers.SizeUint32)
	return x
}

func (rcv *CastOptions) Init(buf []byte, i flatbuffers.UOffsetT) {
	rcv._tab.Bytes = buf
	rcv._tab.Pos = i
}

func (rcv *CastOptions) Table() flatbuffers.Table {
	return rcv._tab
}

func (rcv *CastOptions) InDataType() TensorType {
	o := flatbuffers.UOffsetT(rcv._tab.Offset(4))
	if o != 0 {
		return TensorType(rcv._tab.GetInt8(o + rcv._tab.Pos))
	}
	return 0
}

func (rcv *CastOptions) MutateInDataType(n TensorType) bool {
	return rcv._tab.MutateInt8Slot(4, int8(n))
}

func (rcv *CastOptions) OutDataType() TensorType {
	o := flatbuffers.UOffsetT(rcv._tab.Offset(6))
	if o != 0 {
		return TensorType(rcv._tab.GetInt8(o + rcv._tab.Pos))
	}
	return 0
}

func (rcv *CastOptions) MutateOutDataType(n TensorType) bool {
	return rcv._tab.MutateInt8Slot(6, int8(n))
}

func CastOptionsStart(builder *flatbuffers.Builder) {
	builder.StartObject(2)
}
func CastOptionsAddInDataType(builder *flatbuffers.Builder, inDataType TensorType) {
	builder.PrependInt8Slot(0, int8(inDataType), 0)
}
func CastOptionsAddOutDataType(builder *flatbuffers.Builder, outDataType TensorType) {
	builder.PrependInt8Slot(1, int8(outDataType), 0)
}
func CastOptionsEnd(builder *flatbuffers.Builder) flatbuffers.UOffsetT {
	return builder.EndObject()
}
