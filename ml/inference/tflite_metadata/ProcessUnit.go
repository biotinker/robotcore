// Code generated by the FlatBuffers compiler. DO NOT EDIT.

package tflite_metadata

import (
	flatbuffers "github.com/google/flatbuffers/go"
)

type ProcessUnitT struct {
	Options *ProcessUnitOptionsT
}

func (t *ProcessUnitT) Pack(builder *flatbuffers.Builder) flatbuffers.UOffsetT {
	if t == nil { return 0 }
	optionsOffset := t.Options.Pack(builder)
	
	ProcessUnitStart(builder)
	if t.Options != nil {
		ProcessUnitAddOptionsType(builder, t.Options.Type)
	}
	ProcessUnitAddOptions(builder, optionsOffset)
	return ProcessUnitEnd(builder)
}

func (rcv *ProcessUnit) UnPackTo(t *ProcessUnitT) {
	optionsTable := flatbuffers.Table{}
	if rcv.Options(&optionsTable) {
		t.Options = rcv.OptionsType().UnPack(optionsTable)
	}
}

func (rcv *ProcessUnit) UnPack() *ProcessUnitT {
	if rcv == nil { return nil }
	t := &ProcessUnitT{}
	rcv.UnPackTo(t)
	return t
}

type ProcessUnit struct {
	_tab flatbuffers.Table
}

func GetRootAsProcessUnit(buf []byte, offset flatbuffers.UOffsetT) *ProcessUnit {
	n := flatbuffers.GetUOffsetT(buf[offset:])
	x := &ProcessUnit{}
	x.Init(buf, n+offset)
	return x
}

func GetSizePrefixedRootAsProcessUnit(buf []byte, offset flatbuffers.UOffsetT) *ProcessUnit {
	n := flatbuffers.GetUOffsetT(buf[offset+flatbuffers.SizeUint32:])
	x := &ProcessUnit{}
	x.Init(buf, n+offset+flatbuffers.SizeUint32)
	return x
}

func (rcv *ProcessUnit) Init(buf []byte, i flatbuffers.UOffsetT) {
	rcv._tab.Bytes = buf
	rcv._tab.Pos = i
}

func (rcv *ProcessUnit) Table() flatbuffers.Table {
	return rcv._tab
}

func (rcv *ProcessUnit) OptionsType() ProcessUnitOptions {
	o := flatbuffers.UOffsetT(rcv._tab.Offset(4))
	if o != 0 {
		return ProcessUnitOptions(rcv._tab.GetByte(o + rcv._tab.Pos))
	}
	return 0
}

func (rcv *ProcessUnit) MutateOptionsType(n ProcessUnitOptions) bool {
	return rcv._tab.MutateByteSlot(4, byte(n))
}

func (rcv *ProcessUnit) Options(obj *flatbuffers.Table) bool {
	o := flatbuffers.UOffsetT(rcv._tab.Offset(6))
	if o != 0 {
		rcv._tab.Union(obj, o)
		return true
	}
	return false
}

func ProcessUnitStart(builder *flatbuffers.Builder) {
	builder.StartObject(2)
}
func ProcessUnitAddOptionsType(builder *flatbuffers.Builder, optionsType ProcessUnitOptions) {
	builder.PrependByteSlot(0, byte(optionsType), 0)
}
func ProcessUnitAddOptions(builder *flatbuffers.Builder, options flatbuffers.UOffsetT) {
	builder.PrependUOffsetTSlot(1, flatbuffers.UOffsetT(options), 0)
}
func ProcessUnitEnd(builder *flatbuffers.Builder) flatbuffers.UOffsetT {
	return builder.EndObject()
}
