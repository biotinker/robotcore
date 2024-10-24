// Code generated by the FlatBuffers compiler. DO NOT EDIT.

package tflite_metadata

import (
	flatbuffers "github.com/google/flatbuffers/go"
)

type AssociatedFileT struct {
	Name string
	Description string
	Type AssociatedFileType
	Locale string
}

func (t *AssociatedFileT) Pack(builder *flatbuffers.Builder) flatbuffers.UOffsetT {
	if t == nil { return 0 }
	nameOffset := builder.CreateString(t.Name)
	descriptionOffset := builder.CreateString(t.Description)
	localeOffset := builder.CreateString(t.Locale)
	AssociatedFileStart(builder)
	AssociatedFileAddName(builder, nameOffset)
	AssociatedFileAddDescription(builder, descriptionOffset)
	AssociatedFileAddType(builder, t.Type)
	AssociatedFileAddLocale(builder, localeOffset)
	return AssociatedFileEnd(builder)
}

func (rcv *AssociatedFile) UnPackTo(t *AssociatedFileT) {
	t.Name = string(rcv.Name())
	t.Description = string(rcv.Description())
	t.Type = rcv.Type()
	t.Locale = string(rcv.Locale())
}

func (rcv *AssociatedFile) UnPack() *AssociatedFileT {
	if rcv == nil { return nil }
	t := &AssociatedFileT{}
	rcv.UnPackTo(t)
	return t
}

type AssociatedFile struct {
	_tab flatbuffers.Table
}

func GetRootAsAssociatedFile(buf []byte, offset flatbuffers.UOffsetT) *AssociatedFile {
	n := flatbuffers.GetUOffsetT(buf[offset:])
	x := &AssociatedFile{}
	x.Init(buf, n+offset)
	return x
}

func GetSizePrefixedRootAsAssociatedFile(buf []byte, offset flatbuffers.UOffsetT) *AssociatedFile {
	n := flatbuffers.GetUOffsetT(buf[offset+flatbuffers.SizeUint32:])
	x := &AssociatedFile{}
	x.Init(buf, n+offset+flatbuffers.SizeUint32)
	return x
}

func (rcv *AssociatedFile) Init(buf []byte, i flatbuffers.UOffsetT) {
	rcv._tab.Bytes = buf
	rcv._tab.Pos = i
}

func (rcv *AssociatedFile) Table() flatbuffers.Table {
	return rcv._tab
}

func (rcv *AssociatedFile) Name() []byte {
	o := flatbuffers.UOffsetT(rcv._tab.Offset(4))
	if o != 0 {
		return rcv._tab.ByteVector(o + rcv._tab.Pos)
	}
	return nil
}

func (rcv *AssociatedFile) Description() []byte {
	o := flatbuffers.UOffsetT(rcv._tab.Offset(6))
	if o != 0 {
		return rcv._tab.ByteVector(o + rcv._tab.Pos)
	}
	return nil
}

func (rcv *AssociatedFile) Type() AssociatedFileType {
	o := flatbuffers.UOffsetT(rcv._tab.Offset(8))
	if o != 0 {
		return AssociatedFileType(rcv._tab.GetInt8(o + rcv._tab.Pos))
	}
	return 0
}

func (rcv *AssociatedFile) MutateType(n AssociatedFileType) bool {
	return rcv._tab.MutateInt8Slot(8, int8(n))
}

func (rcv *AssociatedFile) Locale() []byte {
	o := flatbuffers.UOffsetT(rcv._tab.Offset(10))
	if o != 0 {
		return rcv._tab.ByteVector(o + rcv._tab.Pos)
	}
	return nil
}

func AssociatedFileStart(builder *flatbuffers.Builder) {
	builder.StartObject(4)
}
func AssociatedFileAddName(builder *flatbuffers.Builder, name flatbuffers.UOffsetT) {
	builder.PrependUOffsetTSlot(0, flatbuffers.UOffsetT(name), 0)
}
func AssociatedFileAddDescription(builder *flatbuffers.Builder, description flatbuffers.UOffsetT) {
	builder.PrependUOffsetTSlot(1, flatbuffers.UOffsetT(description), 0)
}
func AssociatedFileAddType(builder *flatbuffers.Builder, type_ AssociatedFileType) {
	builder.PrependInt8Slot(2, int8(type_), 0)
}
func AssociatedFileAddLocale(builder *flatbuffers.Builder, locale flatbuffers.UOffsetT) {
	builder.PrependUOffsetTSlot(3, flatbuffers.UOffsetT(locale), 0)
}
func AssociatedFileEnd(builder *flatbuffers.Builder) flatbuffers.UOffsetT {
	return builder.EndObject()
}
