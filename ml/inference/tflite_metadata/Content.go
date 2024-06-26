// Code generated by the FlatBuffers compiler. DO NOT EDIT.

package tflite_metadata

import (
	flatbuffers "github.com/google/flatbuffers/go"
)

type ContentT struct {
	ContentProperties *ContentPropertiesT
	Range *ValueRangeT
}

func (t *ContentT) Pack(builder *flatbuffers.Builder) flatbuffers.UOffsetT {
	if t == nil { return 0 }
	contentPropertiesOffset := t.ContentProperties.Pack(builder)
	
	rangeOffset := t.Range.Pack(builder)
	ContentStart(builder)
	if t.ContentProperties != nil {
		ContentAddContentPropertiesType(builder, t.ContentProperties.Type)
	}
	ContentAddContentProperties(builder, contentPropertiesOffset)
	ContentAddRange(builder, rangeOffset)
	return ContentEnd(builder)
}

func (rcv *Content) UnPackTo(t *ContentT) {
	contentPropertiesTable := flatbuffers.Table{}
	if rcv.ContentProperties(&contentPropertiesTable) {
		t.ContentProperties = rcv.ContentPropertiesType().UnPack(contentPropertiesTable)
	}
	t.Range = rcv.Range(nil).UnPack()
}

func (rcv *Content) UnPack() *ContentT {
	if rcv == nil { return nil }
	t := &ContentT{}
	rcv.UnPackTo(t)
	return t
}

type Content struct {
	_tab flatbuffers.Table
}

func GetRootAsContent(buf []byte, offset flatbuffers.UOffsetT) *Content {
	n := flatbuffers.GetUOffsetT(buf[offset:])
	x := &Content{}
	x.Init(buf, n+offset)
	return x
}

func GetSizePrefixedRootAsContent(buf []byte, offset flatbuffers.UOffsetT) *Content {
	n := flatbuffers.GetUOffsetT(buf[offset+flatbuffers.SizeUint32:])
	x := &Content{}
	x.Init(buf, n+offset+flatbuffers.SizeUint32)
	return x
}

func (rcv *Content) Init(buf []byte, i flatbuffers.UOffsetT) {
	rcv._tab.Bytes = buf
	rcv._tab.Pos = i
}

func (rcv *Content) Table() flatbuffers.Table {
	return rcv._tab
}

func (rcv *Content) ContentPropertiesType() ContentProperties {
	o := flatbuffers.UOffsetT(rcv._tab.Offset(4))
	if o != 0 {
		return ContentProperties(rcv._tab.GetByte(o + rcv._tab.Pos))
	}
	return 0
}

func (rcv *Content) MutateContentPropertiesType(n ContentProperties) bool {
	return rcv._tab.MutateByteSlot(4, byte(n))
}

func (rcv *Content) ContentProperties(obj *flatbuffers.Table) bool {
	o := flatbuffers.UOffsetT(rcv._tab.Offset(6))
	if o != 0 {
		rcv._tab.Union(obj, o)
		return true
	}
	return false
}

func (rcv *Content) Range(obj *ValueRange) *ValueRange {
	o := flatbuffers.UOffsetT(rcv._tab.Offset(8))
	if o != 0 {
		x := rcv._tab.Indirect(o + rcv._tab.Pos)
		if obj == nil {
			obj = new(ValueRange)
		}
		obj.Init(rcv._tab.Bytes, x)
		return obj
	}
	return nil
}

func ContentStart(builder *flatbuffers.Builder) {
	builder.StartObject(3)
}
func ContentAddContentPropertiesType(builder *flatbuffers.Builder, contentPropertiesType ContentProperties) {
	builder.PrependByteSlot(0, byte(contentPropertiesType), 0)
}
func ContentAddContentProperties(builder *flatbuffers.Builder, contentProperties flatbuffers.UOffsetT) {
	builder.PrependUOffsetTSlot(1, flatbuffers.UOffsetT(contentProperties), 0)
}
func ContentAddRange(builder *flatbuffers.Builder, range_ flatbuffers.UOffsetT) {
	builder.PrependUOffsetTSlot(2, flatbuffers.UOffsetT(range_), 0)
}
func ContentEnd(builder *flatbuffers.Builder) flatbuffers.UOffsetT {
	return builder.EndObject()
}
