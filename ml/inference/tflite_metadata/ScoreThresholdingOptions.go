// Code generated by the FlatBuffers compiler. DO NOT EDIT.

package tflite_metadata

import (
	flatbuffers "github.com/google/flatbuffers/go"
)

type ScoreThresholdingOptionsT struct {
	GlobalScoreThreshold float32
}

func (t *ScoreThresholdingOptionsT) Pack(builder *flatbuffers.Builder) flatbuffers.UOffsetT {
	if t == nil { return 0 }
	ScoreThresholdingOptionsStart(builder)
	ScoreThresholdingOptionsAddGlobalScoreThreshold(builder, t.GlobalScoreThreshold)
	return ScoreThresholdingOptionsEnd(builder)
}

func (rcv *ScoreThresholdingOptions) UnPackTo(t *ScoreThresholdingOptionsT) {
	t.GlobalScoreThreshold = rcv.GlobalScoreThreshold()
}

func (rcv *ScoreThresholdingOptions) UnPack() *ScoreThresholdingOptionsT {
	if rcv == nil { return nil }
	t := &ScoreThresholdingOptionsT{}
	rcv.UnPackTo(t)
	return t
}

type ScoreThresholdingOptions struct {
	_tab flatbuffers.Table
}

func GetRootAsScoreThresholdingOptions(buf []byte, offset flatbuffers.UOffsetT) *ScoreThresholdingOptions {
	n := flatbuffers.GetUOffsetT(buf[offset:])
	x := &ScoreThresholdingOptions{}
	x.Init(buf, n+offset)
	return x
}

func GetSizePrefixedRootAsScoreThresholdingOptions(buf []byte, offset flatbuffers.UOffsetT) *ScoreThresholdingOptions {
	n := flatbuffers.GetUOffsetT(buf[offset+flatbuffers.SizeUint32:])
	x := &ScoreThresholdingOptions{}
	x.Init(buf, n+offset+flatbuffers.SizeUint32)
	return x
}

func (rcv *ScoreThresholdingOptions) Init(buf []byte, i flatbuffers.UOffsetT) {
	rcv._tab.Bytes = buf
	rcv._tab.Pos = i
}

func (rcv *ScoreThresholdingOptions) Table() flatbuffers.Table {
	return rcv._tab
}

func (rcv *ScoreThresholdingOptions) GlobalScoreThreshold() float32 {
	o := flatbuffers.UOffsetT(rcv._tab.Offset(4))
	if o != 0 {
		return rcv._tab.GetFloat32(o + rcv._tab.Pos)
	}
	return 0.0
}

func (rcv *ScoreThresholdingOptions) MutateGlobalScoreThreshold(n float32) bool {
	return rcv._tab.MutateFloat32Slot(4, n)
}

func ScoreThresholdingOptionsStart(builder *flatbuffers.Builder) {
	builder.StartObject(1)
}
func ScoreThresholdingOptionsAddGlobalScoreThreshold(builder *flatbuffers.Builder, globalScoreThreshold float32) {
	builder.PrependFloat32Slot(0, globalScoreThreshold, 0.0)
}
func ScoreThresholdingOptionsEnd(builder *flatbuffers.Builder) flatbuffers.UOffsetT {
	return builder.EndObject()
}
