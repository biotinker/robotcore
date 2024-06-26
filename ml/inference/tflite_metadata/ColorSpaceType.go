// Code generated by the FlatBuffers compiler. DO NOT EDIT.

package tflite_metadata

import "strconv"

type ColorSpaceType int8

const (
	ColorSpaceTypeUNKNOWN   ColorSpaceType = 0
	ColorSpaceTypeRGB       ColorSpaceType = 1
	ColorSpaceTypeGRAYSCALE ColorSpaceType = 2
)

var EnumNamesColorSpaceType = map[ColorSpaceType]string{
	ColorSpaceTypeUNKNOWN:   "UNKNOWN",
	ColorSpaceTypeRGB:       "RGB",
	ColorSpaceTypeGRAYSCALE: "GRAYSCALE",
}

var EnumValuesColorSpaceType = map[string]ColorSpaceType{
	"UNKNOWN":   ColorSpaceTypeUNKNOWN,
	"RGB":       ColorSpaceTypeRGB,
	"GRAYSCALE": ColorSpaceTypeGRAYSCALE,
}

func (v ColorSpaceType) String() string {
	if s, ok := EnumNamesColorSpaceType[v]; ok {
		return s
	}
	return "ColorSpaceType(" + strconv.FormatInt(int64(v), 10) + ")"
}
