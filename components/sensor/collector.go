package sensor

import (
	"context"

	"google.golang.org/protobuf/types/known/anypb"

	"go.viam.com/rdk/data"
)

type method int64

const (
	readings method = iota
)

func (m method) String() string {
	if m == readings {
		return "Readings"
	}
	return "Unknown"
}

// SensorRecords a collection of SensorRecord.
type SensorRecords struct {
	Readings []SensorRecord
}

// SensorRecord a single analog reading.
type SensorRecord struct {
	ReadingName  string
	Reading interface{}
}

func newSensorCollector(resource interface{}, params data.CollectorParams) (data.Collector, error) {
	sensorResource, err := assertSensor(resource)
	if err != nil {
		return nil, err
	}

	cFunc := data.CaptureFunc(func(ctx context.Context, arg map[string]*anypb.Any) (interface{}, error) {
		var records []SensorRecord
		values, err := sensorResource.Readings(ctx, nil) // TODO: pass in something here from the config rather than nil?
		if err != nil {
			return nil, data.FailedToReadErr(params.ComponentName, readings.String(), err)
		}
		for name, value := range values {
			records = append(records, SensorRecord{ReadingName: name, Reading: value})
		}
		return SensorRecords{Readings: records}, nil
	})
	return data.NewCollector(cFunc, params)
}

func assertSensor(resource interface{}) (Sensor, error) {
	sensorResource, ok := resource.(Sensor)
	if !ok {
		return nil, data.InvalidInterfaceErr(SubtypeName)
	}

	return sensorResource, nil
}
