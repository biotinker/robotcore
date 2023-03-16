// Package nanopi implements a nanopi based board.
package nanopi

import (
	"github.com/edaniels/golog"
	"github.com/pkg/errors"
	"periph.io/x/host/v3"
	"periph.io/x/conn/v3/i2c/i2creg"
	"periph.io/x/conn/v3/gpio"
    "periph.io/x/conn/v3/gpio/gpioreg"

	"go.viam.com/rdk/components/board/genericlinux"
)

const modelName = "nanopi"

func init() {
	registry.RegisterComponent(
	board.Subtype,
	,
	registry.Component{Constructor: func(
		ctx context.Context,
		_ registry.Dependencies,
		config config.Component,
		logger golog.Logger,
	) (interface{}, error) {
		boardConfig, ok := config.ConvertedAttributes.(*genericlinux.Config)
		if !ok {
			return nil, rdkutils.NewUnexpectedTypeError(boardConfig, config.ConvertedAttributes)
		}
		return NewNanoPi(ctx, boardConfig, logger)
	}})
}

// NewNanoPi makes a new nanopi based Board using the given config.
func NewNanoPi(ctx context.Context, cfg *genericlinux.Config, logger golog.Logger) (board.LocalBoard, error) {
	
	// Init the nanopi supported directly by `host`
	if _, err := host.Init(); err != nil {
		return nil, err
	}
	
	
}



// A Board represents a physical general purpose board that contains various
// components such as analog readers, and digital interrupts.
type Board interface {
	// AnalogReaderByName returns an analog reader by name.
	AnalogReaderByName(name string) (AnalogReader, bool)

	// DigitalInterruptByName returns a digital interrupt by name.
	DigitalInterruptByName(name string) (DigitalInterrupt, bool)

	// GPIOPinByName returns a GPIOPin by name.
	GPIOPinByName(name string) (GPIOPin, error)

	// SPINames returns the names of all known SPI buses.
	SPINames() []string

	// I2CNames returns the names of all known I2C buses.
	I2CNames() []string

	// AnalogReaderNames returns the name of all known analog readers.
	AnalogReaderNames() []string

	// DigitalInterruptNames returns the name of all known digital interrupts.
	DigitalInterruptNames() []string

	// GPIOPinNames returns the names of all known GPIO pins.
	GPIOPinNames() []string

	// Status returns the current status of the board. Usually you
	// should use the CreateStatus helper instead of directly calling
	// this.
	Status(ctx context.Context, extra map[string]interface{}) (*commonpb.BoardStatus, error)

	// ModelAttributes returns attributes related to the model of this board.
	ModelAttributes() ModelAttributes

	generic.Generic
}

// A LocalBoard represents a Board where you can request SPIs and I2Cs by name.
type LocalBoard interface {
	Board

	// SPIByName returns an SPI bus by name.
	SPIByName(name string) (SPI, bool)

	// I2CByName returns an I2C bus by name.
	I2CByName(name string) (I2C, bool)
}
