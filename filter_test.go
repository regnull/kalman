package kalman

import (
	"fmt"
	"math"
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestInvalidProcNoise(t *testing.T) {
	assert := assert.New(t)
	d := &ProcessNoise{SX: 1.0}
	_, err := NewFilter(d)
	assert.Equal(ErrInvalidProcNoise, err)
}

func TestImproveAccuracy(t *testing.T) {
	// We test that multiple simultaneous observations with the same data reduce uncertainty.
	assert := assert.New(t)
	d := &ProcessNoise{}
	f, err := NewFilter(d)
	assert.NoError(err)
	ob := &Observed{
		X:   10.0,
		Y:   10.0,
		Z:   10.0,
		XA:  1.0,
		YA:  1.0,
		ZA:  1.0,
		VXA: 0.01,
		VYA: 0.01,
		VZA: 0.01,
	}
	// First observation.
	assert.NoError(f.Observe(0.0, ob))
	// After a single observation, the error should be the same as in the observation.

	// Second observation.
	assert.NoError(f.Observe(0.0, ob))
	assert.InDelta(0.5, f.cov.At(_X, _X), 0.1)
	assert.InDelta(0.5, f.cov.At(_Y, _Y), 0.1)
	assert.InDelta(0.5, f.cov.At(_Z, _Z), 0.1)

	// Third observation.
	assert.NoError(f.Observe(0.0, ob))
	assert.InDelta(1.0/3.0, f.cov.At(_X, _X), 0.1)
	assert.InDelta(1.0/3.0, f.cov.At(_Y, _Y), 0.1)
	assert.InDelta(1.0/3.0, f.cov.At(_Z, _Z), 0.1)
}

func TestPointInBetween(t *testing.T) {
	// Feed two separate points to the filter, the location must be in the middle.
	assert := assert.New(t)
	f, err := NewFilter(&ProcessNoise{})
	assert.NoError(err)
	ob1 := &Observed{
		X:   10.0,
		Y:   10.0,
		Z:   10.0,
		XA:  1.0,
		YA:  1.0,
		ZA:  1.0,
		VXA: 0.01,
		VYA: 0.01,
		VZA: 0.01,
	}
	assert.NoError(f.Observe(0.0, ob1))

	ob2 := &Observed{
		X:   20.0,
		Y:   20.0,
		Z:   20.0,
		XA:  1.0,
		YA:  1.0,
		ZA:  1.0,
		VXA: 0.01,
		VYA: 0.01,
		VZA: 0.01,
	}
	assert.NoError(f.Observe(0.0, ob2))
	assert.InDelta(15.0, f.state.AtVec(_X), 0.1)
	assert.InDelta(15.0, f.state.AtVec(_Y), 0.1)
	assert.InDelta(15.0, f.state.AtVec(_Z), 0.1)
}

func TestImproveAccuracyWhileMoving(t *testing.T) {
	// We test that multiple simultaneous observations with the same data reduce uncertainty.
	assert := assert.New(t)
	d := &ProcessNoise{}
	f, err := NewFilter(d)
	assert.NoError(err)
	ob := &Observed{
		X:   10.0,
		Y:   10.0,
		Z:   10.0,
		VX:  1.0,
		XA:  1.0,
		YA:  1.0,
		ZA:  1.0,
		VXA: 0.01,
		VYA: 0.01,
		VZA: 0.01,
	}
	// First observation.
	assert.NoError(f.Observe(0.0, ob))
	// After a single observation, the error should be the same as in the observation.
	assert.InDelta(10.0, f.state.AtVec(_X), 0.1)
	assert.InDelta(1.0, f.cov.At(_X, _X), 0.1)

	// Second observation.
	ob.X = 11.0
	assert.NoError(f.Observe(1.0, ob))
	assert.InDelta(11.0, f.state.AtVec(_X), 0.1)
	assert.InDelta(0.5, f.cov.At(_X, _X), 0.1)

	// Third observation.
	ob.X = 12.0
	assert.NoError(f.Observe(1.0, ob))
	assert.InDelta(12.0, f.state.AtVec(_X), 0.1)
	assert.InDelta(1.0/3.0, f.cov.At(_X, _X), 0.1)
}

func TestGoodPrecisionPoorPrecision(t *testing.T) {
	// Give the filter two points, one has good precision, one has poor precision.
	// The result must be closer to the good precision one.
	assert := assert.New(t)
	d := &ProcessNoise{}
	f, err := NewFilter(d)
	assert.NoError(err)
	goodPrecision := &Observed{
		X:   10.0,
		Y:   10.0,
		Z:   10.0,
		XA:  1.0,
		YA:  1.0,
		ZA:  1.0,
		VXA: 0.01,
		VYA: 0.01,
		VZA: 0.01,
	}
	assert.NoError(f.Observe(0.0, goodPrecision))

	poorPrecision := &Observed{
		X:   20.0,
		Y:   20.0,
		Z:   20.0,
		XA:  5.0,  // Poor precision.
		YA:  10.0, // Even worse.
		ZA:  15.0, // Oh boy even worse.
		VXA: 0.01,
		VYA: 0.01,
		VZA: 0.01,
	}
	assert.NoError(f.Observe(0.0, poorPrecision))

	assert.InDelta(10.38, f.state.AtVec(_X), 0.01)
	assert.InDelta(10.1, f.state.AtVec(_Y), 0.01)
	assert.InDelta(10.04, f.state.AtVec(_Z), 0.01)
}

func TestConvergeOnLocation(t *testing.T) {
	// Test that when we move, the filter converges on a new location.
	assert := assert.New(t)
	f, err := NewFilter(&ProcessNoise{})
	assert.NoError(err)

	ob0 := &Observed{
		X:   10.0,
		Y:   10.0,
		Z:   10.0,
		XA:  1.0,
		YA:  1.0,
		ZA:  1.0,
		VXA: 0.01,
		VYA: 0.01,
		VZA: 0.01,
	}
	assert.NoError(f.Observe(0.0, ob0))

	// TODO: Use converge()
	converged := false
	for i := 0; i < 100; i++ {
		ob1 := &Observed{
			X:   50.0,
			Y:   10.0,
			Z:   10.0,
			XA:  1.0,
			YA:  1.0,
			ZA:  1.0,
			VXA: 0.01,
			VYA: 0.01,
			VZA: 0.01,
		}
		assert.NoError(f.Observe(1.0, ob1))
		if math.Abs(f.state.AtVec(_X)-50.0) < 1.0 {
			converged = true
			break
		}
	}
	assert.True(converged)
}

func TestConvergeFasterWithMoreNoise(t *testing.T) {
	// Test that we are converging faster on the new location if there is more process noise.
	assert := assert.New(t)
	processNoise := &ProcessNoise{
		SX: 1.0,
		SY: 1.0,
		SZ: 1.0,
		ST: 1.0}
	f, err := NewFilter(processNoise)
	assert.NoError(err)

	ob0 := &Observed{
		X:   10.0,
		Y:   10.0,
		Z:   10.0,
		XA:  1.0,
		YA:  1.0,
		ZA:  1.0,
		VXA: 0.01,
		VYA: 0.01,
		VZA: 0.01}

	ob1 := &Observed{
		X:   200.0,
		Y:   200.0,
		Z:   200.0,
		XA:  1.0,
		YA:  1.0,
		ZA:  1.0,
		VXA: 0.01,
		VYA: 0.01,
		VZA: 0.01}

	n1, err := converge(f, ob0, ob1, 1.0, 1.0, 100)
	assert.NoError(err)
	assert.True(n1 > 0)

	// Increase process noise, converge again.
	processNoise.SX = 2.0
	processNoise.SY = 2.0
	processNoise.SZ = 2.0

	f, err = NewFilter(processNoise)
	assert.NoError(err)
	n2, err := converge(f, ob0, ob1, 1.0, 1.0, 100)
	assert.NoError(err)
	assert.True(n2 > 0)
	assert.True(n2 < n1)
}

func converge(filter *Filter, loc0, loc1 *Observed, td float64, distance float64, maxIter int) (int, error) {
	if err := filter.Observe(0.0, loc0); err != nil {
		return 0, err
	}

	for i := 0; i < maxIter; i++ {
		if err := filter.Observe(td, loc1); err != nil {
			return 0, err
		}
		dx := filter.state.AtVec(_X) - loc1.X
		dy := filter.state.AtVec(_Y) - loc1.Y
		dz := filter.state.AtVec(_Z) - loc1.Z
		d := math.Sqrt(dx*dx + dy*dy + dz*dz)
		if d < distance {
			return i, nil
		}
	}
	return maxIter, fmt.Errorf("max iteration reached")
}
