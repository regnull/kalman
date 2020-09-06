package kalman

import (
	"fmt"
	"math"
	"testing"

	"github.com/regnull/kalman/geo"
	"github.com/stretchr/testify/assert"
)

func TestSpeedLatAccuracy(t *testing.T) {
	assert := assert.New(t)
	dirRad := 45.0 * math.Pi / 180.0
	dirRadAccuracy := 5.0 * math.Pi / 180.0
	metersPerDegreeLat := geo.FastMetersPerDegreeLat(43.0)
	acc := speedLatAccuracy(10.0, 1.0, dirRad, dirRadAccuracy, metersPerDegreeLat)
	assert.InDelta(8.447849136425959e-06, acc, 0.01)
}

func TestSpeedLngAccuracy(t *testing.T) {
	assert := assert.New(t)
	dirRad := 45.0 * math.Pi / 180.0
	dirRadAccuracy := 5.0 * math.Pi / 180.0
	metersPerDegreeLng := geo.FastMetersPerDegreeLng(43.0)
	acc := speedLngAccuracy(10.0, 1.0, dirRad, dirRadAccuracy, metersPerDegreeLng)
	assert.InDelta(1.1509487689869777e-05, acc, 0.01)
}

func TestImproveGeoAccuracy(t *testing.T) {
	// Test that repeated simultaneous observations improve precision.
	assert := assert.New(t)
	g, err := NewGeoFilter(&GeoProcessNoise{})
	assert.NoError(err)
	ob := &GeoObserved{
		Lat:                43.0,
		Lng:                -71.0,
		Altitude:           100.0,
		Speed:              0.0,
		SpeedAccuracy:      0.01,
		Direction:          0.0,
		DirectionAccuracy:  5.0,
		HorizontalAccuracy: 100.0,
		VerticalAccuracy:   10.0,
	}
	// First observation.
	assert.NoError(g.Observe(0.0, ob))
	e := g.Estimate()
	assert.InDelta(43.0, e.Lat, 0.01)
	assert.InDelta(-71.0, e.Lng, 0.01)
	assert.InDelta(100.0, e.HorizontalAccuracy, 0.01)

	// Second observation.
	assert.NoError(g.Observe(0.0, ob))
	e = g.Estimate()
	assert.InDelta(43.0, e.Lat, 0.01)
	assert.InDelta(-71.0, e.Lng, 0.01)
	// TODO: This should really be 50.0
	assert.InDelta(70.71067811865476, e.HorizontalAccuracy, 0.01)

	// Third observation.
	assert.NoError(g.Observe(0.0, ob))
	e = g.Estimate()
	assert.InDelta(43.0, e.Lat, 0.01)
	assert.InDelta(-71.0, e.Lng, 0.01)
	assert.InDelta(57.73502691896258, e.HorizontalAccuracy, 0.01)
}

func TestMoreRecentIsBetter(t *testing.T) {
	// Test that more recent observation is preferred over the past ones.
	assert := assert.New(t)
	g, err := NewGeoFilter(&GeoProcessNoise{
		BaseLat:           43.0,
		DistancePerSecond: 1.0, // Meters.
		SpeedPerSecond:    0.1, // Meters per second.
	})
	assert.NoError(err)
	ob := &GeoObserved{
		Lat:                43.0,
		Lng:                -71.0,
		Altitude:           100.0,
		Speed:              0.0,
		SpeedAccuracy:      0.01,
		Direction:          0.0,
		DirectionAccuracy:  5.0,
		HorizontalAccuracy: 10.0,
		VerticalAccuracy:   10.0,
	}
	assert.NoError(g.Observe(0.0, ob))
	e := g.Estimate()
	assert.InDelta(43.0, e.Lat, 0.01)
	assert.InDelta(-71.0, e.Lng, 0.01)
	assert.InDelta(10.0, e.HorizontalAccuracy, 0.0001)

	// Second observation is a bit further away, and it happens 10 seconds later.
	ob.Lat = 43.01
	assert.NoError(g.Observe(100.0, ob))
	e = g.Estimate()
	assert.InDelta(43.006, e.Lat, 0.0001)
}

func TestGeoConvergeOnLocation(t *testing.T) {
	assert := assert.New(t)
	g, err := NewGeoFilter(&GeoProcessNoise{
		BaseLat:           43.0,
		DistancePerSecond: 1.0, // Meters.
		SpeedPerSecond:    0.1, // Meters per second.
	})
	assert.NoError(err)

	ob0 := &GeoObserved{
		Lat:                43.0,
		Lng:                -71.0,
		Altitude:           100.0,
		Speed:              0.0,
		SpeedAccuracy:      0.01,
		Direction:          0.0,
		DirectionAccuracy:  5.0,
		HorizontalAccuracy: 10.0,
		VerticalAccuracy:   10.0,
	}

	ob1 := &GeoObserved{
		Lat:                43.0,
		Lng:                -71.001,
		Altitude:           100.0,
		Speed:              0.0,
		SpeedAccuracy:      0.01,
		Direction:          0.0,
		DirectionAccuracy:  5.0,
		HorizontalAccuracy: 10.0,
		VerticalAccuracy:   10.0,
	}
	n, err := geoConverge(g, ob0, ob1, 1.0, 1.0, 100)
	assert.NoError(err)
	fmt.Printf("converged in %d iterations\n", n)
	assert.True(n > 0)
}

func TestGeoConvergeFasterWithMoreNoise(t *testing.T) {
	assert := assert.New(t)
	g, err := NewGeoFilter(&GeoProcessNoise{
		BaseLat:           43.0,
		DistancePerSecond: 1.0, // Meters.
		SpeedPerSecond:    0.1, // Meters per second.
	})
	assert.NoError(err)

	ob0 := &GeoObserved{
		Lat:                43.0,
		Lng:                -71.0,
		Altitude:           100.0,
		Speed:              0.0,
		SpeedAccuracy:      0.01,
		Direction:          0.0,
		DirectionAccuracy:  5.0,
		HorizontalAccuracy: 10.0,
		VerticalAccuracy:   10.0,
	}

	ob1 := &GeoObserved{
		Lat:                43.0,
		Lng:                -71.001,
		Altitude:           100.0,
		Speed:              0.0,
		SpeedAccuracy:      0.01,
		Direction:          0.0,
		DirectionAccuracy:  5.0,
		HorizontalAccuracy: 10.0,
		VerticalAccuracy:   10.0,
	}
	n1, err := geoConverge(g, ob0, ob1, 1.0, 1.0, 100)
	assert.NoError(err)
	fmt.Printf("1: converged in %d iterations\n", n1)
	assert.True(n1 > 0)

	// Increase noise.
	g, err = NewGeoFilter(&GeoProcessNoise{
		BaseLat:           43.0,
		DistancePerSecond: 2.0, // Meters.
		SpeedPerSecond:    0.1, // Meters per second.
	})

	n2, err := geoConverge(g, ob0, ob1, 1.0, 1.0, 100)
	assert.NoError(err)
	fmt.Printf("2: converged in %d iterations\n", n2)
	assert.True(n2 > 0)
	assert.True(n2 < n1)
}

func geoConverge(filter *GeoFilter, loc0, loc1 *GeoObserved, td float64, distance float64, maxIter int) (int, error) {
	if err := filter.Observe(td, loc0); err != nil {
		return 0, err
	}

	for i := 0; i < maxIter; i++ {
		if err := filter.Observe(td, loc1); err != nil {
			return 0, err
		}
		e := filter.Estimate()
		if e == nil {
			return 0, fmt.Errorf("cannot estimate location")
		}
		dlat := e.Lat - loc1.Lat
		dlng := e.Lng - loc1.Lng
		dx := dlat * geo.MetersPerDegreeLat(loc1.Lat)
		dy := dlng * geo.MetersPerDegreeLng(loc1.Lng)
		d := math.Sqrt(dx*dx + dy*dy)
		if d < distance {
			return i, nil
		}
	}
	return maxIter, fmt.Errorf("max iteration reached")
}
