package kalman

import (
	"math"

	"github.com/regnull/kalman/geo"
)

const (
	_LAT             = 0
	_LNG             = 1
	_ALTITUDE        = 2
	_VLAT            = 3
	_VLNG            = 4
	minSpeedAccuracy = 0.1 // Meters per second.
	incline          = 5   // Degrees, used to estimate altitude random step.
)

var sqrtOf2 = math.Sqrt(2)                              // Pre-computed to speed up computations.
var inclineFactor = math.Sin(incline * math.Pi / 180.0) // Pre-computed to speed up computations.

// GeoFilter is a Kalman filter that deals with geographic coordinates and altitude.
type GeoFilter struct {
	filter *Filter
}

// GeoProcessNoise is used to initialize the process noise.
type GeoProcessNoise struct {
	// Base latitude to use for computing distances.
	BaseLat float64
	// DistancePerSecond is the expected random walk distance per second.
	DistancePerSecond float64
	// SpeedPerSecond is the expected speed per second change.
	SpeedPerSecond float64
}

// GeoObserved represents a single observation, in geographical coordinates and altitude.
type GeoObserved struct {
	Lat, Lng, Altitude float64 // Geographical coordinates (in degrees) and latitude.
	Speed              float64 // Speed, in meters per second.
	SpeedAccuracy      float64 // Speed accuracy, in meters per second.
	Direction          float64 // Travel direction, in degrees from North, 0 to 360 range.
	DirectionAccuracy  float64 // Direction accuracy, in degrees.
	HorizontalAccuracy float64 // Horizontal accuracy, in meters.
	VerticalAccuracy   float64 // Vertical accuracy, in meters.
}

// GeoEstimated contains estimated location, obtained by processing several observed locations.
type GeoEstimated struct {
	Lat, Lng, Altitude float64
	Speed              float64
	Direction          float64
	HorizontalAccuracy float64
}

// NewGeoFilter creates and returns a new GeoFilter.
func NewGeoFilter(d *GeoProcessNoise) (*GeoFilter, error) {
	metersPerDegreeLat := geo.FastMetersPerDegreeLat(d.BaseLat)
	metersPerDegreeLng := geo.FastMetersPerDegreeLng(d.BaseLat)

	dx := d.DistancePerSecond / sqrtOf2 / metersPerDegreeLat
	dy := d.DistancePerSecond / sqrtOf2 / metersPerDegreeLng
	dz := d.DistancePerSecond * inclineFactor
	dsvx := d.SpeedPerSecond / sqrtOf2 / metersPerDegreeLat
	dsvy := d.SpeedPerSecond / sqrtOf2 / metersPerDegreeLng
	dsvz := d.SpeedPerSecond * inclineFactor
	f, err := NewFilter(&ProcessNoise{
		ST:  1.0,
		SX:  dx,
		SY:  dy,
		SZ:  dz,
		SVX: dsvx,
		SVY: dsvy,
		SVZ: dsvz})
	if err != nil {
		return nil, err
	}
	return &GeoFilter{filter: f}, nil
}

func (g *GeoFilter) Observe(td float64, ob *GeoObserved) error {
	metersPerDegreeLat := geo.FastMetersPerDegreeLat(ob.Lat)
	metersPerDegreeLng := geo.FastMetersPerDegreeLng(ob.Lat)
	directionRad := ob.Direction * math.Pi / 180.0
	directionRadAccuracy := ob.DirectionAccuracy * math.Pi / 180.0
	speedLat := ob.Speed * math.Cos(directionRad) / metersPerDegreeLat
	speedLng := ob.Speed * math.Sin(directionRad) / metersPerDegreeLng
	ob1 := &Observed{
		X:   ob.Lat,
		Y:   ob.Lng,
		Z:   ob.Altitude,
		VX:  speedLat,
		VY:  speedLng,
		VZ:  0.0, // There is no way to estimate vertical speed.
		XA:  ob.HorizontalAccuracy / metersPerDegreeLat,
		YA:  ob.HorizontalAccuracy / metersPerDegreeLng,
		ZA:  ob.VerticalAccuracy,
		VXA: speedLatAccuracy(ob.Speed, ob.SpeedAccuracy, directionRad, directionRadAccuracy, metersPerDegreeLat),
		VYA: speedLngAccuracy(ob.Speed, ob.SpeedAccuracy, directionRad, directionRadAccuracy, metersPerDegreeLng),
		VZA: minSpeedAccuracy,
	}
	return g.filter.Observe(td, ob1)
}

// Estimate returns the best location estimate.
func (g *GeoFilter) Estimate() *GeoEstimated {
	if g.filter.state == nil {
		return nil
	}
	lat := g.filter.state.AtVec(_LAT)
	metersPerDegreeLat := geo.FastMetersPerDegreeLat(lat)
	metersPerDegreeLng := geo.FastMetersPerDegreeLng(lat)
	speedLatMeters := g.filter.state.AtVec(_VLAT) * metersPerDegreeLat
	speedLngMeters := g.filter.state.AtVec(_VLNG) * metersPerDegreeLng
	speed := math.Sqrt(speedLatMeters*speedLatMeters + speedLngMeters*speedLngMeters)
	haLatSquared := g.filter.cov.At(_LAT, _LAT) * metersPerDegreeLat * metersPerDegreeLat
	haLngSquared := g.filter.cov.At(_LNG, _LNG) * metersPerDegreeLng * metersPerDegreeLng
	ha := math.Max(math.Sqrt(haLatSquared), math.Sqrt(haLngSquared))

	return &GeoEstimated{
		Lat:                g.filter.state.AtVec(_LAT),
		Lng:                g.filter.state.AtVec(_LNG),
		Altitude:           g.filter.state.AtVec(_ALTITUDE),
		Speed:              speed,
		HorizontalAccuracy: ha,
	}
}

func speedLatAccuracy(speed float64, speedAccuracy float64, directionRad float64, directionRadAccuracy float64, metersPerDegreeLat float64) float64 {
	ds := math.Cos(directionRad) / metersPerDegreeLat * speedAccuracy
	dr := -speed * math.Sin(directionRad) / metersPerDegreeLat * directionRadAccuracy
	return math.Max(math.Sqrt(ds*ds+dr*dr), minSpeedAccuracy/metersPerDegreeLat)
}

func speedLngAccuracy(speed float64, speedAccuracy float64, directionRad float64, directionRadAccuracy float64, metersPerDegreeLng float64) float64 {
	ds := math.Sin(directionRad) / metersPerDegreeLng * speedAccuracy
	dr := speed * math.Cos(directionRad) / metersPerDegreeLng * directionRadAccuracy
	return math.Max(math.Sqrt(ds*ds+dr*dr), minSpeedAccuracy/metersPerDegreeLng)
}
