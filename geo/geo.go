package geo

import "math"

// ApproximateMetersPerDegreeLat is an approximate distance per degree latitide.
var ApproximateMetersPerDegreeLat = MetersPerDegreeLat(40.0)

// ApproximateMetersPerDegreeLng is an approximate distance per degree longitude.
var ApproximateMetersPerDegreeLng = MetersPerDegreeLng(40.0)

// MetersPerDegreeLat returns meters per one degree latitude.
func MetersPerDegreeLat(lat float64) float64 {
	latRad := lat * math.Pi / 180.0
	return 111132.92 - 559.82*math.Cos(2*latRad) + 1.175*math.Cos(4*latRad) - 0.0023*math.Cos(6*latRad)
}

// MetersPerDegreeLng returns meters per one degree longitude.
func MetersPerDegreeLng(lat float64) float64 {
	latRad := lat * math.Pi / 180.0
	return 111412.84*math.Cos(latRad) - 93.5*math.Cos(3*latRad) + 0.118*math.Cos(5*latRad)
}

// ApproximateDistance returns approximate distance between two points.
func ApproximateDistance(lat1, lng1 float64, lat2, lng2 float64) float64 {
	deltaLat := (lat1 - lat2) * ApproximateMetersPerDegreeLat
	deltaLng := (lng1 - lng2) * ApproximateMetersPerDegreeLng

	return math.Sqrt(deltaLat*deltaLat + deltaLng*deltaLng)
}

// Direction returns direction in degrees from north from point 1 to point 2.
func Direction(lat1, lng1 float64, lat2, lng2 float64) float64 {
	deltaLat := lat2 - lat1
	deltaLng := lng2 - lng1
	d := math.Atan2(deltaLng, deltaLat) / math.Pi * 180.0
	if d < 0.0 {
		d += 360.0
	}
	return d
}
