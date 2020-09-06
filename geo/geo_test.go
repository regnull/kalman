package geo

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestMetersPerDegreeLat(t *testing.T) {
	assert := assert.New(t)
	expected := 111034.605288
	actual := MetersPerDegreeLat(40.0)
	assert.InDelta(expected, actual, 0.01)
}

func TestMetersPerDegreeLng(t *testing.T) {
	assert := assert.New(t)
	expected := 85393.826090
	actual := MetersPerDegreeLng(40.0)
	assert.InDelta(expected, actual, 0.01)
}

func TestApproximateDistance(t *testing.T) {
	assert := assert.New(t)
	lat1 := 40.0
	lat2 := 40.0001
	lng1 := -73.0
	lng2 := -73.0001
	expected := 14.007423
	actual := ApproximateDistance(lat1, lng1, lat2, lng2)
	assert.InDelta(expected, actual, 0.01)
}

func TestDirection(t *testing.T) {
	assert := assert.New(t)
	assert.InDelta(0.0, Direction(40.0, 10.0, 40.1, 10.0), 0.01)
	assert.InDelta(45.0, Direction(40.0, 10.0, 40.1, 10.1), 0.01)
	assert.InDelta(90.0, Direction(40.0, 10.0, 40, 10.1), 0.01)
	assert.InDelta(180.0, Direction(40.0, 10.0, 39.0, 10.0), 0.01)
	assert.InDelta(270.0, Direction(40.0, 10.0, 40.0, 9.0), 0.01)
}
