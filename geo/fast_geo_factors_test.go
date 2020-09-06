package geo

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestFastMetersPerDegreeLat(t *testing.T) {
	assert := assert.New(t)
	assert.InDelta(110574.2727, FastMetersPerDegreeLat(0.0), 0.001)
	assert.InDelta(111693.9173, FastMetersPerDegreeLat(90.0), 0.001)
	assert.InDelta(110575.793897, FastMetersPerDegreeLat(2.1), 0.001)
}

func TestFastMetersPerDegreeLng(t *testing.T) {
	assert := assert.New(t)
	assert.InDelta(111319.458, FastMetersPerDegreeLng(0.0), 0.001)
	assert.InDelta(0.0, FastMetersPerDegreeLng(90.0), 0.001)
	assert.InDelta(111243.6806156, FastMetersPerDegreeLng(2.1), 0.001)
}
