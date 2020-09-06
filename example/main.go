package main

import (
	"fmt"
	"os"

	"github.com/regnull/kalman"
)

// Time passed between observed data points.
var timeDelta = 10.0

// Observed data points.
var observed = []kalman.GeoObserved{
	{
		Lat:                41.154874,
		Lng:                -73.773139,
		Altitude:           105.0,
		Speed:              0.0,
		SpeedAccuracy:      0.1,
		HorizontalAccuracy: 100.0,
		VerticalAccuracy:   10.0,
	},
	{
		Lat:                41.155874,
		Lng:                -73.773239,
		Altitude:           107.0,
		Speed:              0.0,
		SpeedAccuracy:      0.1,
		HorizontalAccuracy: 50.0,
		VerticalAccuracy:   5.0,
	},
	{
		Lat:                41.154974,
		Lng:                -73.763139,
		Altitude:           99.0,
		Speed:              0.0,
		SpeedAccuracy:      0.1,
		HorizontalAccuracy: 200.0,
		VerticalAccuracy:   10.0,
	},
	{
		Lat:                41.153874,
		Lng:                -73.763139,
		Altitude:           50.0,
		Speed:              0.0,
		SpeedAccuracy:      0.1,
		HorizontalAccuracy: 1000.0,
		VerticalAccuracy:   100.0,
	},
	{
		Lat:                41.154574,
		Lng:                -73.772139,
		Altitude:           130.0,
		Speed:              0.0,
		SpeedAccuracy:      0.1,
		HorizontalAccuracy: 200.0,
		VerticalAccuracy:   50.0,
	},
}

func main() {
	var filter *kalman.GeoFilter
	var err error
	for _, point := range observed {
		if filter == nil {
			// Estimate process noise.
			processNoise := &kalman.GeoProcessNoise{
				// We assume the measurements will take place at the approximately the
				// same location, so that we can disregard the earth's curvature.
				BaseLat: point.Lat,
				// How much do we expect the user to move, meters per second.
				DistancePerSecond: 1.0,
				// How much do we expect the user's speed to change, meters per second squared.
				SpeedPerSecond: 0.1,
			}
			// Initialize Kalman filter.
			filter, err = kalman.NewGeoFilter(processNoise)
			if err != nil {
				fmt.Printf("failed to initialize Kalman filter: %s\n", err)
				os.Exit(1)
			}
		}
		// Observe the next data point.
		err = filter.Observe(timeDelta, &point)
		// Sometimes the filter may return error, although this should not happen under any
		// realistic curcumstances.
		if err != nil {
			fmt.Printf("error observing data: %s\n", err)
		}
	}
	estimated := filter.Estimate()
	fmt.Printf("Estimated lat: %f\n", estimated.Lat)
	fmt.Printf("Estimated lng: %f\n", estimated.Lng)
	fmt.Printf("Estimated alt: %f\n", estimated.Altitude)
	fmt.Printf("Estimated speed: %f\n", estimated.Speed)
	fmt.Printf("Estimated horizontal accuracy: %f\n", estimated.HorizontalAccuracy)
}
