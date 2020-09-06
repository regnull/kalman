# Kalman Filter for Geographical Coordinates in Go

Kalman filter is an algorithm that uses a series of measurements observed over time, containing statistical noise and other inaccuracies, 
and produces estimates of unknown variables that tend to be more accurate than those based on a single measurement alone, by estimating a 
joint probability distribution over the variables for each timeframe. 

https://en.wikipedia.org/wiki/Kalman_filter

This repo contains a Kalman filter implementation for motion in three dimensions, which takes speed into account but not acceleration.
It also contains a specialized implementation to work with geographical coordinates (lattitude, longitude and altitude). This
implementation may be used to process GPS measurements from mobile devices to produce a better estimate of the user's position.

## How to Use

To use, you first create an instance of GeoFilter, passing the arguments for process noise. The process noise is basically how much
you expect the user to move, per second.

```
import "github.com/regnull/kalman/kalman"

processNoise := &kalman.GeoProcessNoise{
    // Base latitude used to compute distances. Can be taken from user's location.
    BaseLat: 43.0,
    // How much do we expect user to move, meters per second.
    DistancePerSecond: 1.0,
    // How much do we expect user speed to change, meters per second squared.
    SpeedPerSecond: 0.1,
}
filter := kalman.NewGeoFilter(processNoise)

// Process observations.
observed := &kalman.GeoObserved{
    Lat: 43.0001,
    Lng: -71.0001,
    Altitude: 10.0,
    Speed: 1.0,
    Direction: 45.0,
    DirectionAccuracy: 1.0,
    HorizontalAccuracy: 55.0,
    VerticalAccuracy: 10.0,
}
// First argument to filter.Observe() is the time since last observation.
if err := filter.Observe(0.0, observed); err != nil {
    fmt.Printf("error processing location: %s", err)
}

// Observe more locations.
// ...

// Get the best estimate of the actual user location.
estimated := filter.Estimate()
fmt.Printf("lat: %f, lng: %f\n", estimated.Lat, estimated.Lng)
```

## License

This library is published under MIT license:

Copyright 2020 github.com/regnull

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

## Status

Still work in progress!