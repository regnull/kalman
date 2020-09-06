# Kalman Filter for Geographical Coordinates in Go

Kalman filter is an algorithm that uses a series of measurements observed over time, containing statistical noise and other inaccuracies, 
and produces estimates of unknown variables that tend to be more accurate than those based on a single measurement alone, by estimating a 
joint probability distribution over the variables for each timeframe. 

https://en.wikipedia.org/wiki/Kalman_filter

This repo contains a Kalman filter implementation for motion in three dimensions, which takes speed into account but not acceleration.
It also contains a specialized implementation to work with geographical coordinates (lattitude, longitude and altitude). This
implementation may be used to process GPS measurements from mobile devices to produce a better estimate of the user's position.

## How to Use

See the example here:

https://github.com/regnull/kalman/blob/master/example/main.go

Basically, using Kalman filter involves the following steps:

* Estimate the process noise
* Create Kalman filter
* Feed observed values to the filter, one by one
* Get the estimated location, hopefully with better precision than any of the observed points

### Estimate the process noise

Process noise tells the filter how much do we expect the user to move, per second. Knowing this value allows the algorithm to estimate how much of the difference between expected and actual values can be attributed to the actual user motion.

Here, we create the process noise struct:

```
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
```

You must assign non-zero values to DistancePerSecond and SpeedPerSecond, otherwise you will get an error when creating an instance of the filter.

### Create Kalman filter

After you create process noise struct, pass it to NetGeoFilter:

```
// Initialize Kalman filter.
filter, err = kalman.NewGeoFilter(processNoise)
if err != nil {
    fmt.Printf("failed to initialize Kalman filter: %s\n", err)
    os.Exit(1)
}
```

You might get an error back if the filter doesn't like your process noise values.

### Feed observed values to the filter

GeoFilter.Observe() takes two arguments, the time passed since the last measurement and the new observed value. Easy enough.

```
// Observe the next data point.
err = filter.Observe(timeDelta, &point)
// Sometimes the filter may return error, although this should not happen under any
// realistic curcumstances.
if err != nil {
    fmt.Printf("error observing data: %s\n", err)
}
```

Where point contains data like this:

```
kalman.GeoObserved { 
    Lat:                41.154874,
    Lng:                -73.773139,
    Altitude:           105.0,
    Speed:              0.0,
    SpeedAccuracy:      0.1,
    HorizontalAccuracy: 100.0,
    VerticalAccuracy:   10.0,
}
```

### Get the estimated values

Finally, get the estimated values obtained by processing the observed values. 

```
estimated := filter.Estimate()
fmt.Printf("Estimated lat: %f\n", estimated.Lat)
fmt.Printf("Estimated lng: %f\n", estimated.Lng)
fmt.Printf("Estimated alt: %f\n", estimated.Altitude)
fmt.Printf("Estimated speed: %f\n", estimated.Speed)
fmt.Printf("Estimated horizontal accuracy: %f\n", estimated.HorizontalAccuracy)
```

## License

This library is published under MIT license:

Copyright 2020 Leonid Gorkin

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

## Status

Work in progress.