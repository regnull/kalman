package kalman

// See https://en.wikipedia.org/wiki/Kalman_filter

import (
	"fmt"

	"gonum.org/v1/gonum/mat"
)

const (
	_N = 6 // Size of the matrixes and vectors.

	// Symbolic names for rows/columns.
	_X  = 0
	_Y  = 1
	_Z  = 2
	_VX = 3
	_VY = 4
	_VZ = 5
)

// ErrInvalidProcNoise is returned when we can't compute process noise.
var ErrInvalidProcNoise = fmt.Errorf("invalid process noise arguments")

// Filter is a Kalman filter.
type Filter struct {
	state     mat.Vector // State.
	cov       mat.Matrix // Covariance.
	procNoise mat.Matrix // Process noise.
}

// ProcessNoise represents process noise.
type ProcessNoise struct {
	SX, SY, SZ    float64 // Random step (coordinates).
	SVX, SVY, SVZ float64 // Random step (speed).
	ST            float64 // Random step (time).
}

// Observed represents a single observation.
type Observed struct {
	X, Y, Z       float64 // Coordinates.
	VX, VY, VZ    float64 // Speed.
	XA, YA, ZA    float64 // Accuracy (coordinates).
	VXA, VYA, VZA float64 // Accuracy (speed).
}

// NewFilter creates and returns a new Kalman filter.
func NewFilter(d *ProcessNoise) (*Filter, error) {
	if d.ST == 0 && (d.SX > 0 || d.SY > 0 || d.SZ > 0 || d.SVX > 0 || d.SVY > 0 || d.SVZ > 0) {
		return nil, ErrInvalidProcNoise
	}
	// Init process noise.
	procNoise := mat.NewDense(_N, _N, nil)
	if d.ST > 0 {
		procNoise.Set(_X, _X, d.SX*d.SX/d.ST)
		procNoise.Set(_Y, _Y, d.SY*d.SY/d.ST)
		procNoise.Set(_Z, _Z, d.SZ*d.SZ/d.ST)
		procNoise.Set(_VX, _VX, d.SVX*d.SVX/d.ST)
		procNoise.Set(_VY, _VY, d.SVY*d.SVY/d.ST)
		procNoise.Set(_VZ, _VZ, d.SVZ*d.SVZ/d.ST)
	}
	return &Filter{procNoise: procNoise}, nil
}

func (f *Filter) initCov(ob *Observed) {
	cov := mat.NewDense(_N, _N, nil)
	cov.Set(_X, _X, ob.XA*ob.XA)
	cov.Set(_Y, _Y, ob.YA*ob.YA)
	cov.Set(_Z, _Z, ob.ZA*ob.ZA)
	cov.Set(_VX, _VX, ob.VXA*ob.VXA)
	cov.Set(_VY, _VY, ob.VYA*ob.VYA)
	cov.Set(_VZ, _VZ, ob.VZA*ob.VZA)
	f.cov = cov
}

func (f *Filter) predictState(td float64) mat.Vector {
	m := mat.DenseCopyOf(eye(_N))
	m.Set(_X, _VX, td)
	m.Set(_Y, _VY, td)
	m.Set(_Z, _VZ, td)

	newState := mat.NewVecDense(6, nil)
	newState.MulVec(m, f.state)
	return newState
}

func (f *Filter) predictCov(td float64) mat.Matrix {
	m := mat.DenseCopyOf(eye(6))
	m.Set(_X, _VX, td)
	m.Set(_Y, _VY, td)
	m.Set(_Z, _VZ, td)

	var w mat.Dense
	w.Scale(td, f.procNoise)
	var r mat.Dense
	r.Mul(f.cov, m.T())
	r.Add(&r, &w)
	return &r
}

func (f *Filter) kalmanGain(predCov mat.Matrix, ob *Observed) (mat.Matrix, error) {
	r := mat.NewDense(_N, _N, nil)
	r.Set(_X, _X, ob.XA*ob.XA)
	r.Set(_Y, _Y, ob.YA*ob.YA)
	r.Set(_Z, _Z, ob.ZA*ob.ZA)
	r.Set(_VX, _VX, ob.VXA*ob.VXA)
	r.Set(_VY, _VY, ob.VYA*ob.VYA)
	r.Set(_VZ, _VZ, ob.VZA*ob.VZA)
	var t mat.Dense
	t.Add(predCov, r)
	var it mat.Dense
	err := it.Inverse(&t)
	if err != nil {
		return nil, err
	}
	var q mat.Dense
	q.Mul(predCov, &it)
	return &q, nil
}

// Observe processes a single act of observation, td is the time since last update.
func (f *Filter) Observe(td float64, ob *Observed) error {
	if f.state == nil {
		f.initCov(ob)
		f.state = mat.NewVecDense(_N, []float64{ob.X, ob.Y, ob.Z, ob.VX, ob.VY, ob.VZ})
		return nil
	}

	predState := f.predictState(td)
	predCov := f.predictCov(td)
	k, err := f.kalmanGain(predCov, ob)
	if err != nil {
		return err
	}

	obState := mat.NewVecDense(_N, []float64{ob.X, ob.Y, ob.Z, ob.VX, ob.VY, ob.VZ})
	var stateDif mat.VecDense
	stateDif.SubVec(obState, predState)
	var r mat.VecDense
	r.MulVec(k, &stateDif)
	r.AddVec(&r, predState)
	f.state = &r

	cov := mat.DenseCopyOf(eye(_N))
	cov.Sub(cov, k)
	cov.Mul(cov, predCov)
	f.cov = cov
	return nil
}

// eye returns an n by n identity matrix.
func eye(n int) mat.Matrix {
	d := make([]float64, n)
	for i := 0; i < n; i++ {
		d[i] = 1.0
	}
	return mat.NewDiagDense(n, d)
}
