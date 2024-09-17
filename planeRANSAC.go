//Maaz Zaidi
//Student Number: 300246507
//Date: March 11, 2023

package main

import (
	"bufio"
	"fmt"
	"log"
	"math"
	"math/rand"
	"os"
	"strconv"
	"strings"
	"sync"
	"time"
)

type Point3D struct {
	X float64
	Y float64
	Z float64
}

type Plane3D struct {
	A float64
	B float64
	C float64
	D float64
}

type Plane3DwSupport struct {
	Plane3D
	SupportSize int
}

// reads an XYZ file and returns a slice of Point3D
func ReadXYZ(filename string) ([]Point3D, error) {

	// read the whole content of file and pass it to file variable, in case of error pass it to err variable
	file, err := os.Open(filename)

	if err != nil {
		fmt.Printf("Could not read the file due to this %s error \n", err)
	}

	fileScanner := bufio.NewScanner(file)
	fileScanner.Split(bufio.ScanLines)

	var points []Point3D

	for fileScanner.Scan() {
		line := fileScanner.Text()
		strPoints := strings.Fields(line)

		x, err := strconv.ParseFloat(strPoints[0], 64)
		//check if error occured
		if err != nil {
			continue
		}

		y, err := strconv.ParseFloat(strPoints[1], 64)
		//check if error occured
		if err != nil {
			continue
		}

		z, err := strconv.ParseFloat(strPoints[2], 64)
		//check if error occured
		if err != nil {
			continue
		}

		points = append(points, Point3D{X: x, Y: y, Z: z})
	}

	if err = file.Close(); err != nil {
		fmt.Printf("could not close the file due to %s", err)
	}

	return points, nil
}

func SaveXYZ(filename string, points []Point3D) {

	//creates initial file
	file, err := os.Create(filename)

	//error
	if err != nil {
		fmt.Println(err)
	}
	defer file.Close()

	wr := bufio.NewWriter(file)
	wr.WriteString("x y z \n")

	//loops for each line using sprint
	for _, p := range points {
		ln := fmt.Sprintf("%f %f %f\n", p.X, p.Y, p.Z)
		_, err := wr.WriteString(ln)
		if err != nil {
			fmt.Println(err)
		}
	}

	//finishes last error
	err = wr.Flush()
	if err != nil {
		fmt.Println(err)
	}
}

// computes the distance between points p1 and p2
func (p1 *Point3D) GetDistance(p2 *Point3D) float64 {
	return math.Sqrt(math.Pow(math.Abs(p2.X-p1.X), 2) + math.Pow(math.Abs(p2.Y-p1.Y), 2) + math.Pow(math.Abs(p2.Z-p1.Z), 2))
}

// computes the plane defined by a set of 3 points
func GetPlane(points [3]Point3D) Plane3D {

	//creates vectors to use
	var plane Plane3D = Plane3D{0, 0, 0, 0}

	var vector1 Point3D = Point3D{points[1].X - points[0].X, points[1].Y - points[0].Y, points[1].Z - points[0].Z}
	var vector2 Point3D = Point3D{points[2].X - points[0].X, points[2].Y - points[0].Y, points[2].Z - points[0].Z}

	// debugPrintPoint(vector1)
	// debugPrintPoint(vector2)

	plane.A = (vector1.Y * vector2.Z) - (vector2.Y * vector1.Z)
	plane.B = (vector1.X * vector2.Z) - (vector2.X * vector1.Z)
	plane.C = (vector1.X * vector2.Y) - (vector2.X * vector1.Y)
	plane.D = (plane.A * points[0].X) + (plane.B * (0 - points[0].Y)) + (plane.C * (0 - points[0].Z))

	return plane
}

// computes the number of required RANSAC iterations
func GetNumberOfIterations(confidence float64, percentageOfPOintsOnPlane float64) int {
	return int(((math.Log10(1 - confidence)) / math.Log10(1-math.Pow(percentageOfPOintsOnPlane, 3))))
}

// computes the support of a plane in a set of points
func GetSupport(plane Plane3D, points []Point3D, eps float64) Plane3DwSupport {
	var support int = 0
	for _, p := range points {
		distance := math.Abs((plane.A*p.X)+(plane.B*p.Y)+(plane.C*p.Z)+plane.D) / math.Sqrt(math.Pow(plane.A, 2)+math.Pow(plane.B, 2)+math.Pow(plane.C, 2))
		if distance < eps {
			support++
		}
	}

	return Plane3DwSupport{plane, support}
}

// extracts the points that supports the given plane
// and returns them as a slice of points
func GetSupportingPoints(plane Plane3D, points []Point3D, eps float64) []Point3D {
	var supporting_points []Point3D
	for _, p := range points {
		distance := (math.Abs((plane.A * p.X) + (plane.B * p.Y) + (plane.C * p.Z) + plane.D)) / (math.Sqrt(math.Pow(plane.A, 2) + math.Pow(plane.B, 2) + math.Pow(plane.C, 2)))
		if distance < eps {
			supporting_points = append(supporting_points, p)
		}
	}

	return supporting_points
}

// creates a new slice of points in which all points
// belonging to the plane have been removed
func RemovePlane(plane Plane3D, points []Point3D, eps float64) []Point3D {
	var remaining_points []Point3D
	for _, p := range points {
		distance := math.Abs((plane.A*p.X)+(plane.B*p.Y)+(plane.C*p.Z)+plane.D) / math.Sqrt(math.Pow(plane.A, 2)+math.Pow(plane.B, 2)+math.Pow(plane.C, 2))
		if distance >= eps {
			remaining_points = append(remaining_points, p)
		}
	}

	return remaining_points
}

// It reads Point3D instances from its input channel and accumulate 3 points. Its output channel transmits arrays of Point3D (composed of three points).
func RandomPointGen(points []Point3D) <-chan Point3D {
	//runs until outputs are working and taken (3)
	out := make(chan Point3D, 3)
	go func() {
		defer close(out)
		for {
			out <- points[rand.Intn(len(points))]
		}
	}()
	return out
}

// It reads arrays of Point3D and resend them. It automatically stops the pipeline after having received N arrays.
func TripletPointsGen(in <-chan Point3D) <-chan [3]Point3D {
	out := make(chan [3]Point3D)
	go func() {
		defer close(out)
		var points [3]Point3D = [3]Point3D{}
		var i int = 0
		for n := range in {
			points[i] = n
			i++

			//compiles each point
			if i == 3 {
				out <- points
				points = [3]Point3D{}
				i = 0
			}
		}
	}()
	return out
}

// reads in all the inputs that delved with []point3d
func TakeN(x int, in <-chan [3]Point3D) <-chan [3]Point3D {
	//takes in controller input to creat channel
	out := make(chan [3]Point3D, x)
	go func() {
		defer close(out)
		for n := range in {
			out <- n
		}
	}()
	return out
}

// It reads arrays of three Point3D and compute the plane defined by these points. Its output channel transmits Plane3D instances describing the computed plane parameters.
func PlaneEstimator(in ...<-chan [3]Point3D) <-chan Plane3D {
	out := make(chan Plane3D, 1)
	go func() {
		defer close(out)
		for _, i := range in {
			for n := range i {
				out <- GetPlane(n)
			}
		}
	}()
	return out
}

// It counts the number of points in the provided slice of Point3D (the input point cloud) that supports
// the received 3D plane. Its output channel transmits the plane parameters and the number of supporting
// points in a Point3DwSupport instance
func SupportingPointFinder(x int, pc []Point3D, eps float64, in <-chan Plane3D) <-chan Plane3DwSupport {
	out := make(chan Plane3DwSupport, x)
	go func() {
		defer close(out)
		for i := 0; i < x; i++ {
			out <- GetSupport(<-in, pc, eps)
		}

	}()
	return out
}

// It multiplexes the results received from multiple channels into one output channel.
func FanIn(in []<-chan Plane3DwSupport) <-chan Plane3DwSupport {
	var wg sync.WaitGroup
	out := make(chan Plane3DwSupport)

	// Start an output goroutine for each input channel in output
	// copies values from c to out until c is closed, then calls wg.Done.
	output := func(c <-chan Plane3DwSupport) {
		for n := range c {
			out <- n
		}
		wg.Done()
	}
	wg.Add(len(in))
	for _, c := range in {
		go output(c)
	}

	// Start a goroutine to close out once all the output goroutines are
	// done.  This must start after the wg.Add call.
	go func() {
		wg.Wait()
		close(out)
	}()
	return out
}

// It receives Plane3DwSupport instances and keepd in memory the plane with the best support
// received so far. This component does not output values, it simply maintains the provided
// *Plane3DwSupport variable.
func DominantPlaneIdentifier(in <-chan Plane3DwSupport, out *Plane3DwSupport) {
	//loops through channel inputs
	for n := range in {
		if n.SupportSize > out.SupportSize {
			out.Plane3D = n.Plane3D
			out.SupportSize = n.SupportSize
		}
	}
}

func main() {
	//reads arguments
	arguments := os.Args[1:]

	points, err := ReadXYZ(arguments[0])

	if err != nil {
		fmt.Println(err)
	}

	//starts time
	start := time.Now()

	//converts variables
	var bestSupport Plane3DwSupport = Plane3DwSupport{Plane3D{0, 0, 0, 0}, 0}

	confidence, err := strconv.ParseFloat(arguments[1], 64)
	if err != nil {
		fmt.Println(err)
	}
	percentage, err := strconv.ParseFloat(arguments[2], 64)
	if err != nil {
		fmt.Println(err)
	}
	eps, err := strconv.ParseFloat(arguments[3], 64)
	if err != nil {
		fmt.Println(err)
	}

	//starts pipeline
	iterations := GetNumberOfIterations(confidence, percentage)
	c1 := RandomPointGen(points)
	c2 := TripletPointsGen(c1)

	//pipeline ends here
	c3 := TakeN(3, c2)
	c4 := PlaneEstimator(c3)

	//value to control threads
	x := 6

	c5 := make([]<-chan Plane3DwSupport, iterations/x)

	//creates multiple threads
	for i := 0; i < iterations/x; i++ {
		c5[i] = SupportingPointFinder(x, points, eps, c4)
	}

	c6 := FanIn(c5)
	DominantPlaneIdentifier(c6, &bestSupport)

	//saves the multiple output files
	result := GetSupportingPoints(bestSupport.Plane3D, points, eps)
	SaveXYZ(arguments[0][:len(arguments[0])-4]+"_p.xyz", result)

	p0 := RemovePlane(bestSupport.Plane3D, points, eps)
	SaveXYZ(arguments[0][:len(arguments[0])-4]+"_p0.xyz", p0)

	//ends timer and logs
	log.Printf("RANSAC runtime: %s", time.Since(start))
}
