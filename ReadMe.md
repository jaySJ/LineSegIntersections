# Description
Given a set of line segments, computes intersections and returns the numnber of interesections found.
Implements two algorithms
	- a simple naive brute force O(n^2) version
	- a sweep line method with theoretical complexity of O((n+k) log n) complexity but whose robustness and performance are dependent on the DSA usd for implementation

See [illustration](https://github.com/jaySJ/LineSegIntersections/blob/main/Line-Segment%20Intersection.pdf) of intersection computations.
## Performance of sweep line
Current implementation of sweep line is slower than the brute force implementation and also not robust (reporting fewer intersections than the brute force).
Example output for file with ~750 of points (with debug flags on):
	`Segment intersections.
	(Naive) Number of intersections = 66374
	Naive algorithm took 0s
	(Sweep Line) Number of intersections = 57646
	Sweep algorithm took 1s`

## Build
Requires Visual Studio with C++20 to build. Use Community Edition 2022.
Open LineSegIntersections.sln and build.

## Usage
Usage: LineSegIntersections.exe <filename> [v]
	Optional argument `v` will print the actual intersections.

Example: LineSegIntersections.exe data/test1.dat
