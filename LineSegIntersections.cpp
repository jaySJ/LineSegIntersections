// LineSegmentIntersection.cpp : Defines the entry point for the application.
//

#include <algorithm>
#include <cassert>
#include <fstream>
#include <iostream>
#include <string>
#include <chrono>
#include <queue>

#include "Event.h"

// Returns cross product of directed segments p0p1 and p0p2 to determine
// whether the second segment is clockwise (>0), colinear (==0) or counterclockwise (<0)
// compared to the first segment. Also the area of the paralellogram.
double direction(Pt2D p0, Pt2D p1, Pt2D p2) {
	return (p1.x() - p0.x()) * (p2.y() - p0.y()) - (p2.x() - p0.x()) * (p1.y() - p0.y());
}


double cross_product(Pt2D p1, Pt2D p2) {
	return p1.x() * p2.y() - p2.x() * p1.y();
}


// Returns true if the point pk (which is known to be colinear) actually
// falls on the line segment pi-pj. This means the intersection is the
// point pk. 
// This routine should only be called when it is known that pk is collinear
// with the pi,pj. (cross-product is zero)
bool on_segment(Pt2D pi, Pt2D pj, Pt2D pk) {
	if ((std::min(pi.x(), pj.x()) <= pk.x() &&
		pk.x() <= std::max(pi.x(), pj.x())) &&
		(std::min(pi.y(), pj.y()) <= pk.y() &&
			pk.y() <= std::max(pi.y(), pj.y())))
		return true;
	return false;
}

// Returns true if the input line segments intersect (including when an 
// end/start point of a segment falls on another line)
bool segments_intersect(LineSeg l1, LineSeg l2) {
	auto d1 = direction(l2._start, l2._end, l1._start);
	auto d2 = direction(l2._start, l2._end, l1._end);
	auto d3 = direction(l1._start, l1._end, l2._start);
	auto d4 = direction(l1._start, l1._end, l2._end);

	if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
		((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)))
		return true;
	else if ((d1 == 0 && on_segment(l2._start, l2._end, l1._start)) ||
		(d2 == 0 && on_segment(l2._start, l2._end, l1._end)) ||
		(d3 == 0 && on_segment(l1._start, l1._end, l2._start)) ||
		(d4 == 0 && on_segment(l1._start, l1._end, l2._end)))
		return true;
	else
		return false;
}


// Naive algorithm - O(N^2) time complexity where N is the number of line segments.
void naive_intersections(const std::vector<LineSeg>& seg, bool verbose = false) {
	const size_t size = seg.size();
	int count = 0;
	// This double loop means we are performing N * (N-1) intersection check, hence O(N^2)
	// In practice, the number of intersections k << N.  Use Plane Sweep algorithm instead.
	for (size_t i = 0; i < size; ++i) {
		for (size_t j = i + 1; j < size; ++j) {
			if (segments_intersect(seg[i], seg[j])) {
				if (verbose) {
					std::cout << seg[i].tostr();
					std::cout << seg[j].tostr() << std::endl;
				}
				count++;
			}
		}
	}
	std::cout << "(Naive) Number of intersections = " << count << std::endl;
}

// https://www.cs.cmu.edu/afs/cs/academic/class/15456-s10/Handouts/BKOS-sweep-line.pdf

// The key observation that leads to a good algorithm is that right before
// two segments cross, they must be neighbors in the segment list.
// The key idea of the algorithm is that as our segment list evolves (as we process the interesting x
// coordinates from left to right), we only need to consider the possible intersections between segments
//that are neighbors, at some point in time, in the segment list.
// Situations that need special consideration:
//    3 lines intersect at same point (use perturbation or exact arithmetic)
//    Vertical line segments (slope = inf) => x-coord of end points and intersection are unique
//    End point lies on another segment
#include <algorithm>

Pt2D calculateIntersection(const LineSeg& s1, const LineSeg& s2) {
	// Calculate intersection of two lines
	float x1 = s1._start.x(), y1 = s1._start.y();
	float x2 = s1._end.x(), y2 = s1._end.y();
	float x3 = s2._start.x(), y3 = s2._start.y();
	float x4 = s2._end.x(), y4 = s2._end.y();

	float denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
	if (denom == 0) {
		return Pt2D(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
	}

	float x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom;
	float y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom;
	return Pt2D(x, y);
}

void addIntersections(std::priority_queue<Event, std::vector<Event>, std::greater<Event>>& eventQ,
	const LineSeg& s1, const LineSeg& s2, double currentSweepX) {
	Pt2D intersection = calculateIntersection(s1, s2);
	// Only add to event queue if intersection is to the right of current sweep line
	if (intersection.x() > currentSweepX) {
		Event intersectionEvent(intersection.x(),
			intersection.y(),
			std::nullopt,  // not using slope
			s2._segId,
			true,
			true, // isLeftEndPt = true for intersection
			std::pair<unsigned long, unsigned long>(s1._segId, s2._segId));
		eventQ.push(intersectionEvent);
	}
}

// Function to check intersection and add to event queue if needed
void checkAndAddIntersection(
	const LineSeg& s1,
	const LineSeg& s2,
	std::priority_queue<Event, std::vector<Event>, std::greater<Event>>& eventQ,
	std::set<std::pair<unsigned long, unsigned long>>& intersectingSegments,
	double currentSweepX)
{
	if (segments_intersect(s1, s2)) {
		auto pr = std::make_pair(
			std::min(s1._segId, s2._segId),
			std::max(s1._segId, s2._segId)
		);

		if (intersectingSegments.find(pr) == intersectingSegments.end()) {
			addIntersections(eventQ, s1, s2, currentSweepX);
			intersectingSegments.insert(pr);
		}
	}
}

bool FindIntersections(std::vector<LineSeg> segments, bool verbose = false) {
	std::priority_queue<Event, std::vector<Event>, std::greater<Event>> eventQ;

	// Status is a balanced binary search tree 
	// We will use std::set which uses a BST/red-black tree
	// Insertion and deletion complexity O(log n)
	// finding neighbors is also O(log n)
	std::set<LineSeg> status;

	using SegIdPair = std::pair<unsigned long, unsigned long>;
	std::set<SegIdPair> intersectingSegments;

	int seg_ind = 0;
	// This operation adds 2 events per each segment. That is 2*n operations 
	// So time complexity O(n) where n is the number if segments.
	for (auto const& seg : segments) {
		std::optional<float> slope = std::nullopt;
		if (seg._start.x() != seg._end.x())
			slope = (seg._end.y() - seg._start.y()) / (seg._end.x() - seg._start.x());
		eventQ.push(Event(seg._start.x(), seg._start.y(), slope, seg._segId, true, false, SegIdPair(seg._segId, seg._segId)));
		eventQ.push(Event(seg._end.x(), seg._end.y(), slope, seg._segId, false, false, SegIdPair(seg._segId, seg._segId)));
		++seg_ind;
	}

	// Time complexity of insertion of n end points and k intersections points is O ( (n + k) log n )
	// The log n factor comes from the balanced tree operations using std::set
	// Time complexity of checking intersections and adding to event queue is O(1)
	while (!eventQ.empty())
	{
		auto p = eventQ.top();
		eventQ.pop();
		// Update the current sweep line x-coordinate
		LineSeg::currentSweepX = p._x + 1e-4;

		// As the sweep line progresses from left to right, we add new intersection points to the event queue,
		// swap segments in the status tree (because of intersection), then check their new neighbors for 
		// intersections.
		// 
		if (p._isIntersection) {
			const auto& [s1, s2] = p._intersectingSegments;

			// Swap segments in status tree
			auto it1 = std::find_if(status.begin(), status.end(),
				[&s1](const LineSeg& s) { return s._segId == s1; });
			auto it2 = std::find_if(status.begin(), status.end(),
				[&s2](const LineSeg& s) { return s._segId == s2; });

			if (it1 != status.end() && it2 != status.end()) {
				// Make copies
				auto it_1 = *it1;
				auto it_2 = *it2;

				// Remove
				status.erase(it1);
				status.erase(it2);

				// reinsert them in swapped order
				status.insert(it_2);  // Insert the second segment first
				status.insert(it_1);  // Then insert the first segment

				// Now check for new intersections with their new neighbors
				auto new_it1 = std::find_if(status.begin(), status.end(),
					[&s1](const LineSeg& s) { return s._segId == s1; });
				auto new_it2 = std::find_if(status.begin(), status.end(),
					[&s2](const LineSeg& s) { return s._segId == s2; });

				// Check for intersections with new neighbors
				if (new_it1 != status.begin()) {
					auto below = std::prev(new_it1);
					if (below != status.end()) {
						checkAndAddIntersection(*below, *new_it1, eventQ, intersectingSegments, LineSeg::currentSweepX);
					}
					auto above = std::next(new_it1);
					if (above != status.end())
						checkAndAddIntersection(*above, *new_it1, eventQ, intersectingSegments, LineSeg::currentSweepX);
				}
				if (std::next(new_it2) != status.end()) {
					auto below = std::prev(new_it2);
					if (below != status.end()) {
						checkAndAddIntersection(*below, *new_it2, eventQ, intersectingSegments, LineSeg::currentSweepX);
					}
					auto above = std::next(new_it2);
					if (above != status.end()) {
						checkAndAddIntersection(*above, *new_it2, eventQ, intersectingSegments, LineSeg::currentSweepX);
					}
				}
			}
		}
		else {
			const auto& seg = segments[p._segIndex];
			// If p is the left end point of a segment
			if (p._isLeftEndPt) {
				// Insert segment 
				auto it = status.insert(seg).first;
				// Above(T,s)
				if (it != status.begin()) {
					auto below = std::prev(it);
					checkAndAddIntersection(*below, seg, eventQ, intersectingSegments, LineSeg::currentSweepX);
				}
				// Below (T,s)
				if (std::next(it) != status.end()) {
					auto above = std::next(it);
					checkAndAddIntersection(*above, seg, eventQ, intersectingSegments, LineSeg::currentSweepX);
				}
			}
			else {
				// If p is the right endpoint of segment s
				//		If both Above and Below exist and Above intersects Below
				auto it = std::find_if(status.begin(), status.end(), [&seg](const LineSeg& s) {
					return s._segId == seg._segId;
				});
				if (it != status.begin() && it != status.end() && std::next(it) != status.end()) {
					auto below = std::prev(it);
					auto above = std::next(it);
					checkAndAddIntersection(*below, *above, eventQ, intersectingSegments, LineSeg::currentSweepX);
				}
				if (it != status.end())
					status.erase(it);
			}
		}
	}
	if (verbose) {
		for (const auto& pair : intersectingSegments) {
			std::cout << segments[pair.first].tostr();
			std::cout << segments[pair.second].tostr() << std::endl;
		}
	}
	std::cout << "(Sweep Line) Number of intersections = " << intersectingSegments.size() << std::endl;
	return false;
}

std::vector<LineSeg> convertToSegmentList(std::vector<std::vector<std::vector<float>>> vectorList) {
	auto segId = 0;
	std::vector<LineSeg> segments;
	for (auto s : vectorList) {
		segments.emplace_back(LineSeg(Pt2D(s[0][0], s[0][1]), Pt2D(s[1][0], s[1][1]), segId++));
	}
	return segments;
}

std::vector<LineSeg> readSegments(const std::string& filename) {
	std::ifstream inputFile(filename);
	if (!inputFile.is_open()) {
		std::cerr << "Error opening file" << std::endl;
		return {};
	}
	std::string line;
	std::vector<LineSeg> segments;
	int segId = 0;
	while (std::getline(inputFile, line)) {
		char comma;
		float sx, sy, ex, ey;
		std::stringstream ss(line);

		if (ss >> sx >> comma >> sy >> comma >> ex >> comma >> ey) {
			segments.emplace_back(LineSeg(Pt2D(sx, sy), Pt2D(ex, ey), segId++));
		}
		else {
			std::cerr << "Warning: Skipping invalid line: " << line << std::endl;
		}
	}
	inputFile.close();
	return segments;
}

int main(int argc, char* argv[])
{
	bool verbose = false;
	if (argc < 2) {
		std::cout << "Usage: " << argv[0] << " <filename>" << std::endl;
		std::cout << "Example: " << argv[0] << " data/test1.dat" << std::endl;
		return 1;
	}
	if (argc > 2) {
		std::string v = argv[2];
		if (v.compare("v") == 0)
			verbose = true;
	}
	std::cout << "Segment intersections." << std::endl;

	auto segments = readSegments(argv[1]);
	if (segments.empty()) {
		std::cerr << "Error: No segments read from file or file not found." << std::endl;
		return 1;
	}

	auto naive_start = std::chrono::steady_clock::now();
	naive_intersections(segments, verbose);

	auto naive_end = std::chrono::steady_clock::now();
	auto naive_elapsed = std::chrono::duration_cast<std::chrono::seconds>(naive_end - naive_start);

	std::cout << "Naive algorithm took " << naive_elapsed << std::endl;

	auto sweep_start = std::chrono::steady_clock::now();
	FindIntersections(segments, verbose);
	auto sweep_end = std::chrono::steady_clock::now();
	auto sweep_elapsed = std::chrono::duration_cast<std::chrono::seconds>(sweep_end - sweep_start);

	std::cout << "Sweep algorithm took " << sweep_elapsed << std::endl;

	return 0;
}
