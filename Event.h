#include "LineSeg.h"
#include <set>
#include <vector>
#include <optional>

struct Event {
	float _x, _y;
	bool _isLeftEndPt=false;
	unsigned long _segIndex;
	std::optional<float> _slope = std::nullopt;
	bool _isIntersection = false;
	std::pair<unsigned long, unsigned long> _intersectingSegments;

	Event(float x, 
		float y, 
		std::optional<float> slope, 
		unsigned long ind, 
		bool leftOf, 
		bool isIntersection,
		std::pair<unsigned long, unsigned long> intersectingSegments) : 
		_x(x), _y(y), _slope(slope), _segIndex(ind), _isLeftEndPt(leftOf), 
		_isIntersection(isIntersection), _intersectingSegments(intersectingSegments) {
	}

	bool operator< (const Event& other) const {
		// proceed from left to right
		// 
		if (_x != other._x) return _x < other._x;

		if (_isLeftEndPt != other._isLeftEndPt) return _isLeftEndPt;

		// if covertical return if y is lower than other's y
		return (_y < other._y) ||
			((_y == other._y) && _x < other._x);
	}
	// Add this operator for priority queue comparison
	bool operator>(const Event& other) const {
		if (std::abs(_x - other._x) < 1e-10) {
			return _y > other._y;
		}
		return _x > other._x;
	}
};
