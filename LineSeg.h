#include <map>
#include <climits>
#include <string>
#include <sstream>


struct Pt2D {
private:
	float _x = 0.0;
	float _y = 0.0;

public:
	Pt2D(float x, float y) : _x(x), _y(y) { }

	// delete default c'tor 
	Pt2D() = delete;

	float x() const { return _x; }

	float y() const { return _y; }

	bool operator<(const Pt2D& other) const {
		return _x < other._x || (_x == other._x && _y < other._y);
	}

	bool operator==(const Pt2D& other) const {
		return abs(_x - other._x) < 1e-6 and abs(_y - other._y) < 1e-6;
	}

};

//static const Pt2D operator-(const Pt2D& p, const Pt2D& q) {
//	return Pt2D(p.x() - q.x(), p.y() - q.y());
//}

struct LineSeg {
	Pt2D _start;
	Pt2D _end;
	long _segId;

public:
	LineSeg() = delete;

	LineSeg(const Pt2D& start, const Pt2D& end, long segId) :
		_start(start),
		_end(end),
		_segId(segId) {
		if (_start.x() > _end.x())
			std::swap(_start, _end);
	}

	std::string tostr() const {
		std::stringstream ss;
		ss << "((" << _start.x() << ", " << _start.y() << "), (" << _end.x() << ", " << _end.y() << ")) \t";
		return ss.str();
	}

	// ordering by y-coordinate at current sweep line x
	//bool operator<(const LineSeg& other) const {
	//	double thisY = getYAtX(currentSweepX);
	//	double otherY = other.getYAtX(currentSweepX);

	//	if (thisY != otherY) return thisY < otherY;
	//	return _segId < other._segId; // break ties with segment ID
	//}

	bool operator==(const LineSeg& other) const {
		return _segId == other._segId;
	}

	double getYAtX(double x) const {
		// Linear interpolation to find y at given x
		if (_start.x() == _end.x()) return _start.y(); // vertical line
		return _start.y() + (_end.y() - _start.y()) * (x - _start.x()) / (_end.x() - _start.x());
	}
};

// line segment comparer object for the sweep line
struct SegmentSweepLineComparer {
	mutable double currentSweepX;
	bool operator()(const LineSeg& a, const LineSeg& b) const {
		double ya = a.getYAtX(currentSweepX);
		double yb = b.getYAtX(currentSweepX);

		if (ya != yb) {
			return ya < yb;
		}
		return a._segId < b._segId;  // Tie breaker
	}
};
