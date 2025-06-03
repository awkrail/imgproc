struct Point {
  int x;
  int y;
};

struct FloatPoint {
  float x;
  float y;
};

float polar_angle(const Point& p, const Point& p0);

float cross(const Point& o, const Point& a, const Point& b);

std::vector<Point> graham_scan(std::vector<Point> & points);
