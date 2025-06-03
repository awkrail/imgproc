#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

struct Point {
  int x;
  int y;
};

float polar_angle(const Point& p, const Point& p0) {
  int dx = p.x - p0.x;
  int dy = p.y - p0.y;
  return std::atan2(dy, dx);
}

float cross(const Point& o, const Point& a, const Point& b) {
  return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

std::vector<Point> graham_scan(std::vector<Point> & points) {
  if (points.size() == 1) {
    return points;
  }

  // get the starting point (x_min, y_min)
  std::sort(points.begin(), points.end(), [](const Point& a, const Point& b) {
    return (a.y < b.y) || (a.y == b.y && a.x < b.x);
  });
  
  // sort the array based on atan2 - counter clockwise order
  Point start = points[0];
  std::sort(points.begin() + 1, points.end(), [start](const Point& a, const Point& b) {
    return (polar_angle(a, start) < polar_angle(b, start));
  });

  // grapham scan algorithm
  std::vector<Point> hull = { points[0], points[1] };
  for (int i = 2; i < points.size(); i++) {
    int last_ind = hull.size() - 1;
    int last_2nd = hull.size() - 2;

    while (hull.size() >= 2 && cross(hull[last_2nd], hull[last_ind], points[i]) <= 0) {
      hull.pop_back();
      last_ind = hull.size() - 1;
      last_2nd = hull.size() - 2;
    }
    hull.push_back(points[i]);
  }

  return hull;
}
