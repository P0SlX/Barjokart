#ifndef BARJOKART_POINT_H
#define BARJOKART_POINT_H

class Point {
public:
    int x, y;

    explicit Point(int a = 0, int b = 0) {
        x = a;
        y = b;
    }

    bool operator==(const Point &o) const { return o.x == x && o.y == y; }

    Point operator+(const Point &o) const { return Point{o.x + x, o.y + y}; }

    Point operator-(const Point &o) const { return Point{x - o.x, y - o.y}; }

    Point operator*(const int &o) const { return Point{x * o, y * o}; }

    Point operator/(const int &o) const { return Point{x / o, y / o}; }

    Point operator%(const int &o) const { return Point{x % o, y % o}; }

    Point operator+=(const Point &o) { return *this = *this + o; }

    Point operator-=(const Point &o) { return *this = *this - o; }

    Point operator*=(const int &o) { return *this = *this * o; }

    Point operator/=(const int &o) { return *this = *this / o; }

    Point operator%=(const int &o) { return *this = *this % o; }

    Point operator-() const { return Point{-x, -y}; }

    int operator[](const int &o) const { return o == 0 ? x : y; }

    int &operator[](const int &o) { return o == 0 ? x : y; }
};

#endif //BARJOKART_POINT_H
