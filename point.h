#ifndef BARJOKART_POINT_H
#define BARJOKART_POINT_H

class Point {
public:
    int x, y;
    int r, g, b;

    explicit Point(int x = 0, int y = 0, int r = 0, int g = 0, int b = 0) {
        this->x = x;
        this->y = y;
        this->r = r;
        this->g = g;
        this->b = b;
    }

    friend std::ostream &operator<<(std::ostream &os, const Point &point) {
        os << "(" << point.x << ", " << point.y << ", [" << point.r << ", " << point.g << ", " << point.b << "])";
        return os;
    }

    [[nodiscard]] bool isWall() const {
        return r == 0 && g == 0 && b == 0;
    }

    Point operator+(const Point &p) const {
        return Point(x + p.x, y + p.y);
    }

    Point operator-(const Point &p) const {
        return Point(x - p.x, y - p.y);
    }

    Point operator*(const int &i) const {
        return Point(x * i, y * i);
    }

    Point operator/(const int &i) const {
        return Point(x / i, y / i);
    }

    bool operator==(const Point &p) const {
        return x == p.x && y == p.y;
    }

    bool operator!=(const Point &p) const {
        return !(*this == p);
    }

    bool operator<(const Point &p) const {
        return x < p.x && y < p.y;
    }

    bool operator>(const Point &p) const {
        return x > p.x && y > p.y;
    }

    bool operator<=(const Point &p) const {
        return x <= p.x && y <= p.y;
    }

    bool operator>=(const Point &p) const {
        return x >= p.x && y >= p.y;
    }
};

#endif //BARJOKART_POINT_H
