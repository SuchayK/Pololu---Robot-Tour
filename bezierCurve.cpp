#include <vector>
#include <cmath>

struct Point {
    double x, y;
};

class BezierCurve {
    
private:

    std::vector<Point> controlPoints;

    double binomialCoeff(int n, int k) {

        double res = 1;
        if (k > n - k) k = n - k;

        for (int i = 0; i < k; ++i) {

            res *= (n - i);
            res /= (i + 1);

        }
        return res;

    }

    Point evaluateBezier(double t) {

        Point result = {0, 0};
        int n = controlPoints.size() - 1;

        for (int i = 0; i <= n; i++) {

            double coefficient = binomialCoeff(n, i) * std::pow(t, i) * std::pow(1 - t, n - i);
            result.x += coefficient * controlPoints[i].x;
            result.y += coefficient * controlPoints[i].y;
        }

        return result;

    }

public:

    BezierCurve(const std::vector<Point>& points) : controlPoints(points) {}

    std::vector<Point> generateCurvePoints(int numPoints) {

        std::vector<Point> curvePoints;

        for (int i = 0; i <= numPoints; ++i) {

            double t = static_cast<double>(i) / numPoints;
            curvePoints.push_back(evaluateBezier(t));

        }

        return curvePoints;
    
    }

}

class RobotController {

public:

    void performSmoothTurn(Point start, Point end, Point controlPoint1, Point controlPoint2) {

        std::vector<Point> controlPoints = {start, controlPoint1, controlPoint2, end};
        BezierCurve curve(controlPoints);
        std::vector<Point> pathPoints = curve.generateCurvePoints(100);

        for (size_t i = 1; i < pathPoints.size(); ++i) {

            Point current = pathPoints[i];
            Point previous = pathPoints[i - 1];
            double dx = current.x - previous.x;
            double dy = current.y - previous.y;
            double angle = std::atan2(dy, dx);
            double distance = std::sqrt(dx*dx + dy*dy);
            
            moveRobot(distance, angle);

        }

    }

}