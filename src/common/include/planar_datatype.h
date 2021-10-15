#ifndef PLANAR_DATATYPE_H
#define PLANAR_DATATYPE_H

#include <string>
#include <math.h>

namespace planar_data_type
{
struct PlanarPoint
{
    double x;
    double y;

    PlanarPoint(double x = 0.0, double y = 0.0)
    {
        this->x = x;
        this->y = y;
    }
};

struct PlanarForceValue
{
    double fx;
    double fy;

    PlanarForceValue()
    {
        fx = 0.0;
        fy = 0.0;
    }

    PlanarForceValue operator*=(const double scale_factor)
    {
        this->fx *= scale_factor;
        this->fy *= scale_factor;
        return *this;
    }

    PlanarForceValue operator+=(PlanarForceValue force)
    {
        this->fx += force.fx;
        this->fy += force.fy;
        return *this;
    }

    PlanarForceValue operator*(double scale)
    {
        PlanarForceValue force_value;
        force_value.fx = this->fx + scale;
        force_value.fy = this->fy + scale;
        return force_value;
    }

    PlanarForceValue operator+(PlanarForceValue force)
    {
        PlanarForceValue force_value;
        force_value.fx = this->fx + force.fx;
        force_value.fy = this->fy + force.fy;
        return force_value;
    }
};

enum MoveDirectionInCircle{
    CLOCKWISE,
    ANTICLOCKWISE
};

enum CuttingDirection
{
    POSITIVE,
    NEGATIVE
};

class Line
{
public:
	float a0, a1, a2; // A,B,C, a1 = k, a2 = b
	Line(PlanarPoint start, PlanarPoint end)
    {
        if (fabs(end.x - start.x) < 0.000001)
        {
            a1 = 10000.0;
        }
        else
        {
            a1 = (end.y - start.y) / (end.x - start.x);
        }

        a2 = start.y - a1 * start.x;
        a0 = -1;
    }
};

}
#endif
