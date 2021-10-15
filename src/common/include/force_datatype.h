#ifndef FORCE_DATATYPE_H
#define FORCE_DATATYPE_H

#include <string>
#include <math.h>
#include "ur_datatype.h"

namespace force_data_type
{

class ForceValue
{
public:
    double fx;
    double fy;
    double fz; 
    
    ForceValue()
    {
        fx = 0.0;
        fy = 0.0;
        fz = 0.0;
    }

    ForceValue(double fx, double fy, double fz)
    {
        this->fx = fx;
        this->fy = fy;
        this->fz = fz;
    }

    void transDoubleList(double force[3])
    {
        force[0] = fx;
        force[1] = fy;
        force[2] = fz;       
    }

    ForceValue operator*(const double scale)
    {
        ForceValue force;
        force.fx = this->fx * scale;
        force.fy = this->fy * scale;
        force.fz = this->fz * scale;    
        return force;
    }

    ForceValue operator+=(const ForceValue force_value)
    {
        this->fx += force_value.fx;
        this->fy += force_value.fy;
        this->fz += force_value.fz;
        return *this;
    }

    ForceValue operator-=(const ForceValue force_value)
    {
        this->fx -= force_value.fx;
        this->fy -= force_value.fy;
        this->fz -= force_value.fz;
        return *this;
    }

    ForceValue operator+(const ForceValue force_value)
    {
        ForceValue force;
        force.fx = this->fx + force_value.fx;
        force.fy = this->fy + force_value.fy;
        force.fz = this->fz + force_value.fz;
        return force;
    }

    ForceValue operator-(const ForceValue force_value)
    {
        ForceValue force;
        force.fx = this->fx - force_value.fx;
        force.fy = this->fy - force_value.fy;
        force.fz = this->fz - force_value.fz;
        return force;
    }

    double getForceAbs()
    {
        return sqrt(pow(fx, 2) + pow(fy, 2) + pow(fz, 2));
    }

    void printValue(std::string comment)
    {
        printf("%s: %lf, %lf, %lf\n", comment.c_str(), fx, fy, fz);
    }

};

class TorqueValue
{
public:
    double tx;
    double ty;
    double tz;  
    
    TorqueValue()
    {
        tx = 0.0;
        ty = 0.0;
        tz = 0.0;
    }

    TorqueValue operator*(const double scale)
    {
        TorqueValue torque;
        torque.tx = this->tx * scale;
        torque.ty = this->ty * scale;
        torque.tz = this->tz * scale;    
        return torque;
    }

    TorqueValue operator+(const TorqueValue torque_value)
    {
        TorqueValue torque;
        torque.tx = this->tx + torque_value.tx;
        torque.ty = this->ty + torque_value.ty;
        torque.tz = this->tz + torque_value.tz;
        return torque;
    }

    TorqueValue operator-(const TorqueValue torque_value)
    {
        TorqueValue torque;
        torque.tx = this->tx - torque_value.tx;
        torque.ty = this->ty - torque_value.ty;
        torque.tz = this->tz - torque_value.tz;
        return torque;
    }

    TorqueValue operator+=(const TorqueValue torque_value)
    {
        this->tx += torque_value.tx;
        this->ty += torque_value.ty;
        this->tz += torque_value.tz;
        return *this;
    }

    TorqueValue operator-=(const TorqueValue torque_value)
    {
        this->tx -= torque_value.tx;
        this->ty -= torque_value.ty;
        this->tz -= torque_value.tz;
        return *this;
    }

    double getTorqueAbs()
    {
        return sqrt(pow(tx, 2) + pow(ty, 2) + pow(tz, 2));
    }

    void printValue(std::string comment)
    {
        printf("%s: %lf, %lf, %lf\n", comment.c_str(), tx, ty, tz);
    }
};

class FTSensorData
{
public: 
    ForceValue force;
    TorqueValue torque;

    FTSensorData()
    {
        force = ForceValue();
        torque = TorqueValue();
    }

    FTSensorData operator+=(const FTSensorData sensor_data)
    {
        this->force += sensor_data.force;
        this->torque += sensor_data.torque;          
        return *this;
    }

    FTSensorData operator-=(const FTSensorData sensor_data)
    {
        this->force -= sensor_data.force;
        this->torque -= sensor_data.torque;          
        return *this;
    }

    FTSensorData operator*(const double scale)
    {
        FTSensorData data;
        data.force.fx = this->force.fx * scale;
        data.force.fy = this->force.fy * scale;
        data.force.fz = this->force.fz * scale;    
        data.torque.tx = this->torque.tx * scale;
        data.torque.ty = this->torque.ty * scale;
        data.torque.tz = this->torque.tz * scale;     
        return data;
    }

    void printValue(std::string comment)
    {
        printf("%s: %lf, %lf, %lf, %lf, %lf, %lf\n", comment.c_str(), 
            force.fx, 
            force.fy, 
            force.fz,
            torque.tx,
            torque.ty,
            torque.tz
            );
    }
};

struct MoveBoundary
{
    double radium_min;
    double error_range;
    ur_data_type::CartesianPoint boundary_min;
    ur_data_type::CartesianPoint boundary_max;
};



struct AssistForceInfo
{
    double assist_force; 
    double random_force;
    double force_rigidity[2];
    double force_edge[4];
};

struct PidFactors
{
    double kp;
    double ki;
    double kd;
};

}

#endif