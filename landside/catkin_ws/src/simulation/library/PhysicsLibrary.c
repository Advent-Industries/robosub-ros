#ifndef PHYSICS_LIBRARY_C
#define PHYSICS_LIBRARY_C

#include <vector>
#include <cstdio>
#include <math.h>
#include "stack/stackArray.h"
#include "stack/stackMap.h"
#include "simLib.h"
#include "PhysicsLibrary.h"

#define RHO 1000         // density of water
#define WATER_HEIGHT 0   // the z height of the water
#define QUADRATIC_DRAG 1 // true is drag should be quadratic, rather than linear
#define PI 3.1415926535897932384626433832

using namespace std;

int calcBuoyancy(int handle, float *buoy)
{
    // buoyancy = rho * V * g, where V is volume of object underwater
    // calculates V assuming the object fills the entire space of its bounding box
    float minsize[3];
    float maxsize[3];
    float objsize[3];

    for (int i = 0; i < 3; i++)
    {
        if (simGetObjectFloatParameter(handle, 15 + i, minsize + i) != 0)
            ; // return -1;
        if (simGetObjectFloatParameter(handle, 18 + i, maxsize + i) != 0)
            ; // return -1;
        printf("min %f, max %f\n", minsize[i], maxsize[i]);
        objsize[i] = maxsize[i] - minsize[i];
    }

    float vol = objsize[0] * objsize[1];
    float zdepth; // the amount that the robot is below the water
    float pos[3];

    simGetObjectPosition(handle, -1, pos);

    if (objsize[2] <= WATER_HEIGHT - pos[2])
    {
        zdepth = objsize[2];
    }
    else
    {
        zdepth = WATER_HEIGHT - pos[2];
    }

    vol *= zdepth;

    float grav_vector[3];
    simGetArrayParameter(sim_arrayparam_gravity, grav_vector);
    float grav = grav_vector[2];

    printf("rho %d, vol %f, grav %f\n", RHO, vol, grav);

    float buoyForce = RHO * vol * grav * -1; // act opposite gravity

    buoy[0] = 0;
    buoy[1] = 0;
    buoy[2] = buoyForce;

    return 0;
}

int applyBuoyancy(int handle, vector<float> centerOfBuoy)
{
    vector<float> buoy(3, 0);
    if (calcBuoyancy(handle, buoy.data()) != 0)
        printf("error calculating");
    if (simAddForce(handle, buoy.data(), centerOfBuoy.data()) != 0)
        printf("error adding force");
    printf("buoydata: %f %f %f", buoy.data()[0], buoy.data()[1], buoy.data()[2]);
    return 0;
}

vector<float> getLinDrag(float dragCoef, vector<float> linVel, float diameter, float length)
{
    vector<float> dragForce(3, 0);

    for (int i = 1; i < 3; i++)
    {
        dragForce[i] = -0.5 * RHO * dragCoef * linVel[i] * linVel[i];
    }
    dragForce[0] = -0.5 * RHO * dragCoef * PI * diameter * length * 0.5 * (diameter / 2) * (diameter / 2) * PI * linVel[0] * linVel[0];
    return dragForce;
}

void applyLinDrag(vector<float> force, int objectHandle)
{
    simAddForceAndTorque(objectHandle, force.data(), NULL);
}

vector<float> getPos(int objectHandle)
{
    int posSize = 3;
    vector<float> pos(posSize, 0);
    int relativeToObjectHandle = -1;
    int errorCode = simGetObjectPosition(objectHandle, relativeToObjectHandle, pos.data());
    return pos;
}

vector<float> getLinVelocity(int objectHandle)
{
    int velSize = 3;
    vector<float> linVel(velSize, 0);
    int errorCode = simGetObjectVelocity(objectHandle, linVel.data(), NULL);
    return linVel;
}

vector<float> getAngVelocity(int objectHandle)
{
    int velSize = 3;
    vector<float> angVel(velSize, 0);
    int errorCode = simGetObjectVelocity(objectHandle, NULL, angVel.data());
    return angVel;
}

vector<float> getAcc(vector<float> prevVel, vector<float> currVel, float timeStep)
{
    int velSize = 3;
    vector<float> accel(velSize, 0);
    for (int i = 0; i < 3; i++)
    {
        accel[i] = (currVel[i] - prevVel[i]) / timeStep;
    }
    return accel;
}

void applyAngDrag(vector<float> torque, int objectHandle)
{
    simAddForceAndTorque(objectHandle, NULL, torque.data());
}

vector<float> getAngDrag(float dragCoeff, vector<float> angVel, float r, float height)
{
    float mu = 8.9 * 0.0001;
    vector<float> angDrag(3, 0);
    angDrag[0] = 2 * PI * mu * r * height * angVel[0];
    angDrag[1] = 2 * PI * mu * r * height * angVel[1] + 2 * (0.2) * (PI) * (dragCoeff) * (RHO) * (angVel[1] * angVel[1]) * (r * r * r * r * r) * PI * (angVel[1] * angVel[1]) * (r * r * r * r) * (height)*RHO * dragCoeff;
    angDrag[2] = 2 * PI * mu * r * height * angVel[2] + 2 * (0.2) * (PI) * (dragCoeff) * (RHO) * (angVel[2] * angVel[2]) * (r * r * r * r * r) * PI * (angVel[2] * angVel[2]) * (r * r * r * r) * (height)*RHO * dragCoeff;
    return angDrag;
}

vector<vector<float>> get_thrusterForces(vector<float> thrusterValues, float thrusterPower)
{

    float d0[3] = {sqrt(2), sqrt(2), 0};
    float d1[3] = {sqrt(2), -1 * sqrt(2), 0};
    float d2[3] = {-1 * sqrt(2), sqrt(2), 0};
    float d3[3] = {-1 * sqrt(2), -1 * sqrt(2), 0};
    float d4[3] = {0, 0, -1};
    float d5[3] = {0, 0, -1};
    float d6[3] = {0, 0, -1};
    float d7[3] = {0, 0, -1};

    float *directions[8];

    *(directions) = d0;
    *(directions + 1) = d1;
    *(directions + 2) = d2;
    *(directions + 3) = d3;
    *(directions + 4) = d4;
    *(directions + 5) = d5;
    *(directions + 6) = d6;
    *(directions + 7) = d7;

    vector<int> flipped{-1, -1, 1, -1, 1, -1, -1, 1};

    vector<vector<float>> thrusterForces(8);
    for (int i = 0; i < 8; i++)
    {
        thrusterForces[i] = vector<float>(3);
        for (int j = 0; j < 3; j++)
        {
            thrusterForces[i][j] = *(*(directions + i) + j) * thrusterPower * thrusterValues[i];
        }
    }
    return thrusterForces;
}

void apply_thrusterForces(vector<vector<float>> thrusterForces, vector<vector<float>> thrusterPositions, int objectHandle)
{
    for (int i = 0; i < 8; i++)
    {
        simAddForce(objectHandle, thrusterPositions[i].data(), thrusterForces[i].data());
    }
}

#endif
