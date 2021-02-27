#ifndef PHYSICS_LIBRARY_H
#define PHYSICS_LIBRARY_H

#include <vector>
using namespace std;

int calcBuoyancy(int handle, float *buoy);
int applyBuoyancy(int handle, vector<float> centerOfBuoy);
vector<float> getLinDrag(float dragCoef, vector<float> linVel, float diameter, float length);
void applyLinDrag(vector<float> force, int objectHandle);
vector<float> getPos(int objectHandle);
vector<float> getLinVelocity(int objectHandle);
vector<float> getAngVelocity(int objectHandle);
vector<float> getAcc(vector<float> prevVel, vector<float> currVel, float timeStep);
void applyAngDrag(vector<float> torque, int objectHandle);
vector<float> getAngDrag(float dragCoeff, vector<float> angVel, float r, float height);
vector<vector<float>> get_thrusterForces(vector<float> thrusterValues, float thrusterPower);
void apply_thrusterForces(vector<vector<float>> thrusterForces, vector<vector<float>> thrusterPositions, int objectHandle);

#endif