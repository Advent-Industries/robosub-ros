#include <vector>

#define RHO 1000            // density of water
#define WATER_HEIGHT 0      // the z height of the water
#define QUADRATIC_DRAG 1    // true is drag should be quadratic, rather than linear
#define PI 3.1415926535897932384626433832

using namespace std; 

using namespace std;

float grav_vector[3];
float grav;

// some C definitions to make CoppeliaSim happy
extern "C" __declspec(dllexport) unsigned char simStart(void* reserved,int reservedInt);
extern "C" __declspec(dllexport) void simEnd();
extern "C" __declspec(dllexport) void* simMessage(int message,int* auxiliaryData,void* customData,int* replyData);

// see https://www.coppeliarobotics.com/helpFiles/en/plugins.htm

unsigned char simStart(void* reserved,int reservedInt){ // called at the beginning of simulation
    // need to initialize constants in here
    // also note that here, unlike the rest of c, zero return is error, not not nonzero return
    if (simGetArrayParameter(sim_arrayparam_gravity, grav_vector)) return 0;
    grav = grav_vector[2];

    return 1;
}

int applyBuoyancy(int handle, vector<float> centerOfBuoy) {
    vector<float> buoy(3,0);
    if (calcBuoyancy(handle, buoy.data()) != 0) return -1;
    if (simAddForce(handle, buoy.data(), centerOfBuoy.data()) != 0) return -1;

    return 0;
}

int calcBuoyancy(int handle, vector<float> buoy) {
    // buoyancy = rho * V * g, where V is volume of object underwater
    // calculates V assuming the object fills the entire space of its bounding box
    float minsize[3];
    float maxsize[3];
    float objsize[3];

    for (int i = 0; i < 2; i++) {
        if (simGetObjectFloatParameter(handle, 15+i, minsize+i) != 0) return -1;
        if (simGetObjectFloatParameter(handle, 18+i, maxsize+i) != 0) return -1;
        objsize[i] = maxsize[i] - minsize[i];
    }

    float vol = size[0] * size[1];
    float zdepth; // the amount that the robot is below the water

    // this whole block can probably be condensed into a one liner
    // but I'm leaving it as is for now because I want to logic to be explicit, in the likely event
    // that we have to change it in the near future.
    if (minsize[2] > WATER_HEIGHT) {
        zdepth = 0;
    } else if (maxsize[2] < WATER_HEIGHT) {
        zdepth = size[2];
    } else {
        zdepth = WATER_HEIGHT - minsize[2];
    }

    vol *= zdepth;

    float buoyForce = RHO * vol * grav;

    buoy[0] = 0;
    buoy[1] = 0;
    buoy[2] = buoyForce;

    return 0;
}

vector<float> getLinDrag(float dragCoef, vector<float> linVel, int linVelSize, float diameter, float length){
    float* dragForce = (float*) malloc(3 * sizeof(float));
    vector<float> dragForce(3, 0);

	for (int i = 1; i < 3; i++){
		dragForce[i] = -0.5 * RHO * dragCoef * * linVel[i] * linVel[i];
	}
    dragForce[0] = -0.5 * RHO * dragCoef * PI * diameter * length * 0.5 * (diameter / 2)*(diameter / 2) * pi * linVel[i] * linVel[i];
    return dragForce;
}

void applyLinDrag(vector<float> force, int objectHandle){
    simAddForceAndTorque(objectHandle, force.data(), NULL);
}

vector<float> getPos(int objectHandle){
    int posSize = 3;
    vector<float> pos(posSize,0);
    int relativeToObjectHandle = -1;
    int errorCode = simGetObjectPosition(objectHandle, relativeToObjectHandle, pos.data());
    return pos;
}

vector<float> getLinVelocity(int objectHandle){
	int velSize = 3;
	vector<float> linVel(velSize, 0);
	int errorCode = simGetObjectVelocity(objectHandle, linVel.data(), NULL);
	return linVel;
}

vector<float> getAngVelocity(int objectHandle){
	int velSize = 3;
    vector<float> angVel(velSize,0);
	int errorCode = simGetObjectVelocity(objectHandle, NULL, angVel.data());
	return angVel;
}

vector<float> getAcc(vector<float> prevVel, vector<float> currVel, float timeStep){
    vector<float> accel(velSize, 0);
	for (int i = 0; i < 3; i++){
		accel[i] = (currVel[i] - prevVel[i]) / timeStep;
	}
	return accel;
}

void applyAngDrag(vector<float> torque, int objectHandle){
    simAddForceAndTorque(objectHandle, NULL, torque.data());
}

vector<float> getAngDrag(float dragCoeff, vector<float> angVel, float r, float height){
    float mu = 8.9 * 0.0001;
    vector<float> angDrag(3,0);
    angDrag[0] = 2*PI*mu*r*height*angVel[0];
    angDrag[1] = 2*PI*mu*r*height*angVel[1] + 2*(0.2)*(PI)(dragCoeff)(RHO)(angVel[1]*angVel[1])*(r*r*r*r*r)
                 PI*(angVel[1]*angVel[1])*(r*r*r*r)*(height)*RHO*dragCoeff;
    angDrag[2] = 2*PI*mu*r*height*angVel[2] + 2*(0.2)*(PI)(dragCoeff)(RHO)(angVel[2]*angVel[2])*(r*r*r*r*r)
                 PI*(angVel[2]*angVel[2])*(r*r*r*r)*(height)*RHO*dragCoeff;
    return angDrag;
} 

vector<vector<float> > get_thrusterforces(vector<float> thrusterValues, float thrusterPower){

    float d0[3] = {sqrt(2), sqrt(2), 0};
    float d1[3] = {sprt(2), -1*sqrt(2), 0};
    float d2[3] = {-1*sqrt(2),sqrt(2), 0};
    float d3[3] = {-1*sqrt(2),-1*sqrt(2), 0};
    float d4[3] = {0, 0, -1};
    float d5[3] = {0, 0, -1};
    float d6[3] = {0, 0, -1};
    float d7[3] = {0, 0, -1};

    *(directions) = d0;
    *(directions+1) = d1;
    *(directions+2) = d2;
    *(directions+3) = d3;
    *(directions+4) = d4;
    *(directions+5) = d5;
    *(directions+6) = d6;
    *(directions+7) = d7;

    vector<int> flipped{-1,-1,1,-1,1,-1,-1,1};

    vector<vector<float> > thrusterForces(8);
    for (int i = 0; i < 8; i++){
        thrusterForces[i] = vector<float>(3);
        for (int j = 0; j < 3; j++){
            thrusterForces[i][j] = *(*(directions+i)+j) * thrusterPower * thrusterValues[i];
        }
    }
    return thrusterForces
} 

void apply_thrusterForces(vector<vector<float> > thrusterForces, vector<vector<float> > thrusterPositions, int objectHandle){

    for (int i = 0; i < 8; i++){
        simAddForce(objectHandle, thrusterPositions[i].data(), thrusterForces[i].data());
    }
}
