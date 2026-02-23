#ifndef PARTICLE_H
#define PARTICLE_H

#include <vector>
#include <cmath>

const float H = 0.15f;
const float HSQ = H * H;
const float MASS = 0.05f;
const float REST_DENISITY = 1000.0f;
const float GAS_CONSTANT = 2000.0f;
const float VISCOSITY = 0.1f;
const float POLY6 = 315.0f / (64.0f * M_PI * pow(H, 9));
const float SPIKY_GRAD = -45.0f / (M_PI * pow(H, 6));
const float VISC_LAP = 45.0f / (M_PI * pow(H, 6));

const float cellSize = H;
const int gridWidth = 35;

const float boxsize = 5.0f;
const float particleRad = 0.05f;
const float gravity = -9.8f;
const float damping = 0.5f;
const float timeStep = 0.020f;

struct Particle
{
    float x, y, z;
    float vx, vy, vz;
    float fx, fy, fz;
    float density;
    float pressure;
    bool active;

    Particle(float px, float py, float pz, float velx = 0.0f, float vely = 0.0f, float velz = 0.0f)
        : x(px), y(py), z(pz), vx(velx), vy(vely), vz(velz),
          fx(0), fy(0), fz(0), density(0), pressure(0), active(false) {}
};

struct SimulationState
{
    std::vector<Particle> particles;
    std::vector<std::vector<int>> grid;

    int particlesSpawned;
    int targgetParticleCount;
    float spawnY;

    int robotParticle;
    int goalParticle;

    std::vector<std::vector<float>> currentPath;
    int pathUpdateCounter;

    bool goalReached;
    bool initialAgitationDone;
    int settlingFrames;

    float elapsedTime;
    bool useRRTStar;
    int runCount;

    SimulationState()
        : particlesSpawned(0),
          targgetParticleCount(1500),
          spawnY(boxsize / 2.0f - 0.5f),
          robotParticle(-1),
          goalParticle(-1),
          pathUpdateCounter(0),
          goalReached(false),
          initialAgitationDone(false),
          settlingFrames(0),
          elapsedTime(0.0f),
          useRRTStar(false),
          runCount(0) {}

    void reset();
};

extern SimulationState simState;

void spawnParticles(SimulationState &simState);
void buildSpatialGrid(SimulationState &simState);
void findNeighbors(SimulationState &simState, size_t particleIdx, std::vector<int> &neighbors);
void computeDensityPressure(SimulationState &simState);
void computeForces(SimulationState &simState);
void updatePhysics(SimulationState &simState);

int gridHash(int x, int y, int z);
int gridIndex(float pos);

#endif