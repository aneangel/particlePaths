#include "pathFinding.h"
#include <queue>

int pathGridDim = 25;
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <cstdlib>
#include <limits>
#include <chrono>

static float computePathLength(const std::vector<std::vector<float>> &path)
{
    float len = 0.0f;
    for (size_t i = 1; i < path.size(); i++)
    {
        float dx = path[i][0] - path[i - 1][0];
        float dy = path[i][1] - path[i - 1][1];
        float dz = path[i][2] - path[i - 1][2];
        len += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    return len;
}

static float computePathDivergence(
    const std::vector<std::vector<float>> &oldPath,
    const std::vector<std::vector<float>> &newPath)
{
    if (oldPath.empty() || newPath.empty())
        return 0.0f;
    float total = 0.0f;
    for (const auto &wp : oldPath)
    {
        float minD = std::numeric_limits<float>::max();
        for (const auto &nwp : newPath)
        {
            float dx = wp[0] - nwp[0];
            float dy = wp[1] - nwp[1];
            float dz = wp[2] - nwp[2];
            float d = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (d < minD)
                minD = d;
        }
        total += minD;
    }
    return total / (float)oldPath.size();
}

void worldToPathgrid(float x, float y, float z, int &gx, int &gy, int &gz)
{
    float halfSize = boxsize / 2.0f;
    gx = (int)((x + halfSize) / pathCellSize);
    gy = (int)((y + halfSize) / pathCellSize);
    gz = (int)((z + halfSize) / pathCellSize);
}

void pathGridToWorld(int gx, int gy, int gz, float &x, float &y, float &z)
{
    float halfSize = boxsize / 2.0f;
    x = -halfSize + (gx + 0.5f) * pathCellSize;
    y = -halfSize + (gy + 0.5f) * pathCellSize;
    z = -halfSize + (gz + 0.5f) * pathCellSize;
}

bool isOccupied(const SimulationState &simState, int gx, int gy, int gz)
{
    float wx, wy, wz;
    pathGridToWorld(gx, gy, gz, wx, wy, wz);

    for (const auto &obs : simState.obstacles)
    {
        if (wx >= obs.cx - obs.hx - pathCellSize && wx <= obs.cx + obs.hx + pathCellSize &&
            wy >= obs.cy - obs.hy - pathCellSize && wy <= obs.cy + obs.hy + pathCellSize &&
            wz >= obs.cz - obs.hz - pathCellSize && wz <= obs.cz + obs.hz + pathCellSize)
            return true;
    }

    float checkRadius = pathCellSize * 0.7f;

    for (size_t i = 0; i < simState.particles.size(); i++)
    {
        if (!simState.particles[i].active)
            continue;
        if ((int)i == simState.robotParticle || (int)i == simState.goalParticle)
            continue;

        float dx = simState.particles[i].x - wx;
        float dy = simState.particles[i].y - wy;
        float dz = simState.particles[i].z - wz;
        float distSq = dx * dx + dy * dy + dz * dz;

        if (distSq < checkRadius * checkRadius)
        {
            return true;
        }
    }

    return false;
}

bool isOccupiedWorld(const SimulationState &simState, float wx, float wy, float wz)
{
    float halfSize = boxsize / 2.0f;
    if (wx < -halfSize || wx > halfSize ||
        wy < -halfSize || wy > halfSize ||
        wz < -halfSize || wz > halfSize)
    {
        return true;
    }

    for (const auto &obs : simState.obstacles)
    {
        if (obs.contains(wx, wy, wz))
            return true;
    }

    float checkRadius = particleRad * 2.0f;

    for (size_t i = 0; i < simState.particles.size(); i++)
    {
        if (!simState.particles[i].active)
            continue;
        if ((int)i == simState.robotParticle || (int)i == simState.goalParticle)
            continue;

        float dx = simState.particles[i].x - wx;
        float dy = simState.particles[i].y - wy;
        float dz = simState.particles[i].z - wz;
        float distSq = dx * dx + dy * dy + dz * dz;

        if (distSq < checkRadius * checkRadius)
        {
            return true;
        }
    }

    return false;
}

float heuristic(int x1, int y1, int z1, int x2, int y2, int z2)
{
    return sqrt((x1 - x2) * (x1 - x2) +
                (y1 - y2) * (y1 - y2) +
                (z1 - z2) * (z1 - z2));
}

bool findPath(const SimulationState &simState, int startX, int startY, int startZ,
              int goalX, int goalY, int goalZ, std::vector<std::vector<float>> &path)
{
    std::priority_queue<pathNode, std::vector<pathNode>, std::greater<pathNode>> openSet;
    std::unordered_map<std::tuple<int, int, int>, std::tuple<int, int, int>, Vec3Hash> cameFrom;
    std::unordered_map<std::tuple<int, int, int>, float, Vec3Hash> gScores;
    std::unordered_set<std::tuple<int, int, int>, Vec3Hash> closedSet;

    pathNode startNode(startX, startY, startZ, 0.0f, heuristic(startX, startY, startZ, goalX, goalY, goalZ));
    openSet.push(startNode);
    gScores[std::make_tuple(startX, startY, startZ)] = 0.0f;

    while (!openSet.empty())
    {
        pathNode current = openSet.top();
        openSet.pop();

        auto currentTuple = std::make_tuple(current.x, current.y, current.z);
        closedSet.insert(currentTuple);

        if (current.x == goalX && current.y == goalY && current.z == goalZ)
        {
            path.clear();
            path.push_back({current.x * pathCellSize - boxsize / 2.0f + pathCellSize / 2.0f,
                            current.y * pathCellSize - boxsize / 2.0f + pathCellSize / 2.0f,
                            current.z * pathCellSize - boxsize / 2.0f + pathCellSize / 2.0f});

            while (cameFrom.find(currentTuple) != cameFrom.end())
            {
                currentTuple = cameFrom[currentTuple];
                float wx = std::get<0>(currentTuple) * pathCellSize - boxsize / 2.0f + pathCellSize / 2.0f;
                float wy = std::get<1>(currentTuple) * pathCellSize - boxsize / 2.0f + pathCellSize / 2.0f;
                float wz = std::get<2>(currentTuple) * pathCellSize - boxsize / 2.0f + pathCellSize / 2.0f;
                path.push_back({wx, wy, wz});
            }

            std::reverse(path.begin(), path.end());
            return true;
        }

        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                for (int dz = -1; dz <= 1; dz++)
                {
                    int nx = current.x + dx;
                    int ny = current.y + dy;
                    int nz = current.z + dz;

                    if (nx < 0 || nx >= pathGridDim ||
                        ny < 0 || ny >= pathGridDim ||
                        nz < 0 || nz >= pathGridDim)
                        continue;

                    auto neighborTuple = std::make_tuple(nx, ny, nz);
                    if (closedSet.find(neighborTuple) != closedSet.end())
                        continue;

                    if (!(nx == goalX && ny == goalY && nz == goalZ) && isOccupied(simState, nx, ny, nz))
                        continue;

                    float moveCost = sqrt(dx * dx + dy * dy + dz * dz);
                    float tentativeG = gScores[currentTuple] + moveCost;

                    if (gScores.find(neighborTuple) == gScores.end() || tentativeG < gScores[neighborTuple])
                    {
                        gScores[neighborTuple] = tentativeG;
                        cameFrom[neighborTuple] = currentTuple;

                        float h = heuristic(nx, ny, nz, goalX, goalY, goalZ);
                        pathNode neighborNode(nx, ny, nz, tentativeG, h);
                        openSet.push(neighborNode);
                    }
                }
            }
        }
    }
    return false;
}

float distance3D(float x1, float y1, float z1, float x2, float y2, float z2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    float dz = z2 - z1;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

bool isPathClear(const SimulationState &simState, float x1, float y1, float z1,
                 float x2, float y2, float z2)
{
    float dist = distance3D(x1, y1, z1, x2, y2, z2);
    int steps = (int)(dist / (particleRad * 2.0f)) + 1;

    for (int i = 0; i <= steps; i++)
    {
        float t = (float)i / (float)steps;
        float wx = x1 + t * (x2 - x1);
        float wy = y1 + t * (y2 - y1);
        float wz = z1 + t * (z2 - z1);

        if (isOccupiedWorld(simState, wx, wy, wz))
        {
            return false;
        }
    }

    return true;
}

float randomFloat(float min, float max)
{
    return min + (float)rand() / (float)RAND_MAX * (max - min);
}

bool findPathRRTStar(const SimulationState &simState,
                     float startX, float startY, float startZ,
                     float goalX, float goalY, float goalZ,
                     std::vector<std::vector<float>> &path)
{
    const int maxIterations = 2000;
    const float stepSize = 0.3f;
    const float goalRadius = 0.3f;
    const float rewireRadius = 0.6f;
    const float halfSize = boxsize / 2.0f;

    std::vector<RRTNode> tree;
    tree.push_back(RRTNode(startX, startY, startZ, 0.0f, -1));

    int goalNodeIdx = -1;

    for (int iter = 0; iter < maxIterations; iter++)
    {
        float randX, randY, randZ;

        if (rand() % 10 == 0)
        {
            randX = goalX;
            randY = goalY;
            randZ = goalZ;
        }
        else
        {
            randX = randomFloat(-halfSize, halfSize);
            randY = randomFloat(-halfSize, halfSize);
            randZ = randomFloat(-halfSize, halfSize);
        }

        int nearestIdx = 0;
        float nearestDist = std::numeric_limits<float>::max();

        for (size_t i = 0; i < tree.size(); i++)
        {
            float d = distance3D(tree[i].x, tree[i].y, tree[i].z, randX, randY, randZ);
            if (d < nearestDist)
            {
                nearestDist = d;
                nearestIdx = (int)i;
            }
        }

        float dx = randX - tree[nearestIdx].x;
        float dy = randY - tree[nearestIdx].y;
        float dz = randZ - tree[nearestIdx].z;
        float dist = sqrt(dx * dx + dy * dy + dz * dz);

        float newX, newY, newZ;
        if (dist > stepSize)
        {
            newX = tree[nearestIdx].x + dx / dist * stepSize;
            newY = tree[nearestIdx].y + dy / dist * stepSize;
            newZ = tree[nearestIdx].z + dz / dist * stepSize;
        }
        else
        {
            newX = randX;
            newY = randY;
            newZ = randZ;
        }

        if (isOccupiedWorld(simState, newX, newY, newZ))
            continue;

        if (!isPathClear(simState, tree[nearestIdx].x, tree[nearestIdx].y, tree[nearestIdx].z,
                         newX, newY, newZ))
            continue;

        std::vector<int> nearNodes;
        for (size_t i = 0; i < tree.size(); i++)
        {
            float d = distance3D(tree[i].x, tree[i].y, tree[i].z, newX, newY, newZ);
            if (d < rewireRadius)
            {
                nearNodes.push_back((int)i);
            }
        }

        int bestParent = nearestIdx;
        float bestCost = tree[nearestIdx].cost +
                         distance3D(tree[nearestIdx].x, tree[nearestIdx].y, tree[nearestIdx].z,
                                    newX, newY, newZ);

        for (int idx : nearNodes)
        {
            float newCost = tree[idx].cost +
                            distance3D(tree[idx].x, tree[idx].y, tree[idx].z, newX, newY, newZ);

            if (newCost < bestCost &&
                isPathClear(simState, tree[idx].x, tree[idx].y, tree[idx].z, newX, newY, newZ))
            {
                bestParent = idx;
                bestCost = newCost;
            }
        }

        int newNodeIdx = (int)tree.size();
        tree.push_back(RRTNode(newX, newY, newZ, bestCost, bestParent));

        for (int idx : nearNodes)
        {
            float rewireCost = bestCost +
                               distance3D(newX, newY, newZ, tree[idx].x, tree[idx].y, tree[idx].z);

            if (rewireCost < tree[idx].cost &&
                isPathClear(simState, newX, newY, newZ, tree[idx].x, tree[idx].y, tree[idx].z))
            {
                tree[idx].parent = newNodeIdx;
                tree[idx].cost = rewireCost;
            }
        }

        float distToGoal = distance3D(newX, newY, newZ, goalX, goalY, goalZ);
        if (distToGoal < goalRadius)
        {
            if (goalNodeIdx < 0 || tree[newNodeIdx].cost < tree[goalNodeIdx].cost)
            {
                goalNodeIdx = newNodeIdx;
            }
        }
    }

    if (goalNodeIdx < 0)
    {
        return false;
    }

    path.clear();
    int currentIdx = goalNodeIdx;

    while (currentIdx >= 0)
    {
        path.push_back({tree[currentIdx].x, tree[currentIdx].y, tree[currentIdx].z});
        currentIdx = tree[currentIdx].parent;
    }

    std::reverse(path.begin(), path.end());
    return true;
}

void updatePath(SimulationState &simState)
{
    if (simState.robotParticle < 0 || simState.goalParticle < 0)
        return;

    if (!simState.particles[simState.robotParticle].active ||
        !simState.particles[simState.goalParticle].active)
        return;

    float startX = simState.particles[simState.robotParticle].x;
    float startY = simState.particles[simState.robotParticle].y;
    float startZ = simState.particles[simState.robotParticle].z;
    float goalX = simState.particles[simState.goalParticle].x;
    float goalY = simState.particles[simState.goalParticle].y;
    float goalZ = simState.particles[simState.goalParticle].z;

    // Save old path for divergence measurement
    std::vector<std::vector<float>> oldPath = simState.currentPath;

    // ── Timed planning call ───────────────────────────────────────────────────
    auto t0 = std::chrono::high_resolution_clock::now();
    bool pathFound = false;

    if (simState.useRRTStar)
    {
        pathFound = findPathRRTStar(simState, startX, startY, startZ,
                                    goalX, goalY, goalZ, simState.currentPath);
    }
    else
    {
        int startGX, startGY, startGZ, goalGX, goalGY, goalGZ;
        worldToPathgrid(startX, startY, startZ, startGX, startGY, startGZ);
        worldToPathgrid(goalX, goalY, goalZ, goalGX, goalGY, goalGZ);
        pathFound = findPath(simState, startGX, startGY, startGZ,
                             goalGX, goalGY, goalGZ, simState.currentPath);
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    long long planUs = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
    // ─────────────────────────────────────────────────────────────────────────

    if (!pathFound)
        simState.currentPath.clear();

    // ── Record metrics ────────────────────────────────────────────────────────
    ReplanEvent ev{};
    ev.timestamp = simState.elapsedTime;
    ev.planTimeUs = planUs;
    ev.pathFound = pathFound;
    ev.pathLength = computePathLength(simState.currentPath);
    ev.pathNodes = (int)simState.currentPath.size();
    ev.pathDivergence = computePathDivergence(oldPath, simState.currentPath);
    simState.replanLog.push_back(ev);
    // ─────────────────────────────────────────────────────────────────────────
}