#include "pathFinding.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <iostream>

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

                    if (nx < 0 || nx >= gridWidth ||
                        ny < 0 || ny >= gridWidth ||
                        nz < 0 || nz >= gridWidth)
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

void updatePath(SimulationState &simState)
{
    if (simState.robotParticle < 0 || simState.goalParticle < 0)
    {
        return;
    }

    if (!simState.particles[simState.robotParticle].active || !simState.particles[simState.goalParticle].active)
    {
        return;
    }

    int startGX, startGY, startGZ;
    int goalGX, goalGY, goalGZ;

    worldToPathgrid(simState.particles[simState.robotParticle].x,
                    simState.particles[simState.robotParticle].y,
                    simState.particles[simState.robotParticle].z,
                    startGX, startGY, startGZ);

    worldToPathgrid(simState.particles[simState.goalParticle].x,
                    simState.particles[simState.goalParticle].y,
                    simState.particles[simState.goalParticle].z,
                    goalGX, goalGY, goalGZ);

    if (findPath(simState, startGX, startGY, startGZ,
                 goalGX, goalGY, goalGZ, simState.currentPath))
    {
        simState.pathUpdateCounter++;
        std::cout << "Path found " << simState.pathUpdateCounter << ": " << simState.currentPath.size() << " nodes" << std::endl;
    }
    else
    {
        std::cout << "No path found " << std::endl;
        simState.currentPath.clear();
    }
}