#include "tilemover2d.h"

#include <cstdio>
#include <algorithm>

namespace tilemover2d
{

//
// Path
//

void Path::debugPrint() const
{
    for(int i=0; i<positions.size(); i++)
    {
        const Position & p = positions[i];

        printf("(%.2f, %.2f)\n", p.x, p.y);
    }
}

//
// Agent
//

Agent::Agent()
    :
    speed(1.0f),
    radius(1.0f)
{
}

void Agent::moveTo(const Position & target_position)
{
    if(world->findPath(path, position, target_position))
    {
        MP_VECTOR<Position> & positions = path.positions;
        state = State::ACTIVE;
        positions[0] = position;
        positions[positions.size() - 1] = target_position;
        currentTime = 0.0f;
        currentDuration = 1.0f;
        currentTargetIndex = 1;
    }
}

void Agent::update(const float dt)
{
    switch(state)
    {
        case State::ACTIVE:
        {
            const Position & previousPosition = path.positions[currentTargetIndex - 1];
            const Position & nextPosition = path.positions[currentTargetIndex];

            currentTime += dt;
            currentTime = std::min(currentTime, currentDuration);

            float factor = currentTime / currentDuration;

            position.x = previousPosition.x + (nextPosition.x - previousPosition.x) * factor;
            position.y = previousPosition.y + (nextPosition.y - previousPosition.y) * factor;

            if(currentTime >= currentDuration)
            {
                if(currentTargetIndex < path.positions.size() - 1)
                {
                    ++currentTargetIndex;
                    currentTime = 0.0f;
                    currentDuration = 1.0f;
                }
                else
                {
                    state = State::IDLE;
                }
            }
        }
        break;

        case State::IDLE:
        break;
    }
}

void Agent::recomputePath()
{
    switch(state)
    {
        case State::ACTIVE:
        {
            Position target_position = path.positions[path.positions.size() - 1];
            moveTo(target_position);
            printf("(%.2f, %.2f)\n", target_position.x, target_position.y);

        }
    }
}

//
// World
//

World::World()
    :
    pather(this),
    mustReset(false)
{
}

void World::init(const int _width, const int _height, const float tile_width, const float tile_height)
{
    width = _width;
    height = _height;
    tileWidth = tile_width;
    tileHeight = tile_height;

    tiles = new Tile[width * height];
    memset(tiles, 0, sizeof(Tile) * width * height);
}

void World::setTileBlocking(const int x, const int y, const bool blocking)
{
    internalGetTile(x, y).blocking = blocking;
    mustReset = true;
}

const Tile & World::getTile(const int x, const int y) const
{
    return tiles[y*width + x];
}

bool World::findPath(Path & path, const Position & from, const Position & to)
{
    Point a, b;
    positionToPoint(a, from);
    positionToPoint(b, to);

    return findPath(path, a, b);
}

bool World::findPath(Path & path, const Point & from, const Point & to)
{
    Point point;
    path.cost = 0;
    int result = pather.Solve(pointToNode(from), pointToNode(to), &lastComputedPath, &path.cost);

    path.positions.resize(lastComputedPath.size());
    for(int i=0; i<lastComputedPath.size(); i++)
    {
        nodeToPoint(lastComputedPath[i],point);

        pointToPosition(path.positions[i], point);
    }

    path.debugPrint();

    return result == micropather::MicroPather::SOLVED;
}

bool World::findPoint(Point & point, const Position & position) const
{
    if(position.x > 0 && position.x <= tileWidth * width
        && position.y > 0 && position.y <= tileHeight * height
        )
    {
        positionToPoint(point, position);
        return true;
    }

    return false;
}

Agent & World::createAgent(const Position & position)
{
    Agent & agent = * new Agent();

    agent.position = position;
    agent.state = Agent::State::IDLE;
    agent.world = this;

    agents.push_back(& agent);

    return agent;
}

void World::update(const float dt)
{
    if(mustReset)
    {
        pather.Reset();
        mustReset = false;

        for(int i=0; i<agents.size(); ++i)
        {
            agents[i]->recomputePath();
        }
    }

    for(int i=0; i<agents.size(); ++i)
    {
        agents[i]->update(dt);
    }
}

float World::LeastCostEstimate(void* stateStart, void* stateEnd)
{
    Point start, end;
    nodeToPoint(stateStart, start);
    nodeToPoint(stateEnd, end);
    int dx = start.x - end.x;
    int dy = start.y - end.y;
    return float(dx*dx + dy*dy);
}

void World::AdjacentCost(void* state, MP_VECTOR< micropather::StateCost > *adjacent)
{
    static const int dx[4] = { 1, 0, -1, 0 };
    static const int dy[4] = { 0, 1, 0, -1 };

    Point p;

    nodeToPoint(state, p);

    for(int i=0; i<4; ++i)
    {
        int nx = p.x + dx[i];
        int ny = p.y + dy[i];

        if(nx < 0 || ny < 0 || nx >= width || ny >= height)
        {
            continue;
        }

        const Tile & tile = getTile(nx, ny);

        if(!tile.blocking)
        {
            micropather::StateCost node_cost = { pointToNode(Point(nx, ny)), 1.0f };
            adjacent->push_back(node_cost);
        }
    }
}

Tile & World::internalGetTile(const int x, const int y)
{
    return tiles[y*width + x];
}

void World::nodeToPoint(void* node, Point & p) const
{
    intptr_t index = (intptr_t)node;
    p.y = index / width;
    p.x = index - p.y * width;
}

void *World::pointToNode(const Point & p) const
{
    return (void*) (p.y*width + p.x);
}

void World::pointToPosition(Position & position, const Point & point) const
{
    position.x = point.x * tileWidth + tileWidth * 0.5f;
    position.y = point.y * tileHeight + tileHeight * 0.5f;
}

void World::positionToPoint(Point & point, const Position & position) const
{
    point.x = int(position.x / tileWidth);
    point.y = int(position.y / tileHeight);
}

}
