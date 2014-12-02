#include "tilemover2d.h"

#include <cstdio>
#include <algorithm>
#include <cassert>

namespace tilemover2d
{

float getSquareDistance(const Vector2 & a, const Vector2 & b)
{
    Vector2 delta(b.x - a.x, b.y - a.y);
    return (delta.x * delta.x + delta.y * delta.y);
}

float getDistance(const Vector2 & a, const Vector2 & b)
{
    Vector2 delta(b.x - a.x, b.y - a.y);
    return sqrt(delta.x * delta.x + delta.y * delta.y);
}

float getLength(const Vector2 & a)
{
    return sqrt(a.x * a.x + a.y * a.y);
}

bool lineCircleIntersection(const Vector2 & from,const Vector2 & to, const Vector2 & circleCenter, const float radius)
{
    float dx = to.x - from.x;
    float dy = to.y - from.y;
    float a = dx * dx + dy * dy;
    float b = 2 * (dx * (from.x - circleCenter.x) + dy * (from.y - circleCenter.y));
    float c = circleCenter.x * circleCenter.x + circleCenter.y * circleCenter.y;

    c += from.x * from.x + from.y * from.y;
    c -= 2 * (circleCenter.x * from.x + circleCenter.y * from.y);
    c -= radius * radius;

    float bb4ac = b * b - 4 * a * c;

    return bb4ac >= 0;
}

bool circleCircleIntersection(const Vector2 & circleCenter, const float radius, const Vector2 & circleCenter2, const float radius2)
{
    return getSquareDistance(circleCenter, circleCenter2) < (radius + radius2) * (radius + radius2);
}

//
// Vector2
//

Vector2 & Vector2::operator+=(const Vector2 & other)
{
    x += other.x;
    y += other.y;

    return * this;
}

Vector2 & Vector2::operator-=(const Vector2 & other)
{
    x -= other.x;
    y -= other.y;

    return * this;
}

Vector2 & Vector2::operator*=(const float value)
{
    x *= value;
    y *= value;

    return * this;
}

Vector2 operator+(const Vector2 & a, const Vector2 & b)
{
    return Vector2(a.x + b.x, a.y + b.y);
}

Vector2 operator-(const Vector2 & a, const Vector2 & b)
{
    return Vector2(a.x - b.x, a.y - b.y);
}

Vector2 operator*(const Vector2 & a, const float value)
{
    return Vector2(a.x * value, a.y * value);
}

bool operator==(const Vector2 & a, const Vector2 & b)
{
    return a.x == b.x && a.y == b.y;
}

bool operator!=(const Vector2 & a, const Vector2 & b)
{
    return a.x != b.x || a.y != b.y;
}

//
// Path
//

void Path::debugPrint() const
{
    for(int i=0; i<positions.size(); i++)
    {
        const Vector2 & p = positions[i];

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

void Agent::moveTo(const Vector2 & target_position)
{
    if(world->findPath(path, position, target_position))
    {
        MP_VECTOR<Vector2> & positions = path.positions;
        state = State::ACTIVE;
        positions[0] = position;
        positions[positions.size() - 1] = target_position;

        prepareForTargetIndex(1);
    }
}

void Agent::prepareForTargetIndex(const uint index)
{
    currentTargetIndex = index;
}

void Agent::update(const float dt)
{
    switch(state)
    {
        case State::ACTIVE:
        {
            const Vector2 & previousPosition = path.positions[currentTargetIndex - 1];
            const Vector2 & nextPosition = path.positions[currentTargetIndex];

            velocity = nextPosition - position;

            float length = getDistance(nextPosition, position);

            velocity *= speed / length;

            desiredDisplacement = velocity * dt;
        }
        break;

        case State::IDLE:
        break;
    }
}

void Agent::postUpdate()
{
    switch(state)
    {
        case State::ACTIVE:
        {
            const Vector2 & nextPosition = path.positions[currentTargetIndex];

            position += desiredDisplacement;

            if(getSquareDistance(nextPosition, position) < 1.0f)
            {
                if(currentTargetIndex < path.positions.size() - 1)
                {
                    ++currentTargetIndex;
                    prepareForTargetIndex(currentTargetIndex);
                }
                else
                {
                    state = State::IDLE;
                }
            }
        }
    }
}

void Agent::recomputePath()
{
    switch(state)
    {
        case State::ACTIVE:
        {
            if(path.positions.size() > 0)
            {
                Vector2 target_position = path.positions[path.positions.size() - 1];
                moveTo(target_position);
            }
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

bool World::findPath(Path & path, const Vector2 & from, const Vector2 & to)
{
    Point a, b;
    getPointFromPosition(a, from);
    getPointFromPosition(b, to);

    return findPath(path, a, b);
}

bool World::findPath(Path & path, const Point & from, const Point & to)
{
    Point point;
    path.cost = 0;

    lastComputedPath.resize(0);

    int result = pather.Solve(getNodeFromPoint(from), getNodeFromPoint(to), &lastComputedPath, &path.cost);

    path.positions.resize(lastComputedPath.size());

    for(int i=0; i<lastComputedPath.size(); i++)
    {
        getPointFromNode(point, lastComputedPath[i]);

        getPositionFromPoint(path.positions[i], point);
    }

    return result == micropather::MicroPather::SOLVED;
}

bool World::findPoint(Point & point, const Vector2 & position) const
{
    if(position.x > 0 && position.x <= tileWidth * width
        && position.y > 0 && position.y <= tileHeight * height
        )
    {
        getPointFromPosition(point, position);
        return true;
    }

    return false;
}

Agent & World::createAgent(const Vector2 & position)
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

    bool it_collides;

    do
    {
        it_collides = false;

        for(int i=0; i<agents.size(); ++i)
        {
            Agent & agent = * agents[i];

            for(int j=0; j<agents.size(); ++j)
            {
                Agent & other_agent = * agents[j];

                if(i != j)
                {
                    if(circleCircleIntersection(agent.position + agent.desiredDisplacement, agent.radius, other_agent.position + other_agent.desiredDisplacement, other_agent.radius))
                    {
                        if(agent.desiredDisplacement != Vector2(0,0))
                        {
                            //it_collides = true;
                            //agent.position -= agent.desiredDisplacement * 0.5f;

                            agent.desiredDisplacement = Vector2(0, 0);
                            continue;
                        }
                    }
                }
            }

            if(it_collides)
            {
                continue;
            }
        }
    }
    while(it_collides);

    for(int i=0; i<agents.size(); ++i)
    {
        agents[i]->postUpdate();
    }
}

float World::LeastCostEstimate(void* stateStart, void* stateEnd)
{
    Point start, end;
    getPointFromNode(start, stateStart);
    getPointFromNode(end, stateEnd);
    int dx = start.x - end.x;
    int dy = start.y - end.y;
    return float(dx*dx + dy*dy);
}

void World::AdjacentCost(void* state, MP_VECTOR< micropather::StateCost > *adjacent)
{
    static const int dx[5] = { 1, 0, -1, 0, 1 };
    static const int dy[5] = { 0, 1, 0, -1, 0 };

    Point p;
    bool result[5];

    getPointFromNode(p, state);

    for(int i=0; i<4; ++i)
    {
        int nx = p.x + dx[i];
        int ny = p.y + dy[i];
        result[i] = false;

        if(nx < 0 || ny < 0 || nx >= width || ny >= height)
        {
            continue;
        }

        const Tile & tile = getTile(nx, ny);

        if(!tile.blocking)
        {
            micropather::StateCost node_cost = { getNodeFromPoint(Point(nx, ny)), 1.0f };
            adjacent->push_back(node_cost);
            result[i] = true;
        }
    }

    result[4] = result[0];

    for(int i=0; i<4; ++i)
    {
        if(result[i] && result[i+1])
        {
            int nx = p.x + dx[i] + dx[i + 1];
            int ny = p.y + dy[i] + dy[i + 1];

            const Tile & tile = getTile(nx, ny);

            if(nx < 0 || ny < 0 || nx >= width || ny >= height)
            {
                continue;
            }

            if(!tile.blocking)
            {
                micropather::StateCost node_cost = { getNodeFromPoint(Point(nx, ny)), 1.41f };
                adjacent->push_back(node_cost);
            }
        }
    }
}

Tile & World::internalGetTile(const int x, const int y)
{
    return tiles[y*width + x];
}

void World::getPointFromNode(Point & p, void* node) const
{
    intptr_t index = (intptr_t)node;
    p.y = index / width;
    p.x = index % width;
}

void *World::getNodeFromPoint(const Point & p) const
{
    return (void*) (p.y*width + p.x);
}

void World::getPositionFromPoint(Vector2 & position, const Point & point) const
{
    position.x = point.x * tileWidth + tileWidth * 0.5f;
    position.y = point.y * tileHeight + tileHeight * 0.5f;
}

void World::getPointFromPosition(Point & point, const Vector2 & position) const
{
    point.x = int(position.x / tileWidth);
    point.y = int(position.y / tileHeight);
}

}
