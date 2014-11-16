#include "tilemover2d.h"

#include <GL/glut.h>
#include <iostream>
using namespace std;
using namespace tilemover2d;

const float
    tileSize(32.0f);
const int
    xTileCount(32),
    yTileCount(32);
GLUquadricObj
    * disk;
World
    world;
Path
    path;
MP_VECTOR<Agent *>
    agents;
int
    previousTime(0),
    windowWidth,
    windowHeight;

int mainTest()
{
    cout << "Initializing world" << endl;

    world.init(xTileCount, yTileCount, tileSize, tileSize);

    cout << "Finding a path" << endl;

    world.setTileBlocking(5, 5, true);

    world.findPath(path, Point(1,1), Point(4, 4));

    path.debugPrint();

    cout << "Creating agent" << endl;

    agents.push_back(& world.createAgent(Position(10.0f, 10.0f)));
    agents[0]->radius = 10.0f;
    agents[0]->speed = 100.0f;

    agents[0]->moveTo(Position(300.0f, 300.0f));

    return 0;
}

void drawGrid()
{
    glColor3f(0.5f, 0.5f, 0.5f);

    for(int i=0; i<xTileCount; i++)
    {
        glBegin(GL_LINES);
        glVertex2f(i * tileSize, yTileCount * tileSize);
        glVertex2f(i * tileSize, -yTileCount * tileSize);
        glEnd();
    }

    for(int i=0; i<yTileCount; i++)
    {
        glBegin(GL_LINES);
        glVertex2f(-xTileCount * tileSize, i * tileSize);
        glVertex2f(xTileCount * tileSize, i * tileSize);
        glEnd();
    }
}

void drawBlockingTiles()
{
    glColor3f(0.6f, 0.1f, 0.1f);

    for(int x=0; x<xTileCount; ++x)
    {
        for(int y=0; y<yTileCount; ++y)
        {
            if(world.getTile(x, y).isBlocking())
            {
                Position position;

                position.x = x * tileSize;
                position.y = y * tileSize;

                glBegin(GL_QUADS);
                glVertex2f(position.x, position.y);
                glVertex2f(position.x + tileSize, position.y);
                glVertex2f(position.x + tileSize, position.y + tileSize);
                glVertex2f(position.x, position.y + tileSize);
                glEnd();
            }
        }
    }
}

void drawAgent(const Agent & agent)
{
    glPushMatrix();
    glTranslatef(agent.getPosition().x, agent.getPosition().y, 0);
    glColor3f(0.0f, 0.0f, 0.0f);
    gluDisk(disk, 0, agent.radius, 32, 32);
    glColor3f(0.0f, 0.0f, 0.8f);
    gluDisk(disk, 0, agent.radius - 3.0f, 32, 32);
    glPopMatrix();
}

void drawPath(const Path & path)
{
    glColor3f(0.0f, 0.8f, 0.0f);

    for(int i=0; i<path.positions.size(); i++)
    {
        glPushMatrix();
        glTranslatef(path.positions[i].x, path.positions[i].y, 0);
        gluDisk(disk, 4, 5, 10, 10);
        glPopMatrix();
    }

    glBegin(GL_LINE_STRIP);

    for(int i=0; i<path.positions.size(); i++)
    {
        glVertex2f(path.positions[i].x, path.positions[i].y);
    }

    glEnd();
}

void mainLoop(void)
{
    int currentTime = glutGet(GLUT_ELAPSED_TIME);
    int deltaTime = currentTime - previousTime;
    previousTime = currentTime;

    world.update(deltaTime / 1000.0f);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    drawGrid();
    drawBlockingTiles();

    for(int i=0; i<agents.size(); i++)
    {
        const Agent & agent = * agents[i];
        drawPath(agent.getPath());
    }

    for(int i=0; i<agents.size(); i++)
    {
        const Agent & agent = * agents[i];
        drawAgent(agent);
    }

    glutSwapBuffers();
    glutPostRedisplay();
}

void onMouseEvent(int button, int state, int x, int y)
{
    if(state == GLUT_DOWN)
    {
        Position world_position;
        Point point;

        world_position.x = (x / float(windowWidth)) * xTileCount * tileSize;
        world_position.y = ((windowHeight - y) / float(windowHeight)) * yTileCount * tileSize;

        if(world.findPoint(point, world_position))
        {
            if(button == GLUT_LEFT_BUTTON)
            {
                agents[0]->moveTo(world_position);
            }
            else
            {
                world.setTileBlocking(point.x, point.y, true);
            }
        }
    }
}

void reshape(int window_width, int window_height)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glViewport(0, 0, window_width, window_height);
    gluOrtho2D(-xTileCount * tileSize * 0.5f, xTileCount * tileSize * 0.5, - yTileCount * tileSize * 0.5, yTileCount * tileSize * 0.5);
    glTranslatef(-xTileCount * tileSize * 0.5f, -yTileCount * tileSize * 0.5f, 0.0f);
    glMatrixMode(GL_MODELVIEW);

    windowWidth = window_width;
    windowHeight = window_height;
}

int main(int argc, char **argv)
{
    cout << "tilemover2d test application" << endl;
    mainTest();

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowPosition(100,100);
    glutInitWindowSize(512, 512);
    glutCreateWindow("tilemover2d - test");
    glutDisplayFunc(mainLoop);
    glutMouseFunc(onMouseEvent);
    glutReshapeFunc(reshape);

    disk = gluNewQuadric();

    glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
    glutMainLoop();

    return 0;
}