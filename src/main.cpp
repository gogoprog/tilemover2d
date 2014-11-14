#include "tilemover2d.h"

#include <GL/glut.h>
#include <iostream>
using namespace std;
using namespace tilemover2d;

const float
    tileSize(32.0f);
const int
    width(32),
    height(32);
GLUquadricObj
    * disk;
World
    world;
Path
    path;
MP_VECTOR<Agent *>
    agents;
int
    previousTime(0);

int mainTest()
{
    cout << "Initializing world" << endl;

    world.init(width, height, tileSize, tileSize);

    cout << "Finding a path" << endl;

    world.findPath(path, Point(1,1), Point(4, 4));

    path.debugPrint();

    cout << "Creating agent" << endl;

    agents.push_back(& world.createAgent(Position(10.0f, 10.0f)));
    agents[0]->radius = 10.0f;

    agents[0]->moveTo(Position(300.0f, 300.0f));

    return 0;
}

void drawGrid()
{
    glColor3f(0.5f, 0.5f, 0.5f);

    for(int i=0; i<width; i++)
    {
        glBegin(GL_LINES);
        glVertex2f(i * tileSize, height * tileSize);
        glVertex2f(i * tileSize, -height * tileSize);
        glEnd();
    }

    for(int i=0; i<height; i++)
    {
        glBegin(GL_LINES);
        glVertex2f(-width * tileSize, i * tileSize);
        glVertex2f(width * tileSize, i * tileSize);
        glEnd();
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

    gluOrtho2D(- width * tileSize * 0.5f, width * tileSize * 0.5, - height * tileSize * 0.5, height * tileSize * 0.5);
    glTranslatef(-width * tileSize * 0.5f, -height * tileSize * 0.5f, 0.0f);

    disk = gluNewQuadric();

    glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
    glutMainLoop();

    return 0;
}