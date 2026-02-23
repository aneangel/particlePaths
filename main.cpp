#include <GL/glew.h>
#include <GL/glut.h>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include "particle.h"
#include "pathFinding.h"
#include "robot.h"

const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;

float cameraAngleX = 30.0f;
float cameraAngleY = 45.0f;
float cameraDistance = 10.0f;

bool mouseLeftDown = false;
int mouseX = 0;
int mouseY = 0;

int frameCounter = 0;
// SimulationState simState;

void setupLighting()
{
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glEnable(GL_NORMALIZE);

    GLfloat lightPos0[] = {0.0f, 10.0f, 0.0f, 1.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos0);
    GLfloat ambient0[] = {0.15f, 0.15f, 0.15f, 1.0f};
    GLfloat diffuse0[] = {0.9f, 0.9f, 0.85f, 1.0f};
    GLfloat specular0[] = {1.0f, 1.0f, 1.0f, 1.0f};
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient0);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse0);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular0);

    glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 1.0f);
    glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.02f);
    glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, 0.005f);
    // GLfloat lightPosition[] = {0.0f, 5.0f, 0.0f, 1.0f};
    // glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);

    // GLfloat ambientLight[] = {0.3f, 0.3f, 0.3f, 1.0f};
    // glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);

    // GLfloat diffuseLight[] = {0.8f, 0.8f, 0.8f, 1.0f};
    // glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);

    // GLfloat specularLight[] = {1.0f, 1.0f, 1.0f, 1.0f};
    // glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);

    // glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 1.0f);
    // glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.05f);
    // glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, 0.01f);
    GLfloat lightPos1[] = {0.0f, -10.0f, 0.0f, 1.0f};
    glLightfv(GL_LIGHT1, GL_POSITION, lightPos1);
    GLfloat ambient1[] = {0.05f, 0.05f, 0.1f, 1.0f};
    GLfloat diffuse1[] = {0.2f, 0.2f, 0.35f, 1.0f};
    GLfloat specular1[] = {0.0f, 0.0f, 0.0f, 1.0f};
    glLightfv(GL_LIGHT1, GL_AMBIENT, ambient1);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse1);
    glLightfv(GL_LIGHT1, GL_SPECULAR, specular1);

    GLfloat globalAmbient[] = {0.2f, 0.2f, 0.25f, 1.0f};
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, globalAmbient);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
}

void setMaterial(GLfloat r, GLfloat g, GLfloat b, GLfloat shininess)
{
    GLfloat materialAmbient[] = {r * 0.2f, g * 0.2f, b * 0.2f, 1.0f};
    GLfloat materialDiffuse[] = {r, g, b, 1.0f};
    GLfloat materialSpecular[] = {1.0f, 1.0f, 1.0f, 1.0f};

    glMaterialfv(GL_FRONT, GL_AMBIENT, materialAmbient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, materialDiffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR, materialSpecular);
    glMaterialf(GL_FRONT, GL_SHININESS, shininess);
}

void drawWireframeCube()
{
    glDisable(GL_LIGHTING);
    glColor3f(0.0f, 0.0f, 0.0f);
    glLineWidth(1.0f);

    float halfSize = boxsize / 2.0f;

    glBegin(GL_LINES);
    // Bottom face
    glVertex3f(-halfSize, -halfSize, -halfSize);
    glVertex3f(halfSize, -halfSize, -halfSize);

    glVertex3f(halfSize, -halfSize, -halfSize);
    glVertex3f(halfSize, -halfSize, halfSize);

    glVertex3f(halfSize, -halfSize, halfSize);
    glVertex3f(-halfSize, -halfSize, halfSize);

    glVertex3f(-halfSize, -halfSize, halfSize);
    glVertex3f(-halfSize, -halfSize, -halfSize);

    // Top face
    glVertex3f(-halfSize, halfSize, -halfSize);
    glVertex3f(halfSize, halfSize, -halfSize);

    glVertex3f(halfSize, halfSize, -halfSize);
    glVertex3f(halfSize, halfSize, halfSize);

    glVertex3f(halfSize, halfSize, halfSize);
    glVertex3f(-halfSize, halfSize, halfSize);

    glVertex3f(-halfSize, halfSize, halfSize);
    glVertex3f(-halfSize, halfSize, -halfSize);

    // Vertical edges
    glVertex3f(-halfSize, -halfSize, -halfSize);
    glVertex3f(-halfSize, halfSize, -halfSize);

    glVertex3f(halfSize, -halfSize, -halfSize);
    glVertex3f(halfSize, halfSize, -halfSize);

    glVertex3f(halfSize, -halfSize, halfSize);
    glVertex3f(halfSize, halfSize, halfSize);

    glVertex3f(-halfSize, -halfSize, halfSize);
    glVertex3f(-halfSize, halfSize, halfSize);
    glEnd();

    glEnable(GL_LIGHTING);
}

void drawPath()
{
    if (simState.currentPath.empty())
        return;

    glDisable(GL_LIGHTING);
    glColor3f(1.0f, 0.0f, 1.0f);
    glLineWidth(3.0f);

    glBegin(GL_LINE_STRIP);
    for (const auto &point : simState.currentPath)
    {
        glVertex3f(point[0], point[1], point[2]);
    }
    glEnd();

    glEnable(GL_LIGHTING);
}

void drawParticles()
{
    for (size_t i = 0; i < simState.particles.size(); i++)
    {
        if (!simState.particles[i].active)
            continue;

        if ((int)i == simState.robotParticle || (int)i == simState.goalParticle)
            continue;

        glPushMatrix();
        float densityRatio = simState.particles[i].density / REST_DENISITY;
        float blue = 0.3f + 0.7f * std::min(densityRatio, 1.0f);
        setMaterial(0.2f, 0.5f, blue, 50.0f);
        glTranslatef(simState.particles[i].x, simState.particles[i].y, simState.particles[i].z);
        glutSolidSphere(particleRad, 8, 8);
        glPopMatrix();
    }

    float highlightRadius = particleRad;

    glDisable(GL_LIGHTING);

    if (simState.robotParticle >= 0 && simState.particles[simState.robotParticle].active)
    {
        glPushMatrix();
        glColor3f(0.0f, 1.0f, 0.0f);
        glTranslatef(simState.particles[simState.robotParticle].x,
                     simState.particles[simState.robotParticle].y,
                     simState.particles[simState.robotParticle].z);
        glutSolidSphere(highlightRadius, 16, 16);
        glPopMatrix();
    }

    if (simState.goalParticle >= 0 && simState.particles[simState.goalParticle].active)
    {
        glPushMatrix();
        glColor3f(1.0f, 0.0f, 0.0f);
        glTranslatef(simState.particles[simState.goalParticle].x,
                     simState.particles[simState.goalParticle].y,
                     simState.particles[simState.goalParticle].z);
        glutSolidSphere(highlightRadius, 16, 16);
        glPopMatrix();
    }

    glEnable(GL_LIGHTING);
}

void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    float camX = cameraDistance * sin(cameraAngleY * M_PI / 180.0f) * cos(cameraAngleX * M_PI / 180.0f);
    float camY = cameraDistance * sin(cameraAngleX * M_PI / 180.0f);
    float camZ = cameraDistance * cos(cameraAngleY * M_PI / 180.0f) * cos(cameraAngleX * M_PI / 180.0f);

    gluLookAt(camX, camY, camZ, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    drawWireframeCube();
    drawParticles();
    drawPath();

    glutSwapBuffers();
}

void reshape(int width, int height)
{
    if (height == 0)
        height = 1;

    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, (float)width / (float)height, 0.1f, 100.0f);
    glMatrixMode(GL_MODELVIEW);
}

void agitateParticles()
{
    for (auto &p : simState.particles)
    {
        if (p.active)
        {
            p.vx += ((rand() % 100) / 100.0f - 0.5f) * 5.0f;
            p.vy += ((rand() % 100) / 100.0f) * 3.0f;
            p.vz += ((rand() % 100) / 100.0f - 0.5f) * 5.0f;
        }
    }
}

void idle()
{
    updatePhysics(simState);
    initializeRobotAndGoal(simState);

    if (simState.particlesSpawned >= simState.targgetParticleCount && !simState.initialAgitationDone)
    {
        simState.settlingFrames++;

        if (simState.settlingFrames == 1)
        {
            agitateParticles();
            std::cout << "Initial agitation applied, waiting for particles to settle..." << std::endl;
        }

        if (simState.settlingFrames >= 200)
        {
            simState.initialAgitationDone = true;
            std::cout << "Particles settled, selecting robot and goal" << std::endl;
        }
    }

    updateRobotControl(simState);

    if (simState.goalReached)
    {
        std::cout << "Resetting simulation..." << std::endl;
        simState.reset();
        for (int i = 0; i < simState.targgetParticleCount; i++)
        {
            simState.particles.push_back(Particle(0.0f, 0.0f, 0.0f));
        }
        frameCounter = 0;
    }

    frameCounter++;
    if (frameCounter >= 10)
    {
        frameCounter = 0;
        updatePath(simState);
    }

    glutPostRedisplay();
}

void keyboard(unsigned char key, int x, int y)
{
    (void)x;
    (void)y;

    switch (key)
    {
    case 'q':
    case 'Q':
    case 27: // ESC
        glutIdleFunc(nullptr);
        exit(0);
        break;

    case 'w':
    case 'W':
        cameraAngleX += 5.0f;
        glutPostRedisplay();
        break;

    case 's':
    case 'S':
        cameraAngleX -= 5.0f;
        glutPostRedisplay();
        break;

    case 'a':
    case 'A':
        cameraAngleY -= 5.0f;
        glutPostRedisplay();
        break;

    case 'd':
    case 'D':
        cameraAngleY += 5.0f;
        glutPostRedisplay();
        break;

    case '+':
    case '=':
        cameraDistance -= 0.5f;
        if (cameraDistance < 1.0f)
            cameraDistance = 1.0f;
        glutPostRedisplay();
        break;

    case '-':
    case '_':
        cameraDistance += 0.5f;
        glutPostRedisplay();
        break;

    case 'r':
    case 'R':
        simState.reset();
        frameCounter = 0;
        std::cout << "Simulation reset" << std::endl;
        break;

    case ' ':
        for (auto &p : simState.particles)
        {
            if (p.active)
            {
                p.vx += ((rand() % 100) / 100.0f - 0.5f) * 5.0f;
                p.vy += ((rand() % 100) / 100.0f) * 3.0f;
                p.vz += ((rand() % 100) / 100.0f - 0.5f) * 5.0f;
            }
        }
        std::cout << "Particles agitated" << std::endl;
        break;
    }
}

void mouse(int button, int state, int x, int y)
{
    if (button == GLUT_LEFT_BUTTON)
    {
        if (state == GLUT_DOWN)
        {
            mouseLeftDown = true;
            mouseX = x;
            mouseY = y;
        }
        else if (state == GLUT_UP)
        {
            mouseLeftDown = false;
        }
    }

    if (button == 3 && state == GLUT_DOWN)
    {
        cameraDistance -= 0.5f;
        if (cameraDistance < 1.0f)
        {
            cameraDistance = 1.0f;
        }

        glutPostRedisplay();
    }
    if (button == 4 && state == GLUT_DOWN)
    {
        cameraDistance += 0.5f;
        glutPostRedisplay();
    }
}

void motion(int x, int y)
{
    if (mouseLeftDown)
    {
        int dx = x - mouseX;
        int dy = y - mouseY;

        cameraAngleY += dx * 0.5f;
        cameraAngleX += dy * 0.5f;

        if (cameraAngleX > 89.0f)
            cameraAngleX = 89.0f;
        if (cameraAngleX < -89.0f)
            cameraAngleX = -89.0f;

        mouseX = x;
        mouseY = y;

        glutPostRedisplay();
    }
}

void initSimulation()
{
    glClearColor(0.15f, 0.15f, 0.15f, 1.0f);
    simState.reset();

    for (int i = 0; i < simState.targgetParticleCount; i++)
    {
        simState.particles.push_back(Particle(0.0f, 0.0f, 0.0f));
    }

    std::cout << "Particle System with Dynamic Pathfinding" << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << "Particles: " << simState.targgetParticleCount << std::endl;
    std::cout << "Box size: " << boxsize << "x" << boxsize << "x" << boxsize << std::endl;
    std::cout << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  W/S - Rotate camera up/down" << std::endl;
    std::cout << "  A/D - Rotate camera left/right" << std::endl;
    std::cout << "  +/- - Zoom in/out" << std::endl;
    std::cout << "  R   - Reset simulation" << std::endl;
    std::cout << "  Space - Agitate particles" << std::endl;
    std::cout << "  Q/ESC - Quit" << std::endl;
    std::cout << std::endl;
    std::cout << "The green particle is the robot" << std::endl;
    std::cout << "The red particle is the goal" << std::endl;
    std::cout << "The magenta line is the computed path" << std::endl;
    std::cout << std::endl;
}

int main(int argc, char **argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
    glutCreateWindow("Particle System with Dynamic Pathfinding");

    glewInit();

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
    glClearColor(0.15f, 0.15f, 0.15f, 1.0f);

    setupLighting();

    initSimulation();

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutIdleFunc(idle);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);

    glutMainLoop();

    return 0;
}
