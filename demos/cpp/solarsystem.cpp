#include <iostream>
#include <cmath>
#include <GL/glut.h>
#include <btBulletDynamicsCommon.h>

// Constants
const float G = 6.67430e-11; // Gravitational constant (m^3/kg/s^2)
const float M_SUN = 1.989e30; // Mass of the Sun (kg)
const float M_EARTH = 5.972e24; // Mass of the Earth (kg)
const float R_EARTH_ORBIT = 1.496e11; // Mean distance from the Sun to Earth (m)

// Global variables
float earthPosition[3] = {0.0f, 0.0f, 0.0f};
float earthVelocity[3] = {0.0f, 0.0f, 29783.0f}; // Initial velocity for Earth (m/s)
btRigidBody* earthRigidBody;

// Function to update Earth's position based on gravitational force
void updateEarthPosition() {
    float r = sqrt(earthPosition[0] * earthPosition[0] +
                   earthPosition[1] * earthPosition[1] +
                   earthPosition[2] * earthPosition[2]);
    float F = (G * M_SUN * M_EARTH) / (r * r);

    for (int i = 0; i < 3; ++i) {
        float direction = -earthPosition[i] / r;
        float acceleration = F / M_EARTH;
        earthVelocity[i] += acceleration * direction;
        earthPosition[i] += earthVelocity[i];
    }
}

// Function to update the simulation
void updateSimulation(int value) {
    updateEarthPosition();
    earthRigidBody->getWorldTransform().setOrigin(btVector3(earthPosition[0], earthPosition[1], earthPosition[2]));
    glutPostRedisplay();
    glutTimerFunc(16, updateSimulation, 0); // 60 FPS
}

// Function to draw the Earth
void drawEarth() {
    glPushMatrix();
    glColor3f(0.0f, 0.3f, 1.0f); // Blue color for Earth
    glTranslatef(earthPosition[0], earthPosition[1], earthPosition[2]);
    glutSolidSphere(6371000.0, 50, 50); // Earth radius in meters
    glPopMatrix();
}

// Function to display the scene
void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    glTranslatef(0.0f, 0.0f, -3.0f);

    drawEarth();

    glutSwapBuffers();
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Solar System Simulation");

    // Initialize Bullet Physics
    btBroadphaseInterface* broadphase = new btDbvtBroadphase();
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
    btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
    dynamicsWorld->setGravity(btVector3(0.0f, 0.0f, 0.0f));

    // Create Earth's rigid body
    btTransform earthTransform;
    earthTransform.setIdentity();
    earthTransform.setOrigin(btVector3(earthPosition[0], earthPosition[1], earthPosition[2]));
    btSphereShape* earthShape = new btSphereShape(6371000.0);
    btVector3 inertia(0, 0, 0);
    earthShape->calculateLocalInertia(M_EARTH, inertia);
    btDefaultMotionState* earthMotionState = new btDefaultMotionState(earthTransform);
    btRigidBody::btRigidBodyConstructionInfo earthRigidBodyCI(M_EARTH, earthMotionState, earthShape, inertia);
    earthRigidBody = new btRigidBody(earthRigidBodyCI);
    dynamicsWorld->addRigidBody(earthRigidBody);

    glutDisplayFunc(display);
    glutTimerFunc(0, updateSimulation, 0);

    glutMainLoop();

    delete dynamicsWorld;
    delete solver;
    delete dispatcher;
    delete collisionConfiguration;
    delete broadphase;
    delete earthRigidBody;
    delete earthShape;

    return 0;
}
