#include <GL/glut.h>

// Function to initialize OpenGL
void init() {
    glClearColor(0.0, 0.0, 0.0, 1.0);
}

// Function to render the scene
void display() {
    glClear(GL_COLOR_BUFFER_BIT);
    
    // Render your 3D objects here

    glutSwapBuffers();
}

// Function to handle window reshape
void reshape(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (float)w / (float)h, 1.0, 100.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Solar System Simulation");

    init();
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);

    glutMainLoop();
    
    return 0;
}

