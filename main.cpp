//Nuengruethai Wutthisak 6088138
//Kanlayakorn Kesorn 6088057
//Thanakorn Pasangthien 6088109
//|___________________________________________________________________
//!
//! \file plane2_base.cpp
//!
//! \brief Base source code for the second plane assignment.
//!
//! Author: Mores Prachyabrued.
//!
//! Keyboard inputs for plane and propeller (subpart):
//!   s   = moves the plane forward
//!   f   = moves the plane backward
//!   q,e = rolls the plane
//!   a   = yaws the plane
//!   x   = pitches the plane
//!
//!   r   = rotates propeller
//!
//! Mouse inputs for world-relative camera:
//!   Hold left button and drag  = controls azimuth and elevation
//!                                (Press CTRL (and hold) before left button to restrict to azimuth control only,
//!                                 Press SHIFT (and hold) before left button to restrict to elevation control only)
//!   Hold right button and drag = controls distance
//!
//! TODO: Extend the code to satisfy the requirements given in the assignment handout
//!
//! Note: Good programmer uses good comments! :)
//|___________________________________________________________________

//|___________________
//|
//| Includes
//|___________________

#include <math.h>

#include <gmtl/gmtl.h>

#include <GL/glut.h>

//|___________________
//|
//| Constants
//|___________________
#define DEF_D 5

// Plane dimensions
const float P_WIDTH        = 3;
const float P_LENGTH       = 3;
const float P_HEIGHT       = 1.5f;

// Plane transforms
const gmtl::Vec3f PLANE_FORWARD(0, 0, 1.0f);            // Plane's forward translation vector (w.r.t. local frame)
const float PLANE_ROTATION = 5.0f;                      // Plane rotated by 5 degs per input

//dimensions (subpart)
const float PP_WIDTH       = 1.5f;
const float PP_LENGTH      = 1.5f;
const float PP_HEIGHT      = 1.5f;

// Propeller transforms
const gmtl::Point3f PROPELLER_POS(P_WIDTH/4, 0, 0);     // Propeller position on the plane (w.r.t. plane's frame)
const float PROPELLER_ROTATION = 5.0f;                  // Propeller rotated by 5 degs per input

// Camera's view frustum
const float CAM_FOV        = 90.0f;                     // Field of view in degs

// Keyboard modifiers
enum KeyModifier {KM_SHIFT = 0, KM_CTRL, KM_ALT};

//|___________________
//|
//| Global Variables
//|___________________

// Track window dimensions, initialized to 800x600
int w_width    = 800;
int w_height   = 600;


// Plane pose (position-quaternion pair)
gmtl::Point4f plane_p;      // Position (using explicit homogeneous form; see Quaternion example code)
gmtl::Quatf plane_q;        // Quaternion

gmtl::Point4f plane_p1;      // Position 1 (using explicit homogeneous form; see Quaternion example code)
gmtl::Quatf plane_q1;        // Quaternion 1

// Quaternions to rotate plane
gmtl::Quatf zrotp_q;        // Positive and negative Z rotations
gmtl::Quatf zrotn_q;

gmtl::Quatf zrotp_q1;        // Positive and negative Z rotations1
gmtl::Quatf zrotn_q1;

gmtl::Quatf xrotp_q;
gmtl::Quatf xrotp_q1;

gmtl::Quatf yrotp_q;
gmtl::Quatf yrotp_q1;

// Propeller rotation (subpart)
float pp_angle   = 0;         // Rotation angle for subpart wing1
float pp_angle_2 = 0;         // Rotation angle for subpart wing2
float pp_angle_3 = 0;         // Rotation angle for subpart wing3
float gun_angle = 0;          // Rotation angle for sub subpart gun
// Mouse & keyboard
int mx_prev = 0, my_prev = 0;
bool mbuttons[3]   = {false, false, false};
bool kmodifiers[3] = {false, false, false};

// Cameras
int cam_id         = 0;                                // Selects which camera to view
int camctrl_id     = 0;                                // Selects which camera to control
float distance[3]  = { 20.0f,  20.0f, 20.0f};                 // Distance of the camera from world's origin.
float elevation[3] = {-45.0f, -45.0f, -45.0f};                 // Elevation of the camera. (in degs)
float azimuth[3]   = { 15.0f,  15.0f, 15.0f};                 // Azimuth of the camera. (in degs)

//|___________________
//|
//| Function Prototypes
//|___________________

void InitTransforms();
void InitGL(void);
void DisplayFunc(void);
void KeyboardFunc(unsigned char key, int x, int y);
void MouseFunc(int button, int state, int x, int y);
void MotionFunc(int x, int y);
void ReshapeFunc(int w, int h);
void DrawCoordinateFrame(const float l);
void DrawPlaneBody(const float width, const float length, const float height);
void DrawPropeller(const float width, const float length);
void DrawWingOne(const float width, const float length, const float height);
void DrawWingTwo(const float width, const float length, const float height);
void DrawWingThree(const float width, const float length, const float height);
void DrawGun(const float width, const float length,const float hight);
void DrawPlaneBody2(const float width, const float length, const float height);

//|____________________________________________________________________
//|
//| Function: InitTransforms
//|
//! \param None.
//! \return None.
//!
//! Initializes all the transforms
//|____________________________________________________________________

void InitTransforms()
{
  const float COSTHETA_D2  = cos(gmtl::Math::deg2Rad(PLANE_ROTATION/2));  // cos() and sin() expect radians
  const float SINTHETA_D2  = sin(gmtl::Math::deg2Rad(PLANE_ROTATION/2));

  // Inits plane pose
  plane_p.set(3.0f, 0.0f, 4.0f, 1.0f);
  plane_q.set(0, 0, 0, 1);
    
  plane_p1.set(-3.0f, 0.0f, 4.0f, 1.0f);
  plane_q1.set(0, 0, 0, 1);

  // Z rotations (roll)
  zrotp_q.set(0, 0, SINTHETA_D2, COSTHETA_D2);      // +Z
  zrotn_q = gmtl::makeConj(zrotp_q);                // -Z

  // Z rotations (roll)
   zrotp_q1.set(0, 0, SINTHETA_D2, COSTHETA_D2);      // +Z
   zrotn_q1 = gmtl::makeConj(zrotp_q1);                // -Z
    

  // X rotation (pitch)
  xrotp_q.set(SINTHETA_D2, 0, 0, COSTHETA_D2);      // +X
  // X rotation (pitch)
  xrotp_q1.set(SINTHETA_D2, 0, 0, COSTHETA_D2);      // +X

  // Y rotation (yaw)
  yrotp_q.set(0, SINTHETA_D2, 0, COSTHETA_D2);      // +Y
  // Y rotation (yaw)
  yrotp_q1.set(0, SINTHETA_D2, 0, COSTHETA_D2);      // +Y
  
  
  // TODO: Initializes the remaining transforms
}

//|____________________________________________________________________
//|
//| Function: InitGL
//|
//! \param None.
//! \return None.
//!
//! OpenGL initializations
//|____________________________________________________________________

void InitGL(void)
{
  glClearColor(0.7f, 0.7f, 0.7f, 1.0f);
  glEnable(GL_DEPTH_TEST);
  glShadeModel(GL_SMOOTH);
}

//|____________________________________________________________________
//|
//| Function: DisplayFunc
//|
//! \param None.
//! \return None.
//!
//! GLUT display callback function: called for every redraw event.
//|____________________________________________________________________

void DisplayFunc(void)
{
  gmtl::AxisAnglef aa;    // Converts plane's quaternion to axis-angle form to be used by glRotatef()
  gmtl::Vec3f axis;       // Axis component of axis-angle representation
  float angle;            // Angle component of axis-angle representation

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(CAM_FOV, (float)w_width/w_height, 0.1f, 1000.0f);     // Check MSDN: google "gluPerspective msdn"

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

//|____________________________________________________________________
//|
//| Setting up view transform by:
//| "move up to the world frame by composing all of the (inverse) transforms from the camera up to the world node"
//|____________________________________________________________________

  switch (cam_id) {
    case 0:
      // For the world-relative camera
      glTranslatef(0, 0, -distance[0]);
      glRotatef(-elevation[0], 1, 0, 0);
      glRotatef(-azimuth[0], 0, 1, 0);
      break;

    case 1:
      // For plane2's camera
      glTranslatef(0, 0, -distance[1]);
      glRotatef(-elevation[1], 1, 0, 0);
      glRotatef(-azimuth[1], 0, 1, 0);

      gmtl::set(aa, plane_q);                    // Converts plane's quaternion to axis-angle form to be used by glRotatef()
      axis  = aa.getAxis();
      angle = aa.getAngle();
      glRotatef(-gmtl::Math::rad2Deg(angle), axis[0], axis[1], axis[2]);
      glTranslatef(-plane_p[0], -plane_p[1], -plane_p[2]);
      break;

      // TODO: Add case for the plane1's camera
          
    case 2:
    // For plane2's camera
    glTranslatef(0, 0, -distance[2]);
    glRotatef(-elevation[2], 1, 0, 0);
    glRotatef(-azimuth[2], 0, 1, 0);

    gmtl::set(aa, plane_q1);                    // Converts plane's quaternion to axis-angle form to be used by glRotatef()
    axis  = aa.getAxis();
    angle = aa.getAngle();
    glRotatef(-gmtl::Math::rad2Deg(angle), axis[0], axis[1], axis[2]);
    glTranslatef(-plane_p1[0], -plane_p1[1], -plane_p1[2]);
    break;
  }

//|____________________________________________________________________
//|
//| Draw traversal begins, start from world (root) node
//|____________________________________________________________________

  // World node: draws world coordinate frame
  DrawCoordinateFrame(10);

  // World-relative camera:
  if (cam_id != 0) {
    glPushMatrix();
      glRotatef(azimuth[0], 0, 1, 0);
      glRotatef(elevation[0], 1, 0, 0);
      glTranslatef(0, 0, distance[0]);
      DrawCoordinateFrame(1);
    glPopMatrix();
  }

  // Plane 2 body:
  glPushMatrix();
    gmtl::set(aa, plane_q);     // Converts plane's quaternion to axis-angle form to be used by glRotatef()
    axis  = aa.getAxis();
    angle = aa.getAngle();
    glTranslatef(plane_p[0], plane_p[1], plane_p[2]);
    glRotatef(gmtl::Math::rad2Deg(angle), axis[0], axis[1], axis[2]);
    DrawPlaneBody(P_WIDTH, P_LENGTH, P_HEIGHT);
    DrawCoordinateFrame(3);

    // Plane 2's camera:
    glPushMatrix();
      glRotatef(azimuth[1], 0, 1, 0);
      glRotatef(elevation[1], 1, 0, 0);
      glTranslatef(0, 0, distance[1]);
      DrawCoordinateFrame(1);
    glPopMatrix();
    
    //wing1
    glPushMatrix();
          glTranslatef(1, 0, -P_LENGTH);
            glRotatef(pp_angle, 0, 0, 1);                                           // Rotates propeller
            DrawWingOne(PP_WIDTH, PP_LENGTH, PP_HEIGHT);
            DrawCoordinateFrame(1);
          glPushMatrix();
            glTranslated(PP_WIDTH, 0, 0);
            glRotatef(gun_angle, 0, 1, 0);
            DrawGun(0.5, 1, 0);
            DrawCoordinateFrame(1);
        glPopMatrix();
    glPopMatrix();
    
    //wing2
    glPushMatrix();
          glTranslatef(-1,0, -P_LENGTH);
            glRotatef(pp_angle_2, 0, 0, 1);                                           // Rotates propeller
            DrawWingTwo(PP_WIDTH, PP_LENGTH, PP_HEIGHT);
            DrawCoordinateFrame(1);
    glPopMatrix();

    //wing3
    glPushMatrix();
          glTranslatef(0, P_HEIGHT/2, -P_LENGTH);
            glRotatef(pp_angle_3, 0, 1, 0);                                           // Rotates propeller
            DrawWingThree(PP_WIDTH, PP_LENGTH, PP_HEIGHT);
            DrawCoordinateFrame(1);
    glPopMatrix();
    
  glPopMatrix();
    
    // Plane 1 body:
    glPushMatrix();
      gmtl::set(aa, plane_q1);                    // Converts plane's quaternion to axis-angle form to be used by glRotatef()
      axis  = aa.getAxis();
      angle = aa.getAngle();
      glTranslatef(plane_p1[0], plane_p1[1], plane_p1[2]);
      glRotatef(gmtl::Math::rad2Deg(angle), axis[0], axis[1], axis[2]);
      DrawPlaneBody2(P_WIDTH, P_LENGTH, P_HEIGHT);
      DrawCoordinateFrame(3);


      // Plane 1's camera:
      glPushMatrix();
        glRotatef(azimuth[2], 0, 1, 0);
        glRotatef(elevation[2], 1, 0, 0);
        glTranslatef(0, 0, distance[2]);
        DrawCoordinateFrame(1);
      glPopMatrix();
    glPopMatrix();
  glutSwapBuffers();                          // Replaces glFlush() to use double buffering
}

//|____________________________________________________________________
//|
//| Function: KeyboardFunc
//|
//! \param key    [in] Key code.
//! \param x      [in] X-coordinate of mouse when key is pressed.
//! \param y      [in] Y-coordinate of mouse when key is pressed.
//! \return None.
//!
//! GLUT keyboard callback function: called for every key press event.
//|____________________________________________________________________

void KeyboardFunc(unsigned char key, int x, int y)
{
  switch (key) {
//|____________________________________________________________________
//|
//| Camera switch
//|____________________________________________________________________

    case 'v': // Select camera to view
      cam_id = (cam_id + 1) % 3;
      printf("View camera = %d\n", cam_id);
      break;
    case 'b': // Select camera to control
      camctrl_id = (camctrl_id + 1) % 3;
      printf("Control camera = %d\n", camctrl_id);
      break;

//|____________________________________________________________________
//|
//| Plane controls
//|____________________________________________________________________

    case 's': { // Forward translation of the plane (+Z translation)
      gmtl::Quatf v_q = plane_q * gmtl::Quatf(PLANE_FORWARD[0], PLANE_FORWARD[1], PLANE_FORWARD[2], 0) * gmtl::makeConj(plane_q);
      plane_p         = plane_p + v_q.mData;
      } break;
    case 'f': { // Backward translation of the plane (-Z translation)
      gmtl::Quatf v_q = plane_q * gmtl::Quatf(-PLANE_FORWARD[0], -PLANE_FORWARD[1], -PLANE_FORWARD[2], 0) * gmtl::makeConj(plane_q);
      plane_p         = plane_p + v_q.mData;
      } break;

    case 'e': // Rolls the plane (+Z rot)
      plane_q = plane_q * zrotp_q;
      break;
    case 'q': // Rolls the plane (-Z rot)
      plane_q = plane_q * zrotn_q;
      break;

    case 'x': // Pitches the plane (+X rot)
      plane_q = plane_q * xrotp_q;
      break;
    case 'a': // Yaws the plane (+Y rot)
      plane_q = plane_q * yrotp_q;
      break;

//|____________________________________________________________________
//|
//| Propeller controls (subpart)
//|____________________________________________________________________

    case 'r': // Rotates propeller subpart wing 1
      pp_angle   += PROPELLER_ROTATION;
      break;
    case 't': // Rotates propeller subpart wing 2
      pp_angle_2 += PROPELLER_ROTATION;
      break;
    case 'h': // Rotates propeller subpart wing 3
        pp_angle_3 += PROPELLER_ROTATION;
        break;
    case 'g': // Rotates propeller subpart Gun
        gun_angle += PROPELLER_ROTATION;
        break;


    // TODO: Add the remaining controls/transforms plane 1
          case 'j': { // Forward translation of the plane (+Z translation)
              gmtl::Quatf v_q1 = plane_q1 * gmtl::Quatf(PLANE_FORWARD[0], PLANE_FORWARD[1], PLANE_FORWARD[2], 0) * gmtl::makeConj(plane_q1);
              plane_p1    = plane_p1 + v_q1.mData;
            } break;
          case 'k': { // Backward translation of the plane (-Z translation)
            gmtl::Quatf v_q1 = plane_q1 * gmtl::Quatf(-PLANE_FORWARD[0], -PLANE_FORWARD[1], -PLANE_FORWARD[2], 0) * gmtl::makeConj(plane_q1);
            plane_p1         = plane_p1 + v_q1.mData;
            } break;

          case 'l': // Rolls the plane (+Z rot)
            plane_q1 = plane_q1 * zrotp_q1;
            break;
          case 'u': // Rolls the plane (-Z rot)
            plane_q1 = plane_q1 * zrotn_q1;
            break;

          case 'i': // Pitches the plane (+X rot)
            plane_q1 = plane_q1 * xrotp_q1;
            break;
          case 'o': // Yaws the plane (+Y rot)
            plane_q1 = plane_q1 * yrotp_q1;
            break;
          
  }

  glutPostRedisplay();                    // Asks GLUT to redraw the screen
}

//|____________________________________________________________________
//|
//| Function: MouseFunc
//|
//! \param button     [in] one of GLUT_LEFT_BUTTON, GLUT_MIDDLE_BUTTON, or GLUT_RIGHT_BUTTON.
//! \param state      [in] one of GLUT_UP (event is due to release) or GLUT_DOWN (press).
//! \param x          [in] X-coordinate of mouse when an event occured.
//! \param y          [in] Y-coordinate of mouse when an event occured.
//! \return None.
//!
//! GLUT mouse-callback function: called for each mouse click.
//|____________________________________________________________________

void MouseFunc(int button, int state, int x, int y)
{
  int km_state;

  // Updates button's sate and mouse coordinates
  if (state == GLUT_DOWN) {
    mbuttons[button] = true;
    mx_prev          = x;
    my_prev          = y;
  } else {
    mbuttons[button] = false;
  }

  // Updates keyboard modifiers
  km_state = glutGetModifiers();
  kmodifiers[KM_SHIFT] = km_state & GLUT_ACTIVE_SHIFT ? true : false;
  kmodifiers[KM_CTRL]  = km_state & GLUT_ACTIVE_CTRL  ? true : false;
  kmodifiers[KM_ALT]   = km_state & GLUT_ACTIVE_ALT   ? true : false;

  //glutPostRedisplay();      // Asks GLUT to redraw the screen
}

//|____________________________________________________________________
//|
//| Function: MotionFunc
//|
//! \param x      [in] X-coordinate of mouse when an event occured.
//! \param y      [in] Y-coordinate of mouse when an event occured.
//! \return None.
//!
//! GLUT motion-callback function: called for each mouse motion.
//|____________________________________________________________________

void MotionFunc(int x, int y)
{
  int dx, dy, d;

  if (mbuttons[GLUT_LEFT_BUTTON] || mbuttons[GLUT_RIGHT_BUTTON]) {
    // Computes distances the mouse has moved
    dx      = x - mx_prev;
    dy      = y - my_prev;

    // Updates mouse coordinates
    mx_prev = x;
    my_prev = y;

    // Hold left button to rotate camera
    if (mbuttons[GLUT_LEFT_BUTTON]) {
      if (!kmodifiers[KM_CTRL]) {
        elevation[camctrl_id] += dy;            // Elevation update
      }
      if (!kmodifiers[KM_SHIFT]) {
        azimuth[camctrl_id] += dx;             // Azimuth update
      }
    }

    // Hold right button to zoom
    if (mbuttons[GLUT_RIGHT_BUTTON]) {
      if (abs(dx) >= abs(dy)) {
        d = dx;
      } else {
        d = -dy;
      }
      distance[camctrl_id] += d;
    }

    glutPostRedisplay();      // Asks GLUT to redraw the screen
  }
}

//|____________________________________________________________________
//|
//| Function: ReshapeFunc
//|
//! \param None.
//! \return None.
//!
//! GLUT reshape callback function: called everytime the window is resized.
//|____________________________________________________________________

void ReshapeFunc(int w, int h)
{
  // Track the current window dimensions
  w_width  = w;
  w_height = h;
  glViewport(0, 0, (GLsizei) w_width, (GLsizei) w_height);
}

//|____________________________________________________________________
//|
//| Function: DrawCoordinateFrame
//|
//! \param l      [in] length of the three axes.
//! \return None.
//!
//! Draws coordinate frame consisting of the three principal axes.
//|____________________________________________________________________

void DrawCoordinateFrame(const float l)
{
  glBegin(GL_LINES);
    // X axis is red
    glColor3f( 1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
      glVertex3f(   l, 0.0f, 0.0f);

    // Y axis is green
    glColor3f( 0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
      glVertex3f(0.0f,    l, 0.0f);

    // Z axis is blue
    glColor3f( 0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
      glVertex3f(0.0f, 0.0f,    l);
  glEnd();
}

//|____________________________________________________________________
//|
//| Function: DrawPlaneBody
//|
//! \param width       [in] Width  of the plane.
//! \param length      [in] Length of the plane.
//! \param height      [in] Height of the plane.
//! \return None.
//!
//! Draws a plane body.
//|____________________________________________________________________

void DrawPlaneBody(const float width, const float length, const float height)
{
  float h = height / 2;
    
       GLfloat x              = 0.0;
       GLfloat y              = 0.0;
       GLfloat angle          = 0.0;
       GLfloat angle_stepsize = 0.1;
       GLfloat radius = 1;

        glColor3ub(255, 175, 175);
    //body
        glBegin(GL_QUAD_STRIP);
        angle = 0.0;
    while( angle < 2*gmtl::Math::PI ) {
                x = radius * cos(angle);
                y = radius * sin(angle);
                glVertex3f(x, y , h-4);
                glVertex3f(x, y , 0.0);
                angle = angle + angle_stepsize;
            }
            glVertex3f(radius, 0.0, h);
            glVertex3f(radius, 0.0, 0.0);
        glEnd();

  glColor3ub(255, 175, 175);
     glBegin(GL_POLYGON);
     angle = 0.0;
    while( angle < 2*gmtl::Math::PI ) {
             x = radius * cos(angle);
             y = radius * sin(angle);
             glVertex3f(x, y , h-4);
             angle = angle + angle_stepsize;
         }
         glVertex3f(radius, 0.0, h);
     glEnd();
    
    
    //front
    int k;
    glBegin(GL_TRIANGLES);
    for (k=0;k<=360;k+=DEF_D){
      glColor3f(0.0,0.0,1.0);
      glVertex3f(0,0,1);
      glVertex3f(cos(k),sin(k),0);
      glVertex3f(cos(k+DEF_D),sin(k+DEF_D),0);
    }
    glEnd();
}

void DrawPlaneBody2(const float width, const float length, const float height) {
    float w = width/2;
    float l = length/2;
    float h = height / 2;
    
    glBegin(GL_TRIANGLES);
       // Wings are red
       glColor3f( 1.0f, 0.0f, 0.0f);
      
       //left wing
       glVertex3f(w-1 , h, l - 2);
       glVertex3f(w-1, h, l-4);
       glVertex3f(w, h, l-4);
      
      // right wing
      glVertex3f(-w+1, h, l - 2);
      glVertex3f(-w+1, h, l - 4);
      glVertex3f(-w, h, l - 4);
      
      //fin
      glVertex3f(0, h, l - 3);
      glVertex3f(0, h, l - 4.5);
      glVertex3f(0, h+1, l - 4.5);
    glEnd();
      
         GLfloat x              = 0.0;
         GLfloat y              = 0.0;
         GLfloat angle          = 0.0;
         GLfloat angle_stepsize = 0.1;
         GLfloat radius = 1;

          glColor3ub(255, 175, 175);
      //body
          glBegin(GL_QUAD_STRIP);
          angle = 0.0;
      while( angle < 2*gmtl::Math::PI ) {
                  x = radius * cos(angle);
                  y = radius * sin(angle);
                  glVertex3f(x, y , h-4);
                  glVertex3f(x, y , 0.0);
                  angle = angle + angle_stepsize;
              }
              glVertex3f(radius, 0.0, h);
              glVertex3f(radius, 0.0, 0.0);
          glEnd();

    glColor3ub(255, 175, 175);
       glBegin(GL_POLYGON);
       angle = 0.0;
      while( angle < 2*gmtl::Math::PI ) {
               x = radius * cos(angle);
               y = radius * sin(angle);
               glVertex3f(x, y , h-4);
               angle = angle + angle_stepsize;
           }
           glVertex3f(radius, 0.0, h);
       glEnd();
      
      
      //front
      int k;
      glBegin(GL_TRIANGLES);
      for (k=0;k<=360;k+=DEF_D){
        glColor3f(0.0,0.0,1.0);
        glVertex3f(0,0,1);
        glVertex3f(cos(k),sin(k),0);
        glVertex3f(cos(k+DEF_D),sin(k+DEF_D),0);
      }
      glEnd();
}

void DrawWingOne(const float width, const float length, const float hight)
{
  glBegin(GL_TRIANGLES);
    glColor3f( 1.0f, 0.0f, 0.0f);
        glVertex3f(width , 0, 0);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, length);
  glEnd();
}

void DrawWingTwo(const float width, const float length, const float hight)
{
  glBegin(GL_TRIANGLES);
    glColor3f( 1.0f, 0.0f, 0.0f);
    glVertex3f(0, 0, length);
    glVertex3f(0, 0, 0);
    glVertex3f(-width, 0, 0);
  glEnd();
}

void DrawWingThree(const float width, const float length,const float height)
{
  glBegin(GL_TRIANGLES);
    glColor3f( 1.0f, 0.0f, 0.0f);
    glVertex3f(0, 0, length);
    glVertex3f(0, 0, 0);
    glVertex3f(0, height, 0);
  glEnd();
}

void DrawGun(const float width, const float length,const float height)
{
  float w = width/2;
  
  glBegin(GL_QUADS);
    glColor3f( 0.0f, 1.0f, 0.0f);
    glVertex3f(w, 0, length);
    glVertex3f(w, 0, 0);
    glVertex3f(-w, 0, 0);
    glVertex3f(-w, 0, length);
  glEnd();
}

//|____________________________________________________________________
//|
//| Function: main
//|
//! \param None.
//! \return None.
//!
//! Program entry point
//|____________________________________________________________________

int main(int argc, char **argv)
{
  InitTransforms();

  glutInit(&argc, argv);

  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);     // Uses GLUT_DOUBLE to enable double buffering
  glutInitWindowSize(w_width, w_height);
  
  glutCreateWindow("Plane Episode 2");

  glutDisplayFunc(DisplayFunc);
  glutKeyboardFunc(KeyboardFunc);
  glutMouseFunc(MouseFunc);
  glutMotionFunc(MotionFunc);
  glutReshapeFunc(ReshapeFunc);
  
  InitGL();

  glutMainLoop();

  return 0;
}
