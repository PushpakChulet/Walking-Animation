#include "stdafx.h"

// standard
#include <assert.h>
#include <math.h>

// glut
#include <GL/glut.h>

//================================
// global variables
//================================
// screen size
int g_screenWidth  = 0;
int g_screenHeight = 0;
GLfloat t = 0.0f;
GLfloat dt = 0.01f;

// frame index
int g_frameIndex = 0;

// angle for rotation
int g_angle = 0;


GLint P = 0 ;
GLint N = 7;

// it will be used to define oriantation
GLfloat tangent[3] =	{ 0.0f, 0.0f, 0.0f };
GLfloat binorm[3] = { 1.0f, 0.0f, 0.0f };
GLfloat norm[3] =	{ 0.0f, 0.0f, 0.0f };
GLfloat loopIndex = 0;

GLfloat M[16] = { 0 }; //torso
GLfloat tempM[3] = {0};//temporate matrix to store the interpolation track (torso position)
GLfloat M1[16] = { 0 };//Left Leg
GLfloat M2[16] = { 0 }; // Right Leg

//this is the blend function which will give Q(t)
GLfloat funcQT(GLfloat T[4], GLfloat mati[16], GLfloat controlpoints[4])
{
	GLfloat tempres[4] = { 0 };
	GLfloat Qt = 0;

	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			tempres[i] += T[j] * mati[4 * i + j];

	// Calcualte Qt
	for (int i = 0; i < 4; i++)
		Qt += tempres[i] * controlpoints[i];

	return Qt;
}


GLfloat temp[16] = {
	0, 0, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0
};

// The Catmul-Rom Spline M Marix
GLfloat Mcat[16] = {
	-0.5f,	1.0f,	-0.5f,	0.0f,
	1.5f,	-2.5f,	0.0f,	1.0f,
	-1.5f,	2.0f,	0.5f,	0.0f,
	0.5f,	-0.5f,	0.0f,	0.0f
};
// The B Spline M Marix
GLfloat MBspline[16] = {
	-1.0/ 6.0,	3.0f / 6.0f,	-3.0f / 6.0f,	1.0f / 6.0f,
	3.0f / 6.0f,	-6.0f / 6.0f,	0.0f / 6.0f,	4.0f / 6.0f,
	-3.0f / 6.0f,	3.0f / 6.0f,	3.0f / 6.0f,	1.0f / 6.0f,
	1.0f / 6.0f,	0.0f / 6.0f,	0.0f / 6.0f,	0.0f / 6.0f
};
//7 cartisian system points
GLfloat point_position[7][3] = { { 8, 0, -20 },
                                { -8, 0, -20 },
                                { -5, 0, -10 },
                                { 5, 0, -10 },
                                { 3, 0, -5 },
                                {-3,0,-5},
                                { 1, 0, -3 } };


//vector multiplication
void vec_cross(GLfloat TempV1[3], GLfloat TempV2[3], GLfloat VResult[3])
{
	for(int i=0; i<3; ++i)
    {
        VResult[i] = TempV1[(i+1)%3]*TempV2[(i+2)%3] - TempV1[(i+2)%3]*TempV2[(i+1)%3];
    }
}
//the function will multiply 4*4 matrix
void matMult4(GLfloat Temp1[16], GLfloat Temp2[16], GLfloat Result[16])

{
	for(int i =0; i<4; ++i)
    {
        for(int j =0; j<4; ++j)
        {
            Result[4*i + j] = Temp1[j]*Temp2[4*i] + Temp1[4+j]*Temp2[4*i+1] + Temp1[8+j]*Temp2[4*i+2] + Temp1[12+j]*Temp2[4*i+3];
        }
    }
}

//use to normalize which converts a vector to unit vector
void quatToVect(GLfloat quat[7])
{
    GLfloat squa_quaterion = quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2];
	if (squa_quaterion != 0) // avoid being divided by 0
	{
		GLfloat base_quaternion = sqrt(squa_quaterion);
		quat[0] = quat[0] / base_quaternion;
		quat[1] = quat[1] / base_quaternion;
		quat[2] = quat[2] / base_quaternion;
	}
}

void Torso_interpolate(GLfloat p_position[7][3], GLfloat SplineM[16])
{
	GLfloat TMatrix[4] = { t*t*t, t*t, t, 1 };

	GLfloat TangentTMatrix[4] = { 3*t*t, 2*t, 1, 0 };

	// Loop to generate the position interpolation track based on 4 points every time
	for (int i = 0; i < 3; i++)
	{
		GLfloat GMatrix[4] = { p_position[P][i],
			p_position[(P + 1) % N][i],
			p_position[(P + 2) % N][i],
			p_position[(P + 3) % N][i] };

		tempM[i] = funcQT(TMatrix, SplineM, GMatrix);
		tangent[i] = funcQT(TangentTMatrix, SplineM, GMatrix);
	}

	quatToVect(tangent);

	if (P == 0 && loopIndex == 0)
	{
		GLfloat TempVector[3] = { 1, 0, 0 };
		quatToVect(TempVector);
		vec_cross(tangent, TempVector, norm);
		quatToVect(norm);
		vec_cross(norm, tangent, binorm);
		quatToVect(binorm);
		loopIndex++;
	}
	else
	{
		vec_cross(tangent, binorm, norm);
		quatToVect(norm);
		vec_cross(norm, tangent, binorm);
		quatToVect(binorm);
	}

	// Generate the Interpolation Matrix M
	for(int i =0; i<3; ++i)
    {
        M[4*i+0] = tangent[i];
        M[4*i+1] = norm[i];
        M[4*i+2] = binorm[i];
        M[4*i+3] = 0;
    }
    M[12] = tempM[0];
	M[13] = tempM[1];
	M[14] = tempM[2];
	M[15] = 1;

}
void TorsoAnim()
{
	Torso_interpolate(point_position, Mcat);
	glLoadMatrixf(M);
	glutSolidCube(1.0);
}
void LeftAnim()
{

	// First Translate Matrix
	GLfloat LT1[16] = { 1, 0, 0, 0,		
						0, 1, 0, 0,		
						0, 0, 1, 0,		
						0, -1, 0, 1 };	

	//Rotation Matrix by Z axix
	GLfloat LAngle = (sin(4 * 3.14*t - 3.14 / 2)*3.14) / 4; // To animate rotation, change ¦È(t)
	GLfloat LT2[16] = { cos(LAngle), sin(LAngle), 0, 0,	
						-sin(LAngle), cos(LAngle), 0, 0,
						0, 0, 1, 0,						
						0, 0, 0, 1 };					

	// Second Translate Matrix
	GLfloat LT3[16] = { 1, 0, 0, 0,		
						0, 1, 0, 0,		
						0, 0, 1, 0,		
						0, 0, 0.3, 1 };	

	// Transformation that describes B in A coordinate system: Tab = T3*T2*T1
	matMult4(M, LT2, M1);
	matMult4(M1, LT1, M1);
	matMult4(M1, LT3, M1);

	// Show the Left Leg
	glLoadMatrixf(M1);
	glScalef(0.3f, 2.0f, 0.3f);
	glutSolidCube(1.0);

}

void RightAnim()
{
	// First Translate Matrix
	GLfloat RT1[16] = { 1, 0, 0, 0,		
						0, 1, 0, 0,		
						0, 0, 1, 0,		
						0, -1, 0, 1 };	

	//Rotation Matrix by Z axis
	GLfloat LAngle = (sin(4*3.14*t-3.14/2)*3.14)/4; // To animate rotation, change ¦È(t)
	GLfloat RT2[16] = { cos(-LAngle), sin(-LAngle), 0, 0,	
						-sin(-LAngle), cos(-LAngle), 0, 0,
						0, 0, 1, 0,							
						0, 0, 0, 1 };						

	// Second Translate Matrix
	GLfloat RT3[16] = { 1, 0, 0, 0,		
						0, 1, 0, 0,		
						0, 0, 1, 0,		
						0, 0, -0.3, 1 };

	// Transformation that describes B in A coordinate system: Tab = T3*T2*T1
	matMult4(M, RT2, M2);
	matMult4(M2, RT1, M2);
	matMult4(M2, RT3, M2);

	// Show the right leg
	glLoadMatrixf(M2);
	glScalef(0.3f, 2.0f, 0.3f);
	glutSolidCube(1.0);
}



//================================
// init
//================================
void init( void ) {
	// init something before main loop...
}

//================================
// update
//================================
void update( void ) {
	// do something before rendering...

	// rotation angle
	g_angle = ( g_angle + 5 ) % 360;
}
//================================
// render
//================================
void render( void ) {
	// clear buffer
	glClearColor (0.0, 0.0, 0.0, 0.0);
	glClearDepth (1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// render state
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);

	// enable lighting
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	// light source attributes
	GLfloat LightAmbient[]	= { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat LightDiffuse[]	= { 0.3f, 0.3f, 0.3f, 1.0f };
	GLfloat LightSpecular[]	= { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat LightPosition[] = { 5.0f, 5.0f, 5.0f, 1.0f };

	glLightfv(GL_LIGHT0, GL_AMBIENT , LightAmbient );
	glLightfv(GL_LIGHT0, GL_DIFFUSE , LightDiffuse );
	glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);

	// surface material attributes
	GLfloat material_Ka[]	= { 0.11f, 0.06f, 0.11f, 1.0f };
	GLfloat material_Kd[]	= { 0.43f, 0.47f, 0.54f, 1.0f };
	GLfloat material_Ks[]	= { 0.33f, 0.33f, 0.52f, 1.0f };
	GLfloat material_Ke[]	= { 0.1f , 0.0f , 0.1f , 1.0f };
	GLfloat material_Se		= 10;

	glMaterialfv(GL_FRONT, GL_AMBIENT	, material_Ka);
	glMaterialfv(GL_FRONT, GL_DIFFUSE	, material_Kd);
	glMaterialfv(GL_FRONT, GL_SPECULAR	, material_Ks);
	glMaterialfv(GL_FRONT, GL_EMISSION	, material_Ke);
	glMaterialf (GL_FRONT, GL_SHININESS	, material_Se);

	// modelview matrix
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();

	//if you don't want to use keys then comment everything inside void keyboard function and uncomment to use a specific function
    TorsoAnim();
    LeftAnim();
    RightAnim();

	// render objects


	// disable lighting
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);

	// swap back and front buffers
	glutSwapBuffers();
}

//================================
// keyboard input
//================================
void keyboard( unsigned char key, int x, int y )
{
}

//================================
// reshape : update viewport and projection matrix when the window is resized
//================================
void reshape( int w, int h ) {
	// screen size
	g_screenWidth  = w;
	g_screenHeight = h;

	// viewport
	glViewport( 0, 0, (GLsizei)w, (GLsizei)h );

	// projection matrix
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	gluPerspective(45.0, (GLfloat)w/(GLfloat)h, 1.0, 2000.0);
}


//================================
// timer : triggered every 16ms ( about 60 frames per second )
//================================
void timer( int value )
{
	glutPostRedisplay();

	t += dt;
	if (t >= 1)
	{
		t = 0;
		if (P < N - 4) {
			P++;
		}
		else {
			P = 0;
		}
	}
	// reset timer
	glutTimerFunc(16, timer, 0);
}

struct Resolution {
	int w;
	int h;

};

//================================
// main
//================================
int main( int argc, char** argv ) {
	// create opengL window
	glutInit( &argc, argv );
	glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB |GLUT_DEPTH );
	glutInitWindowSize( 600, 600 );
	glutInitWindowPosition( 100, 100 );
	glutCreateWindow( argv[0] );

	// init
	init();

	// set callback functions
	glutDisplayFunc( render );
	glutReshapeFunc( reshape );
	glutKeyboardFunc( keyboard );
	glutTimerFunc( 16, timer, 0 );

	// main loop
	glutMainLoop();

	return 0;
}
