/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2011 Advanced Micro Devices, Inc.  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///original author: Erwin Coumans

///
/// OpenCL - OpenGL interop example. Updating transforms of many cubes on GPU, without going through main memory/using the PCIe bus
/// Create all OpenGL resources AFTER create OpenCL context!
///

#include <GL/glew.h>
#include <stdio.h>

#include "btGlutInclude.h"
#include "btStopwatch.h"


#include <assert.h>

#ifdef _WIN32
#include <windows.h>
#endif

//OpenCL stuff
#include "../basic_initialize/btOpenCLUtils.h"
#include "btOpenCLGLInteropBuffer.h"

cl_context			g_cxMainContext;
cl_command_queue	g_cqCommandQue;
cl_device_id		g_device;
static const size_t workGroupSize = 128;


btOpenCLGLInteropBuffer* g_interopBuffer = 0;
cl_kernel g_interopKernel;

bool useCPU = false;
bool printStats = true;
bool runOpenCLKernels = true;

#define MSTRINGIFY(A) #A
static char* interopKernelString =
#include "interopKernel.cl"

btStopwatch gStopwatch;
int m_glutScreenWidth = 640;
int m_glutScreenHeight= 480;

bool m_ortho = false;

static GLuint               instancingShader;        // The instancing renderer
static GLuint               square_vao;
static GLuint               square_vbo;
static GLuint               index_vbo;
static GLuint				m_texturehandle;

static bool                 done = false;
static GLint                angle_loc = 0;
static GLint ModelViewMatrix;
static GLint ProjectionMatrix;
static GLint NormalMatrix;

static GLint                uniform_texture_diffuse = 0;

#define MAX_SHADER_LENGTH   8192

static GLubyte shaderText[MAX_SHADER_LENGTH];

static const char* vertexShader= \
"#version 330\n"
"precision highp float;\n"
"\n"
"\n"
"\n"
"layout (location = 0) in vec4 position;\n"
"layout (location = 1) in vec4 instance_color;\n"
"layout (location = 2) in vec4 instance_position;\n"
"layout (location = 3) in vec4 instance_quaternion;\n"
"layout (location = 4) in vec2 uvcoords;\n"
"layout (location = 5) in vec3 vertexnormal;\n"
"\n"
"\n"
"uniform float angle = 0.0;\n"
"uniform mat4 ModelViewMatrix;\n"
"uniform mat4 ProjectionMatrix;\n"
"uniform mat3 NormalMatrix;\n"
"\n"
"out Fragment\n"
"{\n"
"    flat vec4 color;\n"
"} fragment;\n"
"\n"
"out Vert\n"
"{\n"
"	vec2 texcoord;\n"
"} vert;\n"
"\n"
"\n"
"vec4 quatMul ( in vec4 q1, in vec4 q2 )\n"
"{\n"
"    vec3  im = q1.w * q2.xyz + q1.xyz * q2.w + cross ( q1.xyz, q2.xyz );\n"
"    vec4  dt = q1 * q2;\n"
"    float re = dot ( dt, vec4 ( -1.0, -1.0, -1.0, 1.0 ) );\n"
"    return vec4 ( im, re );\n"
"}\n"
"\n"
"vec4 quatFromAxisAngle(vec4 axis, in float angle)\n"
"{\n"
"    float cah = cos(angle*0.5);\n"
"    float sah = sin(angle*0.5);\n"
"	float d = inversesqrt(dot(axis,axis));\n"
"	vec4 q = vec4(axis.x*sah*d,axis.y*sah*d,axis.z*sah*d,cah);\n"
"	return q;\n"
"}\n"
"//\n"
"// vector rotation via quaternion\n"
"//\n"
"vec4 quatRotate3 ( in vec3 p, in vec4 q )\n"
"{\n"
"    vec4 temp = quatMul ( q, vec4 ( p, 0.0 ) );\n"
"    return quatMul ( temp, vec4 ( -q.x, -q.y, -q.z, q.w ) );\n"
"}\n"
"vec4 quatRotate ( in vec4 p, in vec4 q )\n"
"{\n"
"    vec4 temp = quatMul ( q, p );\n"
"    return quatMul ( temp, vec4 ( -q.x, -q.y, -q.z, q.w ) );\n"
"}\n"
"\n"
"varying vec3 lightDir,normal,ambient;\n"
"\n"
"void main(void)\n"
"{\n"
"	vec4 q = instance_quaternion;\n"
"	ambient = vec3(0.2,0.2,0.2);\n"
"		\n"
"		\n"
"	vec4 local_normal = (quatRotate3( vertexnormal,q));\n"
"	vec3 light_pos = vec3(1000,1000,1000);\n"
"	normal = normalize(ModelViewMatrix * local_normal).xyz;\n"
"\n"
"	lightDir = normalize(light_pos);//gl_LightSource[0].position.xyz));\n"
"//	lightDir = normalize(vec3(gl_LightSource[0].position));\n"
"		\n"
"	vec4 axis = vec4(1,1,1,0);\n"
"	vec4 localcoord = quatRotate3( position.xyz,q);\n"
"	vec4 vertexPos = ProjectionMatrix * ModelViewMatrix *(instance_position+localcoord);\n"
"\n"
"	gl_Position = vertexPos;\n"
"	\n"
"//	fragment.color = instance_color;\n"
"	vert.texcoord = uvcoords;\n"
"}\n"
;


static const char* fragmentShader= \
"#version 330\n"
"precision highp float;\n"
"\n"
"in Fragment\n"
"{\n"
"    flat vec4 color;\n"
"} fragment;\n"
"\n"
"in Vert\n"
"{\n"
"	vec2 texcoord;\n"
"} vert;\n"
"\n"
"uniform sampler2D Diffuse;\n"
"uniform float diffuse_alpha;\n"
"\n"
"varying vec3 lightDir,normal,ambient;\n"
"\n"
"out vec4 color;\n"
"\n"
"void main_textured(void)\n"
"{\n"
"    color = texture2D(Diffuse,vert.texcoord);//fragment.color;\n"
"}\n"
"\n"
"void main(void)\n"
"{\n"
"    vec4 texel = texture2D(Diffuse,vert.texcoord);//fragment.color;\n"
"	vec3 ct,cf;\n"
"	float intensity,at,af;\n"
"	intensity = max(dot(lightDir,normalize(normal)),0.0);\n"
"	cf = intensity*vec3(1.0,1.0,1.0);//intensity * (gl_FrontMaterial.diffuse).rgb+ambient;//gl_FrontMaterial.ambient.rgb;\n"
"	af = diffuse_alpha;\n"
"		\n"
"	ct = texel.rgb;\n"
"	at = texel.a;\n"
"		\n"
"	gl_FragColor = vec4(ct * cf, at * af);	\n"
"//	color  = vec4(ct * cf, at * af);	\n"
"}\n"
;


// Load the shader from the source text
void gltLoadShaderSrc(const char *szShaderSrc, GLuint shader)
	{
    GLchar *fsStringPtr[1];

    fsStringPtr[0] = (GLchar *)szShaderSrc;
    glShaderSource(shader, 1, (const GLchar **)fsStringPtr, NULL);
	}


////////////////////////////////////////////////////////////////
// Load the shader from the specified file. Returns false if the
// shader could not be loaded
bool gltLoadShaderFile(const char *szFile, GLuint shader)
	{
    GLint shaderLength = 0;
    FILE *fp;
	
    // Open the shader file
    fp = fopen(szFile, "r");
    if(fp != NULL)
		{
        // See how long the file is
        while (fgetc(fp) != EOF)
            shaderLength++;
		
        // Allocate a block of memory to send in the shader
        assert(shaderLength < MAX_SHADER_LENGTH);   // make me bigger!
        if(shaderLength > MAX_SHADER_LENGTH)
			{
            fclose(fp);
            return false;
			}
		
        // Go back to beginning of file
        rewind(fp);
		
        // Read the whole file in
        if (shaderText != NULL)
            fread(shaderText, 1, shaderLength, fp);
		
        // Make sure it is null terminated and close the file
        shaderText[shaderLength] = '\0';
        fclose(fp);
		}
    else
        return false;    
	
    // Load the string
    gltLoadShaderSrc((const char *)shaderText, shader);
    
    return true;
	}   


/////////////////////////////////////////////////////////////////
// Load a pair of shaders, compile, and link together. Specify the complete
// file path for each shader. Note, there is no support for
// just loading say a vertex program... you have to do both.
GLuint gltLoadShaderPair(const char *szVertexProg, const char *szFragmentProg, bool loadFromFile)
{
	// Temporary Shader objects
	GLuint hVertexShader;
	GLuint hFragmentShader; 
	GLuint hReturn = 0;   
	GLint testVal;

	// Create shader objects
	hVertexShader = glCreateShader(GL_VERTEX_SHADER);
	hFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

	if (loadFromFile)
	{
	
	if(gltLoadShaderFile(szVertexProg, hVertexShader) == false)
		{
		glDeleteShader(hVertexShader);
		glDeleteShader(hFragmentShader);
		return (GLuint)NULL;
		}

	if(gltLoadShaderFile(szFragmentProg, hFragmentShader) == false)
		{
		glDeleteShader(hVertexShader);
		glDeleteShader(hFragmentShader);
		return (GLuint)NULL;
		}
	} else
	{
		gltLoadShaderSrc(vertexShader, hVertexShader);
		gltLoadShaderSrc(fragmentShader, hFragmentShader);
	}
	// Compile them
	glCompileShader(hVertexShader);
	glCompileShader(hFragmentShader);

	// Check for errors
	glGetShaderiv(hVertexShader, GL_COMPILE_STATUS, &testVal);
	if(testVal == GL_FALSE)
		{
		glDeleteShader(hVertexShader);
		glDeleteShader(hFragmentShader);
		return (GLuint)NULL;
		}

	glGetShaderiv(hFragmentShader, GL_COMPILE_STATUS, &testVal);
	if(testVal == GL_FALSE)
		{
		glDeleteShader(hVertexShader);
		glDeleteShader(hFragmentShader);
		return (GLuint)NULL;
		}

	// Link them - assuming it works...
	hReturn = glCreateProgram();
	glAttachShader(hReturn, hVertexShader);
	glAttachShader(hReturn, hFragmentShader);

	glLinkProgram(hReturn);

	// These are no longer needed
	glDeleteShader(hVertexShader);
	glDeleteShader(hFragmentShader);  

	// Make sure link worked too
	glGetProgramiv(hReturn, GL_LINK_STATUS, &testVal);
	if(testVal == GL_FALSE)
		{
		glDeleteProgram(hReturn);
		return (GLuint)NULL;
		}

	return hReturn;  
}   

///position xyz, unused w, normal, uv
static const GLfloat cube_vertices[] =
{
    -1.0f, -1.0f, 1.0f, 0.0f,	0,0,1,	0,0,//0
     1.0f, -1.0f, 1.0f, 0.0f,	0,0,1,	1,0,//1
     1.0f,  1.0f, 1.0f, 0.0f,	0,0,1,	1,1,//2
    -1.0f,  1.0f, 1.0f, 0.0f,	0,0,1,	0,1	,//3

	-1.0f, -1.0f, -1.0f, 1.0f,	0,0,-1,	0,0,//4
     1.0f, -1.0f, -1.0f, 1.0f,	0,0,-1,	1,0,//5
     1.0f,  1.0f, -1.0f, 1.0f,	0,0,-1,	1,1,//6
    -1.0f,  1.0f, -1.0f, 1.0f,	0,0,-1,	0,1,//7

    -1.0f, -1.0f, -1.0f, 1.0f,	-1,0,0,	0,0,
    -1.0f, 1.0f, -1.0f, 1.0f,	-1,0,0,	1,0,
    -1.0f,  1.0f, 1.0f, 1.0f,	-1,0,0,	1,1,
    -1.0f,  -1.0f, 1.0f, 1.0f,	-1,0,0,	0,1,

	1.0f, -1.0f, -1.0f, 1.0f,	1,0,0,	0,0,
    1.0f, 1.0f, -1.0f, 1.0f,	1,0,0,	1,0,
    1.0f,  1.0f, 1.0f, 1.0f,	1,0,0,	1,1,
    1.0f,  -1.0f, 1.0f, 1.0f,	1,0,0,	0,1,

	-1.0f, -1.0f,  -1.0f, 1.0f,	0,-1,0,	0,0,
    -1.0f, -1.0f, 1.0f, 1.0f,	0,-1,0,	1,0,
    1.0f, -1.0f,  1.0f, 1.0f,	0,-1,0,	1,1,
    1.0f,-1.0f,  -1.0f,  1.0f,	0,-1,0,	0,1,

	-1.0f, 1.0f,  -1.0f, 1.0f,	0,1,0,	0,0,
    -1.0f, 1.0f, 1.0f, 1.0f,	0,1,0,	1,0,
    1.0f, 1.0f,  1.0f, 1.0f,	0,1,0,	1,1,
    1.0f,1.0f,  -1.0f,  1.0f,	0,1,0,	0,1,
};

static const int cube_indices[]=
{
	0,1,2,0,2,3,//ground face
	4,5,6,4,6,7,//top face
	8,9,10,8,10,11,
	12,13,14,12,14,15,
	16,17,18,16,18,19,
	20,21,22,20,22,23
};

static const GLfloat instance_colors[] =
{
    1.0f, 0.0f, 0.0f, 1.0f,
    0.0f, 1.0f, 0.0f, 1.0f,
    0.0f, 0.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 0.0f, 1.0f
};

#define NUM_OBJECTS_X 100
#define NUM_OBJECTS_Y 100
#define NUM_OBJECTS_Z 100


#define NUM_OBJECTS (NUM_OBJECTS_X*NUM_OBJECTS_Y*NUM_OBJECTS_Z)
#define POSITION_BUFFER_SIZE (NUM_OBJECTS*sizeof(float)*4)
#define ORIENTATION_BUFFER_SIZE (NUM_OBJECTS*sizeof(float)*4)


GLfloat* instance_positions_ptr = 0;
GLfloat* instance_quaternion_ptr = 0;



void SetupRC()
{
	bool loadFromFile = false;
    instancingShader = gltLoadShaderPair("instancing.vs",
                                         "instancing.fs",
										 loadFromFile);
    glLinkProgram(instancingShader);
    glUseProgram(instancingShader);
    angle_loc = glGetUniformLocation(instancingShader, "angle");
	ModelViewMatrix = glGetUniformLocation(instancingShader, "ModelViewMatrix");
	ProjectionMatrix = glGetUniformLocation(instancingShader, "ProjectionMatrix");

	uniform_texture_diffuse = glGetUniformLocation(instancingShader, "Diffuse");

    GLuint offset = 0;

    glGenVertexArrays(1, &square_vao);
    glGenBuffers(1, &square_vbo);
    glBindVertexArray(square_vao);
    glBindBuffer(GL_ARRAY_BUFFER, square_vbo);

	instance_positions_ptr = (GLfloat*)new float[NUM_OBJECTS*4];
	instance_quaternion_ptr = (GLfloat*)new float[NUM_OBJECTS*4];
	int index=0;
	for (int i=0;i<NUM_OBJECTS_X;i++)
		for (int j=0;j<NUM_OBJECTS_Y;j++)
			for (int k=0;k<NUM_OBJECTS_Z;k++)
		{
			instance_positions_ptr[index*4]=-(i-NUM_OBJECTS_X/2)*10;
			instance_positions_ptr[index*4+1]=-(j-NUM_OBJECTS_Y/2)*10;
			instance_positions_ptr[index*4+2]=-k*10;
			instance_positions_ptr[index*4+3]=1;

			instance_quaternion_ptr[index*4]=0;
			instance_quaternion_ptr[index*4+1]=0;
			instance_quaternion_ptr[index*4+2]=0;
			instance_quaternion_ptr[index*4+3]=1;
			index++;
		}

	int size = sizeof(cube_vertices) + sizeof(instance_colors) + POSITION_BUFFER_SIZE+ORIENTATION_BUFFER_SIZE;
	glBufferData(GL_ARRAY_BUFFER, size, NULL, GL_DYNAMIC_DRAW);//GL_STATIC_DRAW);

	///initialize parts of the buffer
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(cube_vertices), cube_vertices);
	glBufferSubData(GL_ARRAY_BUFFER, sizeof(cube_vertices), sizeof(instance_colors), instance_colors);
    glBufferSubData(GL_ARRAY_BUFFER, sizeof(cube_vertices) + sizeof(instance_colors), POSITION_BUFFER_SIZE, instance_positions_ptr);
    glBufferSubData(GL_ARRAY_BUFFER, sizeof(cube_vertices) + sizeof(instance_colors)+POSITION_BUFFER_SIZE,ORIENTATION_BUFFER_SIZE , instance_quaternion_ptr);

	

//	glBindBuffer(GL_ARRAY_BUFFER,0);

	glGenBuffers(1, &index_vbo);
	int indexBufferSize = sizeof(cube_indices);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_vbo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexBufferSize, NULL, GL_STATIC_DRAW);
	glBufferSubData(GL_ELEMENT_ARRAY_BUFFER,0,indexBufferSize,cube_indices);
//	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	
	glBindVertexArray(0);

}

#include "btVector3.h"
#include "btQuaternion.h"
#include "btMatrix3x3.h"


void updateCamera() {

	float m_ele = 0;
	float m_azi=0;

	btVector3 m_cameraPosition(12,20,12);
	btVector3 m_cameraTargetPosition(0,10,0);

	btVector3 m_cameraUp(0,1,0);
	int m_forwardAxis=2;
	btScalar m_cameraDistance = 130;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();


	float m_frustumZNear=1;
	float m_frustumZFar=1000;

	if (m_glutScreenWidth == 0 && m_glutScreenHeight == 0)
		return;

	float aspect;
	btVector3 extents;

	if (m_glutScreenWidth > m_glutScreenHeight) 
	{
		aspect = m_glutScreenWidth / (float)m_glutScreenHeight;
		extents.setValue(aspect * 1.0f, 1.0f,0);
	} else 
	{
		aspect = m_glutScreenHeight / (float)m_glutScreenWidth;
		extents.setValue(1.0f, aspect*1.f,0);
	}

	
	if (m_ortho)
	{
		// reset matrix
		glLoadIdentity();
		extents *= m_cameraDistance;
		btVector3 lower = m_cameraTargetPosition - extents;
		btVector3 upper = m_cameraTargetPosition + extents;
		glOrtho(lower.getX(), upper.getX(), lower.getY(), upper.getY(),-1000,1000);
		
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
	} else
	{
		if (m_glutScreenWidth > m_glutScreenHeight) 
		{
			glFrustum (-aspect * m_frustumZNear, aspect * m_frustumZNear, -m_frustumZNear, m_frustumZNear, m_frustumZNear, m_frustumZFar);
		} else 
		{
			glFrustum (-aspect * m_frustumZNear, aspect * m_frustumZNear, -m_frustumZNear, m_frustumZNear, m_frustumZNear, m_frustumZFar);
		}
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		gluLookAt(m_cameraPosition[0], m_cameraPosition[1], m_cameraPosition[2], 
			m_cameraTargetPosition[0], m_cameraTargetPosition[1], m_cameraTargetPosition[2], 
			m_cameraUp.getX(),m_cameraUp.getY(),m_cameraUp.getZ());
	}

}



void myinit()
{

	
//	GLfloat light_ambient[] = { btScalar(0.2), btScalar(0.2), btScalar(0.2), btScalar(1.0) };
	GLfloat light_ambient[] = { btScalar(1.0), btScalar(1.2), btScalar(0.2), btScalar(1.0) };

	GLfloat light_diffuse[] = { btScalar(1.0), btScalar(1.0), btScalar(1.0), btScalar(1.0) };
	GLfloat light_specular[] = { btScalar(1.0), btScalar(1.0), btScalar(1.0), btScalar(1.0 )};
	/*	light_position is NOT default value	*/
	GLfloat light_position0[] = { btScalar(1000.0), btScalar(1000.0), btScalar(1000.0), btScalar(0.0 )};
	GLfloat light_position1[] = { btScalar(-1.0), btScalar(-10.0), btScalar(-1.0), btScalar(0.0) };

	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position0);

	glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT1, GL_POSITION, light_position1);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);


//	glShadeModel(GL_FLAT);//GL_SMOOTH);
	glShadeModel(GL_SMOOTH);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	glClearColor(float(0.7),float(0.7),float(0.7),float(0));
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);


	static bool m_textureenabled = true;
	static bool m_textureinitialized = false;


		if(m_textureenabled)
		{
			if(!m_textureinitialized)
			{
				glActiveTexture(GL_TEXTURE0);

				GLubyte*	image=new GLubyte[256*256*3];
				for(int y=0;y<256;++y)
				{
					const int	t=y>>5;
					GLubyte*	pi=image+y*256*3;
					for(int x=0;x<256;++x)
					{
						const int		s=x>>5;
						const GLubyte	b=180;					
						GLubyte			c=b+((s+t&1)&1)*(255-b);
						pi[0]=255;
						pi[1]=c;
						pi[2]=c;
						pi+=3;
					}
				}

				glGenTextures(1,(GLuint*)&m_texturehandle);
				glBindTexture(GL_TEXTURE_2D,m_texturehandle);
				glTexEnvf(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
				glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_LINEAR);
				glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR_MIPMAP_LINEAR);
				glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
				glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
				gluBuild2DMipmaps(GL_TEXTURE_2D,3,256,256,GL_RGB,GL_UNSIGNED_BYTE,image);
				delete[] image;
				m_textureinitialized=true;
			}
	//		glMatrixMode(GL_TEXTURE);
	//		glLoadIdentity();
	//		glMatrixMode(GL_MODELVIEW);

			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D,m_texturehandle);

		} else
		{
			glDisable(GL_TEXTURE_2D);
		}

		glEnable(GL_COLOR_MATERIAL);


//	  glEnable(GL_CULL_FACE);
//	  glCullFace(GL_BACK);
}

//#pragma optimize( "g", off )

void updatePos()
{
	static float angle(0);

	if (useCPU)
	{
		glBindBuffer(GL_ARRAY_BUFFER, square_vbo);
		char* bla =  (char*)glMapBuffer( GL_ARRAY_BUFFER,GL_WRITE_ONLY);

		float* positions = (float*)(bla+sizeof(cube_vertices) + sizeof(instance_colors));
		float* orientations = (float*)(bla+sizeof(cube_vertices) + sizeof(instance_colors)+ POSITION_BUFFER_SIZE);
	//	glFinish();
	//	positions[0]+=0.001f;

		static int offset=0;
		//offset++;

		static btVector3 axis(1,0,0);
		angle += 0.01f;
		int index=0;
		btQuaternion orn(axis,angle);
		for (int i=0;i<NUM_OBJECTS_X;i++)
			for (int j=0;j<NUM_OBJECTS_Y;j++)
				for (int k=0;k<NUM_OBJECTS_Z;k++)
				{
					//if (!((index+offset)%15))
					{
					positions[index*4+1]-=.01f;
					orientations[index*4] = orn[0];
					orientations[index*4+1] = orn[1];
					orientations[index*4+2] = orn[2];
					orientations[index*4+3] = orn[3];
					}
	//				memcpy((void*)&orientations[index*4],orn,sizeof(btQuaternion));
					index++;
				}

		glUnmapBuffer( GL_ARRAY_BUFFER);
	} 
	else
	{

//		glBindBuffer(GL_ARRAY_BUFFER,0);

		cl_mem clBuffer = g_interopBuffer->getCLBUffer();

		glFinish();
		cl_int ciErrNum = CL_SUCCESS;
		ciErrNum = clEnqueueAcquireGLObjects(g_cqCommandQue, 1, &clBuffer, 0, 0, NULL);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		if (runOpenCLKernels)
		{
			int numObjects = NUM_OBJECTS;
			int offset = (sizeof(cube_vertices) + sizeof(instance_colors))/4;

			ciErrNum = clSetKernelArg(g_interopKernel, 0, sizeof(int), &offset);
			ciErrNum = clSetKernelArg(g_interopKernel, 1, sizeof(int), &numObjects);
			ciErrNum = clSetKernelArg(g_interopKernel, 2, sizeof(cl_mem), (void*)&clBuffer );
			size_t	numWorkItems = workGroupSize*((NUM_OBJECTS + (workGroupSize-1)) / workGroupSize);
			ciErrNum = clEnqueueNDRangeKernel(g_cqCommandQue, g_interopKernel, 1, NULL, &numWorkItems, &workGroupSize,0 ,0 ,0);
			oclCHECKERROR(ciErrNum, CL_SUCCESS);
		}
		ciErrNum = clEnqueueReleaseGLObjects(g_cqCommandQue, 1, &clBuffer, 0, 0, 0);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		clFinish(g_cqCommandQue);

	}

}
//#pragma optimize( "g", on )

void RenderScene(void)
{

#if 0
	float modelview[20]={0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9};
	// get the current modelview matrix
	glGetFloatv(GL_MODELVIEW_MATRIX , modelview);
	float projection[20]={0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9};
	glGetFloatv(GL_PROJECTION_MATRIX, projection);
#endif

	myinit();
	
	updateCamera();

//    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
//    glClear(GL_COLOR_BUFFER_BIT);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

//render coordinate system
	glBegin(GL_LINES);
	glColor3f(1,0,0);
	glVertex3f(0,0,0);
	glVertex3f(1,0,0);
	glColor3f(0,1,0);
	glVertex3f(0,0,0);
	glVertex3f(0,1,0);
	glColor3f(0,0,1);
	glVertex3f(0,0,0);
	glVertex3f(0,0,1);
	glEnd();


	float start = gStopwatch.getTimeMilliseconds();

	updatePos();

	float stop = gStopwatch.getTimeMilliseconds();
	gStopwatch.reset();
	if (printStats)
	{
		printf("updatePos=%f ms on ",stop-start);

		if (useCPU)
		{
			printf("CPU \n");
		} else
		{
			printf("OpenCL ");
			if (runOpenCLKernels)
				printf("running the kernels");
			else
				printf("without running the kernels");
			printf("\n");
		}
	}
	glBindBuffer(GL_ARRAY_BUFFER, square_vbo);

    glBindVertexArray(square_vao);

	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 9*sizeof(float), 0);
	glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, (GLvoid *)sizeof(cube_vertices));
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 0, (GLvoid *)(sizeof(cube_vertices) + sizeof(instance_colors)));
    glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, 0, (GLvoid *)(sizeof(cube_vertices) + sizeof(instance_colors)+POSITION_BUFFER_SIZE));
	int uvoffset = 7*sizeof(float);
	int normaloffset = 4*sizeof(float);

	glVertexAttribPointer(4, 2, GL_FLOAT, GL_FALSE, 9*sizeof(float), (GLvoid *)uvoffset);
	glVertexAttribPointer(5, 3, GL_FLOAT, GL_FALSE, 9*sizeof(float), (GLvoid *)normaloffset);

    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glEnableVertexAttribArray(2);
    glEnableVertexAttribArray(3);
    glEnableVertexAttribArray(4);
    glEnableVertexAttribArray(5);

    glVertexAttribDivisor(1, 1);
    glVertexAttribDivisor(2, 1);
    glVertexAttribDivisor(3, 1);
    glVertexAttribDivisor(4, 0);
    glVertexAttribDivisor(5, 0);

	glUseProgram(instancingShader);
    glUniform1f(angle_loc, 0);
	GLfloat pm[16];
	glGetFloatv(GL_PROJECTION_MATRIX, pm);
	glUniformMatrix4fv(ProjectionMatrix, 1, false, &pm[0]);

	GLfloat mvm[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, mvm);
	glUniformMatrix4fv(ModelViewMatrix, 1, false, &mvm[0]);

   	glUniform1i(uniform_texture_diffuse, 0);
	
	int numInstances = NUM_OBJECTS;
	int indexCount = sizeof(cube_indices);
	int indexOffset = 0;

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_vbo);

	glDrawElementsInstanced(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, (void*)indexOffset, numInstances);

	glUseProgram(0);
	glBindBuffer(GL_ARRAY_BUFFER,0);
	glBindVertexArray(0);

	glutSwapBuffers();
	glFinish();
	glutPostRedisplay();
}


void ChangeSize(int w, int h)
{
	m_glutScreenWidth = w;
	m_glutScreenHeight = h;

    // Prevent a divide by zero
    if(h == 0)
        h = 1;

    // Set Viewport to window dimensions
    glViewport(0, 0, w, h);
}

void Keyboard(unsigned char key, int x, int y)
{
    switch (key)
    {
        case 27:
            done = true;
            break;
		case 'O':
		case 'o':
			{
				m_ortho = !m_ortho;
				break;
			}
		case 'c':
		case 'C':
			{
				useCPU = !useCPU;
				if (useCPU)
					printf("using CPU\n");
				else
					printf("using OpenCL\n");
				break;
			}
		case 's':
		case 'S':
			{
				printStats = !printStats;
				break;
			}
		case 'k':
		case 'K':
			{
				runOpenCLKernels=!runOpenCLKernels;
				break;
			}
        default:
            break;
    }
}

// Cleanup
void ShutdownRC(void)
{
    glDeleteBuffers(1, &square_vbo);
    glDeleteVertexArrays(1, &square_vao);
}

int main(int argc, char* argv[])
{
	printf("vertexShader = \n%s\n",vertexShader);
	printf("fragmentShader = \n%s\n",fragmentShader);

    glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
    
	glutInitWindowSize(m_glutScreenWidth, m_glutScreenHeight);
	char buf[1024];
	sprintf(buf,"OpenCL - OpenGL interop, updating transforms of %d cubes using glDrawElementsInstanced", NUM_OBJECTS);
    glutCreateWindow(buf);
    
	ChangeSize(m_glutScreenWidth,m_glutScreenHeight);

	glutReshapeFunc(ChangeSize);

    glutKeyboardFunc(Keyboard);
    glutDisplayFunc(RenderScene);

    GLenum err = glewInit();
    if (GLEW_OK != err)
    {
        /* Problem: glewInit failed, something is seriously wrong. */
        fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
    }


void* glCtx=0;
void* glDC = 0;

#ifdef _WIN32
    glCtx = wglGetCurrentContext();
#else //!_WIN32
    GLXContext glCtx = glXGetCurrentContext();
#endif //!_WIN32
	glDC = wglGetCurrentDC();

		int ciErrNum = 0;
	cl_device_type deviceType = CL_DEVICE_TYPE_GPU;
	g_cxMainContext = btOpenCLUtils::createContextFromType(deviceType, &ciErrNum, glCtx, glDC);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	int numDev = btOpenCLUtils::getNumDevices(g_cxMainContext);

	if (numDev>0)
	{
		g_device= btOpenCLUtils::getDevice(g_cxMainContext,0);
		btOpenCLDeviceInfo clInfo;
		btOpenCLUtils::getDeviceInfo(g_device,clInfo);
		btOpenCLUtils::printDeviceInfo(g_device);
		// create a command-queue
		g_cqCommandQue = clCreateCommandQueue(g_cxMainContext, g_device, 0, &ciErrNum);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		//normally you would create and execute kernels using this command queue

	}

	    SetupRC();

	g_interopBuffer = new btOpenCLGLInteropBuffer(g_cxMainContext,g_cqCommandQue,square_vbo);
	clFinish(g_cqCommandQue);

	//cl_mem clBuffer = g_interopBuffer->getCLBUffer();
//	ciErrNum = clEnqueueAcquireGLObjects(g_cqCommandQue, 1, &clBuffer, 0, 0, NULL);
//	oclCHECKERROR(ciErrNum, CL_SUCCESS);


	g_interopKernel = btOpenCLUtils::compileCLKernelFromString(g_cxMainContext, interopKernelString, "interopKernel" );

    glutMainLoop();
    ShutdownRC();

    return 0;
}