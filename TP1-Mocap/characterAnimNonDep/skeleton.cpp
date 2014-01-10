#include "skeleton.h"
#include <skeletonIO.h>
#include <qglviewer.h>
#include <math.h>


using namespace std;

Skeleton* Skeleton::createFromFile(std::string fileName) {
	Skeleton* root = NULL;
	cout << "Loading from " << fileName << endl;

	ifstream inputfile(fileName.data());
	if(inputfile.good()) {
		while(!inputfile.eof()) {
			string buf;	
			inputfile >> buf;
			if(!buf.compare("HIERARCHY")) {
				root = readHierarchy(inputfile);
			}
		}
		inputfile.close();
	} else {
		std::cerr << "Failed to load the file " << fileName.data() << std::endl;
		fflush(stdout);
	}

	cout << "file loaded" << endl;

	return root;
}


void drawBone(Skeleton *child) 
{
	qglviewer::Vec v0(0,0,1);
	qglviewer::Vec v1(child->_offX, child->_offY, child->_offZ);
	qglviewer::Vec vRot = v0^v1; vRot.normalize();
	float angle = acosf((v0*v1)/(v0.norm()*v1.norm()))*180.0/M_PI;
	float height = (v1-v0).norm();
	float radius = 0.1f;
	glPushMatrix();
	{
		glRotatef(angle, vRot.x, vRot.y, vRot.z);
		gluCylinder(gluNewQuadric(), 0.1, 0.1, height, 5, 5);
	}
	glPopMatrix();
	
}

void Skeleton::rotateSkeleton() {
	switch (_rorder) {
		case roXYZ :
			glRotatef(_curRx, 1, 0, 0);
			glRotatef(_curRy, 0, 1, 0);
			glRotatef(_curRz, 0, 0, 1);
			break;
		case roYZX :
			glRotatef(_curRy, 0, 1, 0);
			glRotatef(_curRz, 0, 0, 1);
			glRotatef(_curRx, 1, 0, 0);
			break;
		case roZXY :
			glRotatef(_curRz, 0, 0, 1);
			glRotatef(_curRx, 1, 0, 0);
			glRotatef(_curRy, 0, 1, 0);
			break;
		case roXZY :
			glRotatef(_curRx, 1, 0, 0);
			glRotatef(_curRz, 0, 0, 1);
			glRotatef(_curRy, 0, 1, 0);
			break;
		case roYXZ :
			glRotatef(_curRy, 0, 1, 0);
			glRotatef(_curRx, 1, 0, 0);
			glRotatef(_curRz, 0, 0, 1);
			break;
		case roZYX :
			glRotatef(_curRz, 0, 0, 1);
			glRotatef(_curRy, 0, 1, 0);
			glRotatef(_curRx, 1, 0, 0);
			break;
	}
}
void Skeleton::draw() 
{
	glPushMatrix();
	{
		// Set good reference frame :
		glTranslatef(_offX, _offY, _offZ);
		// Use current value of dofs :
		glTranslatef(_curTx, _curTy, _curTz);
		rotateSkeleton();
		// Draw articulation :
		glColor3f(1,0,0),
		gluSphere(gluNewQuadric(), 0.25, 10, 10);
		// Draw bone and children :
		glColor3f(0,0,1);
		for (unsigned int ichild = 0 ; ichild < _children.size() ; ichild++) {
			drawBone(_children[ichild]);
			_children[ichild]->draw();
		}
	}
	glPopMatrix();
}

void Skeleton::animate(int iframe) 
{
	// Update dofs :
	_curTx = 0; _curTy = 0; _curTz = 0;
	_curRx = 0; _curRy = 0; _curRz = 0;
	for (unsigned int idof = 0 ; idof < _dofs.size() ; idof++) {
		if(!_dofs[idof].name.compare("Xposition")) _curTx = _dofs[idof]._values[iframe];
		if(!_dofs[idof].name.compare("Yposition")) _curTy = _dofs[idof]._values[iframe];
		if(!_dofs[idof].name.compare("Zposition")) _curTz = _dofs[idof]._values[iframe];
		if(!_dofs[idof].name.compare("Zrotation")) _curRz = _dofs[idof]._values[iframe];
		if(!_dofs[idof].name.compare("Yrotation")) _curRy = _dofs[idof]._values[iframe];
		if(!_dofs[idof].name.compare("Xrotation")) _curRx = _dofs[idof]._values[iframe];
	}	
	// Animate children :
	for (unsigned int ichild = 0 ; ichild < _children.size() ; ichild++) {
		_children[ichild]->animate(iframe);
	}
}

void Skeleton::eulerToMatrix(double rx, double ry, double rz, int rorder, glm::mat3 *R)
{
  glm::mat3 mx(1, 0,       0,
	       0, cos(rx), -sin(rx),
	       0, sin(rx), cos(rx));

  glm::mat3 my(cos(ry),  0, sin(ry),
	       0,        1, 0,
	       -sin(ry), 0, cos(ry));

  glm::mat3 mz(cos(rz),  -sin(ry), 0,
	       sin(rz),  cos(rz),  0,
	       0,        0,        1);

  switch (rorder) {
  case roZYX :
    *R = mx*my*mz;
    break;
  case roXZY :
    *R = my*mz*mx;
    break;
  case roYXZ :
    *R = mz*mx*my;
    break;
  case roYZX :
    *R = mx*mz*my;
    break;
  case roZXY :
    *R = my*mx*mz;
    break;
  case roXYZ :
    *R = mz*my*mx;
    break;
  default :
    *R = mx*my*mz;
  }
  
}

inline float SIGN(float x) {return (x >= 0.0f) ? +1.0f : -1.0f;}
inline float NORM(float a, float b, float c, float d) {return sqrt(a * a + b * b + c * c + d * d);}

void Skeleton::matrixToQuaternion(glm::mat3 R, qglviewer::Quaternion *q)
{
  
  q0 = ( r11 + r22 + r33 + 1.0f) / 4.0f;
  q1 = ( r11 - r22 - r33 + 1.0f) / 4.0f;
  q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
  q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;

  if(q0 < 0.0f) q0 = 0.0f;
  if(q1 < 0.0f) q1 = 0.0f;
  if(q2 < 0.0f) q2 = 0.0f;
  if(q3 < 0.0f) q3 = 0.0f;

  q0 = sqrt(q0);
  q1 = sqrt(q1);
  q2 = sqrt(q2);
  q3 = sqrt(q3);

  if(q0 >= q1 && q0 >= q2 && q0 >= q3) {
    q0 *= +1.0f;
    q1 *= SIGN(r32 - r23);
    q2 *= SIGN(r13 - r31);
    q3 *= SIGN(r21 - r12);

  } else if(q1 >= q0 && q1 >= q2 && q1 >= q3) {
    q0 *= SIGN(r32 - r23);
    q1 *= +1.0f;
    q2 *= SIGN(r21 + r12);
    q3 *= SIGN(r13 + r31);

  } else if(q2 >= q0 && q2 >= q1 && q2 >= q3) {
    q0 *= SIGN(r13 - r31);
    q1 *= SIGN(r21 + r12);
    q2 *= +1.0f;
    q3 *= SIGN(r32 + r23);

  } else if(q3 >= q0 && q3 >= q1 && q3 >= q2) {
    q0 *= SIGN(r21 - r12);
    q1 *= SIGN(r31 + r13);
    q2 *= SIGN(r32 + r23);
    q3 *= +1.0f;

  } else {
    std::cerr << "coding error\n" << std::endl;
  }

  r = NORM(q0, q1, q2, q3);
  q0 /= r;
  q1 /= r;
  q2 /= r;
  q3 /= r;
  
}
void Skeleton::quaternionToAxisAngle(qglviewer::Quaternion q, qglviewer::Vec *vaa)
{
	
}
void Skeleton::eulerToAxisAngle(double rx, double ry, double rz, int rorder, qglviewer::Vec *vaa)
{
	// Euler -> matrix :
	glm::mat3 R;
	eulerToMatrix(M_PI*rx/180.0, M_PI*ry/180.0, M_PI*rz/180.0, rorder, &R);
	// matrix -> quaternion :
	qglviewer::Quaternion q;
	matrixToQuaternion(R, &q);
	// quaternion -> axis/angle :
	quaternionToAxisAngle(q, vaa);
}

void Skeleton::nbDofs() {
	if (_dofs.empty()) return;

	double tol = 1e-4;

	int nbDofsR = -1;

	// TO COMPLETE :
	int isImplemented = 0;

	
	if (!isImplemented) return;
	cout << _name << " : " << nbDofsR << " degree(s) of freedom in rotation\n";

	// Propagate to children :
	for (unsigned int ichild = 0 ; ichild < _children.size() ; ichild++) {
		_children[ichild]->nbDofs();
	}

}
