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
	qglviewer::Vec vRot = v0^v1;
	if (vRot.norm() != 0)
		vRot.normalize();
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
  R[1][1];
	(*q)[0] = ( R[0][0] + R[1][1] + R[2][2] + 1.0f) / 4.0f;
	(*q)[1] = ( R[0][0] - R[1][1] - R[2][2] + 1.0f) / 4.0f;
	(*q)[2] = (-R[0][0] + R[1][1] - R[2][2] + 1.0f) / 4.0f;
	(*q)[3] = (-R[0][0] - R[1][1] + R[2][2] + 1.0f) / 4.0f;

	if((*q)[0] < 0.0f) (*q)[0] = 0.0f;
	if((*q)[1] < 0.0f) (*q)[1] = 0.0f;
	if((*q)[2] < 0.0f) (*q)[2] = 0.0f;
	if((*q)[3] < 0.0f) (*q)[3] = 0.0f;

	(*q)[0] = sqrt((*q)[0]);
	(*q)[1] = sqrt((*q)[1]);
	(*q)[2] = sqrt((*q)[2]);
	(*q)[3] = sqrt((*q)[3]);

	if((*q)[0] >= (*q)[1] && (*q)[0] >= (*q)[2] && (*q)[0] >= (*q)[3]) {
		(*q)[0] *= +1.0f;
		(*q)[1] *= SIGN(R[2][1] - R[1][2]);
		(*q)[2] *= SIGN(R[0][2] - R[2][0]);
		(*q)[3] *= SIGN(R[1][0] - R[0][1]);

	} else if((*q)[1] >= (*q)[0] && (*q)[1] >= (*q)[2] && (*q)[1] >= (*q)[3]) {
		(*q)[0] *= SIGN(R[2][1] - R[1][2]);
		(*q)[1] *= +1.0f;
		(*q)[2] *= SIGN(R[1][0] + R[0][1]);
		(*q)[3] *= SIGN(R[0][2] + R[2][0]);

	} else if((*q)[2] >= (*q)[0] && (*q)[2] >= (*q)[1] && (*q)[2] >= (*q)[3]) {
		(*q)[0] *= SIGN(R[0][2] - R[2][0]);
		(*q)[1] *= SIGN(R[1][0] + R[0][1]);
		(*q)[2] *= +1.0f;
		(*q)[3] *= SIGN(R[2][1] + R[1][2]);

	} else if((*q)[3] >= (*q)[0] && (*q)[3] >= (*q)[1] && (*q)[3] >= (*q)[2]) {
		(*q)[0] *= SIGN(R[1][0] - R[0][1]);
		(*q)[1] *= SIGN(R[2][0] + R[0][2]);
		(*q)[2] *= SIGN(R[2][1] + R[1][2]);
		(*q)[3] *= +1.0f;

	} else {
		std::cerr << "coding error\n" << std::endl;
	}

	float r = NORM((*q)[0], (*q)[1], (*q)[2], (*q)[3]);
	(*q)[0] /= r;
	(*q)[1] /= r;
	(*q)[2] /= r;
	(*q)[3] /= r;

}

void Skeleton::quaternionToAxisAngle(qglviewer::Quaternion q, qglviewer::Vec *vaa)
{
   if (q[3] > 1) q.normalize(); // if w>1 acos and sqrt will produce errors, this cant happen if quaternion is normalised
   float angle = 2 * acos(q[3]);
   double s = sqrt(1-q[3]*q[3]); // assuming quaternion normalised then w is less than 1, so term always positive.
   if (s < 0.001) { // test to avoid divide by zero, s is always positive due to sqrt
     // if s close to zero then direction of axis not important
     (*vaa)[0] = q[0]; // if it is important that axis is normalised then replace with x=1; y=z=0;
     (*vaa)[1] = q[1];
     (*vaa)[2] = q[2];
   } else {
     (*vaa)[0] = q[0] / s; // normalise axis
     (*vaa)[1] = q[2] / s;
     (*vaa)[2] = q[3] / s;
   }

   (*vaa) = angle*(*vaa);
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
	int isImplemented = 1;

	// TEST ////////////////////
	qglviewer::Vec *vaa = new qglviewer::Vec();
	eulerToAxisAngle(90,90,0, roXYZ, vaa);
	// TODO : pas sur que ca marche ce truc ...
	std::cout << (*vaa)[0] << ", " << (*vaa)[1] << ", " << (*vaa)[2] << ", norm = " << (*vaa).norm() << std::endl;
	////////////////////////////
	if (vaa->norm() < 0.001)
	  nbDofsR = 0;
	else {
	  eulerToAxisAngle(_dofs[0]._values[0],_dofs[1]._values[0],_dofs[2]._values[0], roXYZ, vaa);
	  //eulerToAxisAngle(this->_curRx,this->_curRy,this->_curRz, this->_rorder, vaa);
	  for (int j = 1; j < _dofs[0]._values.size(); ++j) {
	    qglviewer::Vec vaaPrec = (*vaa);
	    vaaPrec.normalize();	
	    eulerToAxisAngle(_dofs[0]._values[j],_dofs[1]._values[j],_dofs[2]._values[j], roXYZ, vaa);
	    qglviewer::Vec vaaNormalized = (*vaa);
	    vaaNormalized.normalize();
	    if (vaaNormalized * vaaPrec >= tol) {
	      nbDofsR = 2;
	    }
	  }
	}
	if (nbDofsR == -1)
	  nbDofsR = 1;

	if (!isImplemented) return;

	if (nbDofsR == 2)
	  	cout << _name << " : >2 degree(s) of freedom in rotation\n";

	else
	cout << _name << " : " << nbDofsR << " degree(s) of freedom in rotation\n";

	// Propagate to children :
	for (unsigned int ichild = 0 ; ichild < _children.size() ; ichild++) {
		_children[ichild]->nbDofs();
	}

}
