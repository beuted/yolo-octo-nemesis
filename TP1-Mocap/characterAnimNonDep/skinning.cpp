#include <iostream>

#include "skinning.h"
#include "dosunix_bind.h"

using namespace std;

void Skinning::init() {
	if (_skin==NULL) return;
	if (_skel==NULL) return;

	// Compute number of joints :
	getJoints(_skel);
	_nbJoints = _joints.size();

	// Get mesh info :
	_nbVtx = _skin->_points.size();
	_weights.resize(_nbVtx);
	_pointsInit.resize(_nbVtx);
	for (int iv = 0 ; iv < _nbVtx ; iv++) {
		_weights[iv].resize(_nbJoints, 0);
		_pointsInit[iv] = _skin->_points[iv];
		_pointsInit[iv][3] = 1.0;
	}

	// Get transfo joint info :
	_transfoInit.resize(_nbJoints);
	_transfoInitInv.resize(_nbJoints);
	_transfoCurr.resize(_nbJoints);
	int idx = 0;
	glPushMatrix();
	glLoadIdentity();
	computeTransfo(_skel, &idx);
	glPopMatrix();
	for (unsigned int i = 0 ; i < _transfoCurr.size() ; i++) {
		_transfoInit[i] = _transfoCurr[i];
		_transfoInitInv[i] = glm::inverse(_transfoInit[i]);
	}

	// Get bones pose info :
	idx = 0;
	_posBonesInit.resize(_nbJoints);
	getBonesPos(_skel, &idx);

	// Compute weights :
	switch (_meth) {
		case 0:
			loadWeights("data/skinning.txt");
			break;
		case 1:
			computeWeights();
			break;
		case 2:
			computeSmoothWeights();
			break;
	}



	// Test skinning :
	animate();
}

void Skinning::recomputeWeights() {
	if (_skin==NULL) return;
	if (_skel==NULL) return;

	// Compute weights :
	switch (_meth) {
		case 0:
			cout << "loading weights\n";
			loadWeights("data/skinning.txt");
			break;
		case 1:
			cout << "computing linear weights\n";
			computeWeights();
			break;
		case 2:
			cout << "computing smooth weights\n";
			computeSmoothWeights();
			break;
	}

	// Test skinning :
	animate();
}

void Skinning::getJoints(Skeleton *skel) {
	_joints.push_back(skel);
	for (unsigned int ichild = 0 ; ichild < skel->_children.size() ; ichild++) {
		getJoints(skel->_children[ichild]);
	}
}
void Skinning::getBonesPos(Skeleton *skel, int *idx) {
	int i0 = (*idx);
	qglviewer::Vec pos(_transfoInit[i0][3][0], _transfoInit[i0][3][1], _transfoInit[i0][3][2]);
	for (unsigned int ichild = 0 ; ichild < skel->_children.size() ; ichild++) {
		(*idx)++;
		pos+=qglviewer::Vec(_transfoInit[(*idx)][3][0], _transfoInit[(*idx)][3][1], _transfoInit[(*idx)][3][2]);
		getBonesPos(skel->_children[ichild], idx);
	}
	pos/=(float)(skel->_children.size()+1);
	_posBonesInit[i0] = glm::vec4(pos.x, pos.y, pos.z, 1.0);
}

void Skinning::computeTransfo(Skeleton *skel, int *idx) {
	int i0 = (*idx);
	glPushMatrix();
	{
		glTranslatef(skel->_offX, skel->_offY, skel->_offZ);
		glTranslatef(skel->_curTx, skel->_curTy, skel->_curTz);
		skel->rotateSkeleton();

		float ptr[16];
		glGetFloatv(GL_MODELVIEW_MATRIX, ptr);
		int i = 0;
		for (int j = 0 ; j < 4 ; j++) {
			for (int k = 0 ; k < 4 ; k++) {
				_transfoCurr[(*idx)][k][j] = ptr[i];
				i++;
			}
		}
		for (unsigned int ichild = 0 ; ichild < skel->_children.size() ; ichild++) {
			(*idx)++;
			computeTransfo(skel->_children[ichild], idx);
		}
	}
	glPopMatrix();
	_transfoCurr[i0] = glm::transpose(_transfoCurr[i0]);
}


void Skinning::computeSmoothWeights() {
	if (_skin==NULL) return;
	if (_skel==NULL) return;

	for (unsigned int i = 0; i < _pointsInit.size() ; ++i) {
		int j_min_dist = -1;
		double min_dist = std::numeric_limits<double>::max();
		glm::vec4 P = _pointsInit[i];

		for (unsigned int j = 0; j < _posBonesInit.size(); ++j) {
			_weights[i][j] = 0; // initialisation

			double PUdist;

			// Deux skeletons S et T
			// Dot product (SP . ST) * ST = SU
			// Resultat dans kU
			glm::vec4 T = _posBonesInit[j];
			Skeleton *skel = _joints[j];
			if (skel->_children.size() == 0) {
				//std::cerr << skel->_name << " n'a pas de fils" << std::endl;
				continue;
			}

			// Vectors definition
			glm::mat4 M = _transfoInit[j];
			glm::vec4 S(M[3][0],M[3][1],M[3][2],M[3][3]); //TODO position du skel
			glm::vec4 ST = T - S;
			glm::vec4 SP = P - S;
			double STn = sqrt(ST[0]*ST[0] + ST[1]*ST[1] + ST[2]*ST[2] + ST[3]*ST[3]);

			if (STn == 0) {
				// Spherical distance
				PUdist = glm::distance(P, T);			
			} else {
				// Projection and coefficient computation
				double dotpdct = glm::dot(SP, ST);
				glm::vec4 SU = (dotpdct / (STn * STn)) * ST;
				double SUn = sqrt(SU[0]*SU[0] + SU[1]*SU[1] + SU[2]*SU[2] + SU[3]*SU[3]);
				double kU = SUn / STn;

				// Check si entre 0 et 2
				if (dotpdct < 0 || kU > 2)
					continue;

				// Pythagore
				// sqrt ( || SP ||² - || ku * ST ||² ) = dist
				double SPn = sqrt(SP[0]*SP[0] + SP[1]*SP[1] + SP[2]*SP[2] + SP[3]*SP[3]);
				double PUdistpow = (SPn * SPn) - (SUn * SUn);
				if (PUdistpow < 0)
					continue;
				PUdist = sqrt(PUdistpow);
				//double norm_diff = sqrt(diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2]);
			}

			// Check for min dist
			if (PUdist < min_dist) {
				j_min_dist = j;
				min_dist = PUdist;
			}
		}

		if (j_min_dist != -1 ) {
			_weights[i][j_min_dist] = 1.0;
		}
	}
}

void Skinning::computeWeights() {
	if (_skin==NULL) return;
	if (_skel==NULL) return;

	for (unsigned int i = 0; i < _pointsInit.size() ; ++i) {
		int j_min_dist = -1;
		double min_dist = std::numeric_limits<double>::max();
		for (unsigned int j = 0; j < _posBonesInit.size(); ++j) {
			_weights[i][j] = 0.0; // initialisation
			// recherche de max
			glm::vec4 diff = (_pointsInit[i] - _posBonesInit[j]);
			double norm_diff = sqrt(diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2]);
			if (norm_diff < min_dist) {
				j_min_dist = j;
				min_dist = norm_diff;
			}

		}
		_weights[i][j_min_dist] = 1.0;
	}
}

void Skinning::loadWeights(std::string filename) {
	std::vector<float> bone_indexA;
	std::vector<float> bone_weightA;
	FILE *file; fopen_r_dosunix(file, filename.data(), "r");
	if (!file) return;
	char * pch, *next_token;
	const int line_size = 600;
	char line[line_size];
	int iV = 0;
	while (!feof(file)) {
		// for each line i.e. for each vertex :
		if (fgets(line, line_size, file)) {
			int iJt = 0;
			float x;
			pch = strtok_r_dosunix(line," ", &next_token);
			while (pch != NULL) {
				// for each number i.e. for each joint :
				if (pch[0]=='\n'){
				} else {
					x = (float)atof(pch);
					_weights[iV][iJt] = x;
				}
				pch = strtok_r_dosunix(NULL, " ", &next_token);
				iJt++;
			}
			iV++;
		}		
	}
	fclose(file);
}

void Skinning::paintWeights(std::string jointName) {
	if (_skin==NULL) return;
	if (_skel==NULL) return;

	unsigned int i;
	for (i = 0; i < _joints.size() && _joints[i]->_name.compare(jointName) != 0; i++);

	// Joint not found
	if (i == _joints.size() && _joints[i]->_name.compare(jointName) != 0) {
		std::cerr << "Warning: Joint " << jointName << " not found" << std::endl;
		return;
	} else {
		std::cerr << "Joint " << jointName << " found at " << i << std::endl;
		std::cerr << _joints.size()  <<  " joints in the mesh" << std::endl;
	}

	unsigned int jointIdx = i;
	_skin->_colors.clear();
	for (unsigned int i = 0; i < _skin->_points.size() ; ++i) {
		double weight = _weights[i][jointIdx];
		_skin->_colors.push_back(glm::vec4(0.0, weight, 0.0, 0.9));
	}

}

void Skinning::animate() {
	if (_skin==NULL) return;
	if (_skel==NULL) return;

	// Animate bones :
	int idx = 0;
	glPushMatrix();
	glLoadIdentity();
	computeTransfo(_skel, &idx);
	glPopMatrix();

	// Animate skin :
#if _SKINNING_GPU
#else
	applySkinning();
#endif
}

void Skinning::applySkinning() {
	for (unsigned int i = 0; i < _pointsInit.size(); ++i) {
		glm::vec4 * P = &(_pointsInit[i]);
		glm::vec4   P_new(0.0,0.0,0.0,0.0);
		for (unsigned int j = 0; j < _joints.size(); ++j) {
			P_new += _weights[i][j] * (_transfoCurr[j] * _transfoInitInv[j]) * (*P);
		}
		_skin->_points[i] = P_new;
	}
}
