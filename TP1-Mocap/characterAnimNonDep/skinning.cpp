#include <iostream>

#include "skinning.h"
#include "dosunix_bind.h"

#define NB_COEF_SMOOTH (_posBonesInit.size())

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
		case 3:
			computeSmoothWeights();
			break;
		case 2:
			computeRigidCylindricWeights();
			break;
		case 4:
			nullWeights();
			break;
	}

	// Distance
	distanceIsCylindric = true;



	// Test skinning :
	animate();
}

void Skinning::switchDistance() {
	distanceIsCylindric = !distanceIsCylindric;
}

void Skinning::recomputeWeights() {
	if (_skin==NULL) return;
	if (_skel==NULL) return;

	// Compute weights :
	switch (_meth) {
		case 0:
			cout << "LOAD : Loading weights from file\n";
			loadWeights("data/skinning.txt");
			break;
		case 1:
			cout << "LINE : Computing linear weights\n";
			computeWeights();
			break;
		case 3:
			cout << "SMOO : Computing smooth weights " << (distanceIsCylindric?"CYLINDRIC":"EUCLIDIAN") << " distance | " << NB_COEF_SMOOTH << " dependencies\n";
			computeSmoothWeights();
			break;
		case 2:
			cout << "RIGI : Computing rigid weights (Euclidian distance)\n";
			computeRigidCylindricWeights();
			break;
		case 4:
			nullWeights();
			cout << "DISA : Disable weights\n";
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

double Skinning::euclidianDistance(unsigned int point, unsigned int joint) {
	
	unsigned int i = point;
	unsigned int j = joint;

	// Initialisation
	glm::vec4 P = _pointsInit[i];

	// T milieu de l'os
	glm::vec4 T = _posBonesInit[j];
	glm::vec4 TP = P - T;
	double TPdist = glm::length(TP);

	return TPdist;
}

double Skinning::cylindricDistance(unsigned int point, unsigned int joint) {

	unsigned int i = point;
	unsigned int j = joint;

	// Initialisation
	glm::vec4 P = _pointsInit[i];
	double PUdist, kU;
	Skeleton *skel = _joints[j];

	// T milieu de l'os
	glm::vec4 T = _posBonesInit[j];
	glm::mat4 M = _transfoInit[j];
	// S articulation
	glm::vec4 S(M[3][0],M[3][1],M[3][2],M[3][3]);
	// ST axe de l'os
	glm::vec4 ST = T - S;
	double STn = glm::length(ST);

	if (STn == 0) {
		// aucun axe, articulation seule
		// Calcul de la distance euclidienne à l'os
		PUdist = glm::distance(S, P);
	} else {
		// Projeté orthogonal du point P sur l'os ST en U
		glm::vec4 SP = P - S;
		double dotpdct = glm::dot(SP, ST);
		glm::vec4 SU = (dotpdct / STn) * (ST / STn);
		glm::vec4 U = S + SU;

		//double SUn = glm::length(SU);
		double SUn = dotpdct / STn;
		double kU = SUn / (2 * STn);

		// Calcul de la distance cylindrique = dist(P,U)
		glm::vec4 R = 2 * ST;
		if (kU < 0)
			PUdist = glm::distance(S, P);
		else if (kU > 1)
			PUdist = glm::distance(R, P);
		else
			PUdist = glm::distance(P, U);
	}

	return PUdist;
}


int Skinning::test_and_put(double coef[], double val, unsigned int indices[], unsigned int j) {
	int imax = 0;
	double max = coef[0];
	for (unsigned int i = 0; i < NB_COEF_SMOOTH; i++) {
		if (max < coef[i]) {
			imax = i;
			max = coef[i];
		}
	}
	if (val < max) {
		coef[imax] = val;
		indices[imax] = j;
		return 1;
	}
	return 0;
}

void Skinning::computeSmoothWeights() {
	if (_skin==NULL) return;
	if (_skel==NULL) return;


	for (unsigned int i = 0; i < _pointsInit.size() ; ++i) {
		double min_dist = std::numeric_limits<double>::max();
		double coef[NB_COEF_SMOOTH];
		unsigned int indices[NB_COEF_SMOOTH];

		unsigned int nbFilled = 0;

		for (unsigned int k = 0; k < NB_COEF_SMOOTH; k++) {
			coef[k] = min_dist;
			indices[k] = -1;
		}
		double coefsum = 0;

		for (unsigned int j = 0; j < _posBonesInit.size(); ++j) {
			// Initialisation
			_weights[i][j] = 0.0;
			double PUdist;

			if (distanceIsCylindric) {
				PUdist = 1/exp(-3 * cylindricDistance(i, j));
			} else {
				PUdist = 1/exp(-3 * euclidianDistance(i, j));
			}

			// Stockage de la distance
			nbFilled += test_and_put(coef, PUdist, indices, j);
		}

		if (nbFilled >= NB_COEF_SMOOTH) {
			for (unsigned int k = 0; k < NB_COEF_SMOOTH; k++)
				coefsum += 1/coef[k];
			for (unsigned int j = 0; j < NB_COEF_SMOOTH; ++j) {
				_weights[i][indices[j]] = 1/(coef[j]*coefsum);
			}
		}
	}
}

void Skinning::nullWeights() {
	if (_skin==NULL) return;
	if (_skel==NULL) return;

	for (unsigned int i = 0; i < _pointsInit.size() ; ++i) {
		for (unsigned int j = 0; j < _posBonesInit.size(); ++j) {
			_weights[i][j] = 0.0;
		}
	}
}

void Skinning::computeRigidCylindricWeights() {
	if (_skin==NULL) return;
	if (_skel==NULL) return;

	for (unsigned int i = 0; i < _pointsInit.size() ; ++i) {
		int j_min_dist = -1;
		double min_dist = std::numeric_limits<double>::max();
		double coef;
		glm::vec4 P = _pointsInit[i];

		for (unsigned int j = 0; j < _posBonesInit.size(); ++j) {
			_weights[i][j] = 0.0; // initialisation
			double PUdist, kU;

			Skeleton *skel = _joints[j];
			//if (skel->_children.size() == 0) {
			//	continue;
			//}

			// T milieu de l'os
			glm::vec4 T = _posBonesInit[j];
			glm::mat4 M = _transfoInit[j];
			// S articulation
			glm::vec4 S(M[3][0],M[3][1],M[3][2],M[3][3]);
			// ST axe de l'os
			glm::vec4 ST = T - S;
			double STn = glm::length(ST);

			if (STn == 0) {
				// aucun axe, articulation seule
				// Calcul de la distance euclidienne à l'os
				PUdist = glm::distance(S, P);
			} else {
				// Projeté orthogonal du point P sur l'os ST en U
				glm::vec4 SP = P - S;
				double dotpdct = glm::dot(SP, ST);
				glm::vec4 SU = (dotpdct / (STn * STn)) * ST;
				glm::vec4 U = S + SU;

				// Test de l'appartennance du U à l'os [ST?]
				double SUn = glm::length(SU);
				kU = SUn / (2 * STn);
				if (dotpdct < 0 || kU > 1)
					continue;

				// Calcul de la distance cylindrique = dist(P,U)
				PUdist = glm::distance(P, U);
			}

			// Mise à jour de la distance minimum
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
		//std::cerr << "Warning: Joint " << jointName << " not found" << std::endl;
		return;
	} else {
		//std::cerr << "Joint " << jointName << " found at " << i << std::endl;
		//std::cerr << _joints.size()  <<  " joints in the mesh" << std::endl;
	}

	unsigned int jointIdx = i;
	_skin->_colors.clear();
	for (unsigned int i = 0; i < _skin->_points.size() ; ++i) {
		double weight = _weights[i][jointIdx];
		_skin->_colors.push_back(glm::vec4(0.0, weight/2, weight, 0.9));
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
