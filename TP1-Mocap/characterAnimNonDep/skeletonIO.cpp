#include "skeletonIO.h"
#include <iostream>
#include <string>
#include <fstream>
using namespace std;

static int ntabs = 0;

Skeleton* readHierarchy(std::ifstream &inputfile)
{
	string buf;
	inputfile >> buf;

	Skeleton* skel;

	if(!buf.compare("ROOT")) {
		// Read root :
		string jointName, buf;

		// Read the joint name
		inputfile >> jointName;
		cout << jointName << endl;

		// ignore "{"
		inputfile >> buf;
		if(buf.compare("{")!=0) {
			cout << "Failed reading the file, invalid format." << endl; 
			return 0;
		}	

		// Read the offset	
		inputfile >> buf;
		if(buf.compare("OFFSET")!=0){
			cout << "Failed reading the file, invalid format." << endl; 
			return 0;
		}	
		double offsetX ;
		double offsetY ;
		double offsetZ ;
		inputfile >> offsetX ;
		inputfile >> offsetY ;
		inputfile >> offsetZ ;

		// Create the joint
		skel = Skeleton::create(jointName, offsetX, offsetY, offsetZ, 0); 

		// Read the CHANNELS
		// And add the liberty degrees to the list
		inputfile >> buf;
		if(buf.compare("CHANNELS")!=0) {
			cout << "Failed reading the file, invalid format." << endl; 
			return 0;
		}
		int nbChannels ;
		inputfile >> nbChannels ;
		for(int i = 0 ; i < nbChannels ; i++) {
			inputfile >> buf ;
			AnimCurve ac;
			ac.name = buf;
			skel->_dofs.push_back(ac);
		}

		// Create all the joints, if any
		inputfile >> buf;
		ntabs++;
		while(buf.compare("JOINT")==0) {
			readJoint(inputfile, skel);		
			inputfile >> buf;
		}
		ntabs--;
	}

	inputfile >> buf;

	if(!buf.compare("MOTION")) readMotion(inputfile, skel);

	return skel;
}
void readJoint(std::ifstream &inputfile, Skeleton* parent)
{
	string jointName, buf;

	// Read the joint name
	inputfile >> jointName;

	for (int i = 0 ; i < ntabs ; i++) cout << "   ";
	cout << jointName << endl;

	// ignore "{"
	inputfile >> buf;
	if(buf.compare("{")!=0) {
		cout << "Failed reading the file, invalid format." << endl; 
		return;
	}	

	// Read the offset	
	inputfile >> buf;
	if(buf.compare("OFFSET")!=0){
		cout << "Failed reading the file, invalid format." << endl; 
		return;
	}	
	double offsetX ;
	double offsetY ;
	double offsetZ ;
	inputfile >> offsetX ;
	inputfile >> offsetY ;
	inputfile >> offsetZ ;

	// Create the joint
	Skeleton* joint = Skeleton::create(jointName, offsetX, offsetY, offsetZ, parent); 

	// Read the CHANNELS
	// And add the liberty degrees to the list
	inputfile >> buf;
	if(buf.compare("CHANNELS")!=0) {
		cout << "Failed reading the file, invalid format." << endl; 
		return;
	}
	int nbChannels ;
	inputfile >> nbChannels ;
	for(int i = 0 ; i < nbChannels ; i++) {
		inputfile >> buf ;
		AnimCurve ac;
		ac.name = buf;
		joint->_dofs.push_back(ac);
	}

	// Create all the joints, if any
	inputfile >> buf;
	ntabs++;
	while(buf.compare("JOINT")==0) {
		readJoint(inputfile, joint);		
		inputfile >> buf;
	}
	ntabs--;

	if(! buf.compare("End")) {	
		inputfile >> buf;
		if(buf.compare("Site")!=0) {
			cout << "Failed reading the file, invalid format." << endl; 
			return;
		}

		inputfile >> buf;
		if(buf.compare("{")!=0) {
			cout << "Failed reading the file, invalid format." << endl; 
			return;
		}

		inputfile >> buf ;
		if(buf.compare("OFFSET")!=0) {
			cout << "Failed reading the file, invalid format." << endl; 
			return;
		}
		double endX ;
		double endY ;
		double endZ ;
		inputfile >> endX ;
		inputfile >> endY ;
		inputfile >> endZ ;
		Skeleton::create("End Site", endX, endY, endZ, joint);

		inputfile >> buf;
		if(buf.compare("}")!=0) {
			cout << "Failed reading the file, invalid format." << endl; 
			return;
		}
		inputfile >> buf ;
	}


	if(buf.compare("}")!=0) {
		cout << "Failed reading the file, invalid format." << endl; 
		return;
	}
}
void readMotion(std::ifstream &inputfile, Skeleton* root)
{
	string buf;

	// Read the nb of frames
	inputfile >> buf;
	if(buf.compare("Frames:")!=0) {
		cout << "Failed reading the file, invalid format." << endl; 
		return;
	}
	int nbFrames ;
	inputfile >> nbFrames;
	cout << "Nb Frames : " <<  nbFrames << endl;

	// Read the frame time
	inputfile >> buf;
	if(buf.compare("Frame")!=0) {
		cout << "Failed reading the file, invalid format." << endl; 
		return;
	}
	inputfile >> buf;
	if(buf.compare("Time:")!=0) {
		cout << "Failed reading the file, invalid format." << endl; 
		return;
	}
	double frameTime ;
	inputfile >> frameTime ;

	for(int f = 0 ; f < nbFrames ; ++f) {
		readKeyFrame(inputfile, root);
	}
}
void readKeyFrame(std::ifstream &inputfile, Skeleton* skel)
{
	// Get the values for the current frame for all the liberty degrees
	double amount = 0.0;
	for(unsigned int i = 0 ; i < skel->_dofs.size() ; ++i) {
		inputfile >> amount;
		skel->_dofs[i]._values.push_back(amount);
	}
	defineRotateOrder(skel);

	// Update the children
	for(unsigned int i = 0 ; i < skel->_children.size() ; ++i) {
		readKeyFrame(inputfile, skel->_children[i]);
	}
}
void defineRotateOrder(Skeleton *skel)
{
	int iX, iY, iZ;
	for (unsigned int idof = 0 ; idof < skel->_dofs.size() ; idof++) {
		if(!skel->_dofs[idof].name.compare("Zrotation")) iZ = idof;
		if(!skel->_dofs[idof].name.compare("Yrotation")) iY = idof;
		if(!skel->_dofs[idof].name.compare("Xrotation")) iX = idof;
	}	
	if (iX < iY && iY < iZ) skel->_rorder = roXYZ;
	if (iY < iZ && iZ < iX) skel->_rorder = roYZX;
	if (iZ < iX && iX < iY) skel->_rorder = roZXY;
	if (iX < iZ && iZ < iY) skel->_rorder = roXZY;
	if (iY < iX && iX < iZ) skel->_rorder = roYXZ;
	if (iZ < iY && iY < iX) skel->_rorder = roZYX;
}