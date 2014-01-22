#ifdef WIN32
	#include <windows.h>
#endif
#include <GL/gl.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <stdlib.h>

#include "mesh.h"
#include "dosunix_bind.h"

void Mesh::draw() 
{
	bool normalsOk = (_normals.size()==_points.size());
	bool colorsOk = (_colors.size()==_points.size());
	glEnable(GL_NORMALIZE);
	glColor4f(_color.x, _color.y, _color.z, _color.w);
	switch (_nbEdges) {
		case 3 :
			glBegin(GL_TRIANGLES);
			for (unsigned int i = 0 ; i < _triangles.size() ; i++) {
				if (normalsOk) glNormal3f(_normals[_triangles[i]].x,_normals[_triangles[i]].y,_normals[_triangles[i]].z);
				if (colorsOk) glColor3f(_colors[_triangles[i]].x,_colors[_triangles[i]].y,_colors[_triangles[i]].z);
				glVertex3d(_points[_triangles[i]].x,_points[_triangles[i]].y,_points[_triangles[i]].z);
			}
			glEnd();
			break;
		case 4 :
			glBegin(GL_QUADS);
			for (unsigned int i = 0 ; i < _triangles.size() ; i++) {
				if (normalsOk) glNormal3f(_normals[_triangles[i]].x,_normals[_triangles[i]].y,_normals[_triangles[i]].z);
				glVertex3d(_points[_triangles[i]].x,_points[_triangles[i]].y,_points[_triangles[i]].z);
			}
			glEnd();
			break;
	}

}

void update_string(char *str) {
    char *p = str;
    while(*p != '\0') {
        if(*p == '.')
            *p = ',';
        p++;
    }
}

void Mesh::load(const char* fileName) {
	_points.clear();
	_normals.clear();
	_faces.clear();
	_triangles.clear();
	_nbEdges = 0;

	FILE *fdat; fopen_r_dosunix(fdat, fileName, "r");
	
	char line[300];
	char *next_token;
	while(fgets(line, 300, fdat)) {
		if (line[0]=='#') {
			//std::cout << "COMMENT :: " << line;
			continue;
		}
		if (strstr(line, "g ")) {
			//std::cout << "G       :: " << line;
			continue;
		} else {
			//std::cout << "        :: " << line;
			char* key = strtok_r(line, " \t\n\r", &next_token);
			if (!key) {
				//std::cout << "  > Empty" << std::endl;
				continue;
			}
			if (key[0]=='v' && key[1]=='n') {
				glm::vec3 vi;
				char* valx = strtok_r(NULL, " \t\n\r", &next_token);// update_string(valx);
				char* valy = strtok_r(NULL, " \t\n\r", &next_token);// update_string(valy);
				char* valz = strtok_r(NULL, " \t\n\r", &next_token);// update_string(valz);
				vi.x = (double)atof(valx);
				vi.y = (double)atof(valy);
				vi.z = (double)atof(valz);
				_normals.push_back(vi);
				//std::cout << "  > ascii  " << valx << " " << valy << " " << valz << std::endl;
				//std::cout << "  > double " << x << " " << y << " " << z << std::endl;
				//std::cout << "  > vector " << vi.x << " " << vi.y << " " << vi.z << std::endl;
			} else if (key[0]=='v' && key[1]=='t') {
				//std::cout << "  > vt" << std::endl;
			} else if (key[0]=='v') {
				glm::vec4 vi(0,0,0,1);
				char* valx = strtok_r(NULL, " \t\n\r", &next_token); // update_string(valx);
				char* valy = strtok_r(NULL, " \t\n\r", &next_token); // update_string(valy);
				char* valz = strtok_r(NULL, " \t\n\r", &next_token); // update_string(valz);
				vi.x = (double)atof(valx);
				vi.y = (double)atof(valy);
				vi.z = (double)atof(valz);
				_points.push_back(vi);
				//std::cout << "  > autre v " << valx << " " << valy << " " << valz << std::endl;
			} else if (key[0]=='f') {
				char* val = strtok_r(NULL, " \t\n\r", &next_token);
				std::vector<unsigned int> face;
				for(;val!=NULL;val = strtok_r(NULL, " \t\n\r", &next_token)) {
					unsigned int iv[3] = { 0, 0, 0 };	// 0:vtx, 1:tex, 2:nrm
					if (sscanf_dosunix(val, "%d/%d/%d", iv+0,iv+1,iv+2)==3) {
						_triangles.push_back(iv[0]-1);
						face.push_back(iv[0]-1);
						//std::cout << "  > 3 " << iv[0] << " " << iv[1] << " " << iv[2] << std::endl;
					} else if (sscanf_dosunix(val, "%d/%d", iv+0,iv+1)==2) {
						_triangles.push_back(iv[0]-1);
						face.push_back(iv[0]-1);
						//std::cout << "  > 2 " << iv[0] << " " << iv[1] << std::endl;
					} else if (sscanf_dosunix(val, "%d//%d", iv+0,iv+2)==2) {
						_triangles.push_back(iv[0]-1);
						face.push_back(iv[0]-1);
						//std::cout << "  > 2 " << iv[0] << " " << iv[2] << std::endl;
					} else if (sscanf_dosunix(val, "%d", iv+0)==1) {
						_triangles.push_back(iv[0]-1);
						face.push_back(iv[0]-1);
						//std::cout << "  > 1 " << iv[0] << std::endl;
					}
				}
				_faces.push_back(face);
				_nbEdges = face.size();
			}
		}
	}
	fclose(fdat);
}
