#include <iostream>
#include <fstream>
#include <cmath>
#include "igl/readOBJ.h"
#include "igl/writePLY.h"
#include <Eigen/Dense>
#include <vector>

struct Quaternion{
       double s;
       Eigen::Vector3d v;
       Eigen::Vector3d rotate(const Eigen::Vector3d x){
          return x * ( s * s - v.squaredNorm()) + 2.0 * v * v.dot(x) + 2.0 * s * v.cross(x);
          //return x*(s * s - dot(v,v)) + 2.*v*dot(v,x) + 2.*cross(v,x)*s;
       }
};

struct BodyPart{
       Eigen::MatrixXd originV;
       Eigen::MatrixXi F;
       std::vector<Eigen::RowVector3d> displacement; //displacement on each frame
       std::vector<Quaternion> rot; // rotation on each frame
};

int mot_parsing (const std::string &filename, std::vector<BodyPart> &body)
{
    std::ifstream ifs(filename);
    if(!ifs)
    {
        std::cerr << "Couldn't read options file: " << filename << std::endl;
        return 1;
    }

    std::string line;
    getline(ifs, line);
    std::stringstream s2(line);
    s2 >> line;
    int nFrames;
    s2 >> nFrames;
    std::cout << "number of frames : " << nFrames << std::endl;
    getline(ifs, line);

    for (int j = 0; j < body.size(); j++){
    	for (int i = 0; i < nFrames; i++){
             getline(ifs, line);
             std::stringstream ss(line);
             Eigen::RowVector3d transformation;
	     for (int k = 0; k < 3; k++)
                  ss >> transformation(k);
             body[j].displacement.push_back(transformation);
	}
        getline(ifs, line);
        getline(ifs, line);
    	for (int i = 0; i < nFrames; i++){
             getline(ifs, line);
             std::stringstream ss(line);
             Quaternion quaternion;
             ss >> quaternion.s;
	     for (int k = 0; k < 3; k++)
                  ss >> quaternion.v(k);
             body[j].rot.push_back(quaternion);
             
	}
        getline(ifs, line);
        getline(ifs, line);
    }
       return nFrames; 
}

void get_pos(BodyPart &part, int frameID, Eigen::MatrixXd &newPos)
{
     newPos.resize(part.originV.rows(), 3);
     for (int i = 0; i < part.originV.rows(); i++)
	  newPos.row(i) = part.rot[frameID].rotate(part.originV.row(i)).transpose() + part.displacement[frameID];
 
}

int main(int argc, char *argv[])
{   
    if (argc <= 1) {
        std::cerr << "Must pass in path to configuration file!" << std::endl;
        return 1;
    }

    std::vector<BodyPart> body; 
    for (int i = 0; i < 16; i++){
        BodyPart part;
        if (i < 10) {
            std::string inputOBJ = "BaseMesh/body000"+std::to_string(i)+".obj";
            igl::readOBJ(inputOBJ, part.originV, part.F);
        }
        else 
            igl::readOBJ("BaseMesh/body00"+std::to_string(i)+".obj", part.originV, part.F); 
        body.push_back(part); 
    }

    int nFrames = mot_parsing(argv[1], body);
   
    for (int j = 0; j < body.size(); j++){
        for (int i = 0; i < nFrames; i++){
	    Eigen::MatrixXd newPos;
            get_pos(body[j], i, newPos);
            if (j < 10) {
                std::string output = "MovingBody/body000"+std::to_string(j) + "_" + std::to_string(i)+".ply";
                igl::writePLY(output, newPos, body[j].F);
            }
            else{
                std::string output = "MovingBody/body00"+std::to_string(j) + "_" + std::to_string(i)+".ply";
                igl::writePLY(output, newPos, body[j].F);
            }
        }
    }
    
    return 0;
}
