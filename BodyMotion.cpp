#include <iostream>
#include <fstream>
#include <cmath>
#include "igl/readOBJ.h"
#include "igl/writeOBJ.h"
#include <Eigen/Dense>
#include <vector>

struct BodyPart{
       Eigen::MatrixXd originV;
       Eigen::MatrixXi F;
       std::vector<Eigen::Vector3d> displacement; //displacement on each frame
       std::vector<Eigen::Vector4d> rot; // rotation on each frame
};

class Body{
	public : void addPart (BodyPart * newPart){
			bodyparts.push_back(newPart);
		 }
		 
		 ~Body(){
			for (auto it : bodyparts)
				delete it;
		 }
	         
		 std::vector<BodyPart *> bodyparts;
};

int mot_parsing (const std::string &filename, std::vector<BodyPart> &body)
{
    std::ifstream ifs(filename);
    if(!ifs)
    {
        std::cerr << "Couldn't read options file: " << filename << std::endl;
        return;
    }

    std::string line;
    ifs >> line;
    int nFrames;
    ifs >> nFrames;
    std::cout << "number of frames : " << nFrames << std::endl;
    getline(ifs, line);

    for (int j = 0; j < body.size(); j++){
    	for (int i = 0; i < nFrames; i++){
             Eigen::Vector3d transformation;
	     for (int k = 0; k < 3; k++)
                  ifs >> transformation(k);
             body[j].displacement.push_back(transformation);
	}
        getline(ifs, line);
        getline(ifs, line);
    	for (int i = 0; i < nFrames; i++){
             Eigen::Vector4d quaternion;
	     for (int k = 0; k < 4; k++)
                  ifs >> quaternion(k);
             body[j].rot.push_back(quaternion);
	}
    }
        
}

void get_pos(BodyPart &part, int frameID, Eigen::MatrixXd newPos)
{
    
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

    int nFrame = mot_parsing(argv[1], body);
   
    for (int j = 0; j < body.size(); j++){
        for (int i = 0; i < nFrame, i++){
            if (i < 10) {
                std::string output = "MovingBody/body000"+std::to_string(j)_std::to_string(i)+".ply";
		Eigen::MatrixXd newPos;
                get_pos(body[j], i, newPos);
                igl::writePLY(output, newPos, body[j].F);
            }
            else
        }
    }
    
    return 0;
}
