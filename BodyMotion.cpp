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

Quaternion slerp(Quaternion q0, Quaternion q1, double t) {
    // Compute the cosine of the angle between the two vectors.
    Eigen::Vector4d v0, v1;
    v0 << q0.s, q0.v(0), q0.v(1), q0.v(2);
    v1 << q1.s, q1.v(0), q1.v(1), q1.v(2);

    double dot = v0.dot(v1);

    if (dot < 0.0f) {
        v1 = -v1;
        dot = -dot;
    }  

    const double DOT_THRESHOLD = 0.9995;
    if (dot > DOT_THRESHOLD) {

        Eigen::Vector4d result = v0 + t*(v1 - v0);
        result = result.normalized();
       
        Quaternion ans;
	ans.s = result(0);
        for (int i = 0; i < 3; i++)
	     ans.v(i) = result(i+1);
        return ans;
    }

    // Since dot is in range [0, DOT_THRESHOLD], acos is safe
    double theta_0 = acos(dot);        // theta_0 = angle between input vectors
    double theta = theta_0*t;          // theta = angle between v0 and result
    double sin_theta = sin(theta);     // compute this value only once
    double sin_theta_0 = sin(theta_0); // compute this value only once

    double s0 = cos(theta) - dot * sin_theta / sin_theta_0;  // == sin(theta_0 - theta) / sin(theta_0)
    double s1 = sin_theta / sin_theta_0;

    Eigen::Vector4d result = (s0 * v0) + (s1 * v1);
    Quaternion ans;
    ans.s = result(0);
    for (int i = 0; i < 3; i++)
         ans.v(i) = result(i+1);
    return ans;
}

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

void get_interpolatedPos(BodyPart &part, int frameID, double elapsetime,  Eigen::MatrixXd &newPos)
{
     newPos.resize(part.originV.rows(), 3);
     Quaternion x = part.rot[frameID];
     std::cout << x.s << " " << x.v(0) << " " << x.v(1) << " " << x.v(2) << std::endl;
     x = part.rot[frameID + 200];
     std::cout << x.s << " " << x.v(0) << " " << x.v(1) << " " << x.v(2) << std::endl;
     Eigen::RowVector3d diff = part.displacement[frameID + 200] - part.displacement[frameID];
     Quaternion q = slerp(part.rot[frameID], part.rot[frameID + 200], elapsetime);
     std::cout <<"new quaternion : " << std::endl << q.s << " " << q.v(0) << " " << q.v(1) << " " << q.v(2) << std::endl;
     std::cout <<"diff : " << diff(0) << " " << diff(1) << " " << diff(2) << std::endl;
     for (int i = 0; i < part.originV.rows(); i++){
	  newPos.row(i) = q.rotate(part.originV.row(i)).transpose() + part.displacement[frameID] + elapsetime * diff;
     }
}

void get_pos(BodyPart &part, int frameID, Eigen::MatrixXd &newPos)
{
     newPos.resize(part.originV.rows(), 3);
     for (int i = 0; i < part.originV.rows(); i++){
	  newPos.row(i) = part.rot[frameID].rotate(part.originV.row(i)).transpose() + part.displacement[frameID];
     }
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

    int frameInterval = 5;
    for(int i = 0; i < body.size(); i++){
        for(int j = 0; j <6 ; j++){
            Eigen::MatrixXd newPos;
            get_interpolatedPos(body[i], 0, (double)j/frameInterval, newPos);
            std::string output = "MovingBody/" + std::to_string(j) + "EnlargedBody" +std::to_string(i) + ".ply";
            igl::writePLY(output, newPos, body[i].F);
        }
    }
   
    //for (int j = 0; j < body.size(); j++){
    //    body[j].originV *= 1.1;
    //    for (int i = 0; i < 5; i++){
    //        Eigen::MatrixXd newPos;
    //        get_pos(body[j], i * 20, newPos);
    //        std::string output = "MovingBody/" + std::to_string(i) + "KeyFrameBody" +std::to_string(j) + ".ply";
    //        igl::writePLY(output, newPos, body[j].F);
    //    }
    //}
    
    return 0;
}
