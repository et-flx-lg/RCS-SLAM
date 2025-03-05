/* ----------------------------------------------------------------------------

 * File: DR_PGO_20bots.cpp
 * Author: Adam Hoburg
 * Date: 03 April 2024
 
 * Purpose: This code reads in a 2D pose graph in G2O format, applies direct ranging  
  constraints and performs the optimization. The communication constraints
 are stored in three different files for each of the three different conditions.
 Direct ranging data is stored in cppcomm.csv and loopclosure.csv. Relies on GTSAM library
 for pose graph formulation and optimization. Optimization is performed with Gauss Newton
 algorithm. The pose graph is actually loaded and optimized twice, producing optimized 
 estimates with and without the loop closure  condition. Optimized pose graphs are 
 saved in DR_IneqOut.csv and DR_LC_IneqOut.csv.


 * -------------------------------------------------------------------------- */


#include <gtsam/slam/dataset.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/sam/RangeFactor.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
//#include <ajh_swarm_slam/LoopClosureFactor.h>

using namespace std;
using namespace gtsam;

struct Pose {
    double x, y, theta;
};

// HOWTO: ./Pose2SLAMExample_g2o inputFile outputFile (maxIterations) (tukey/huber)
int main(const int argc, const char *argv[]) {
  string kernelType = "none";
  int maxIterations = 100;                                    // default
  string commgraph;  // default
  double commdist = 1.0;
  double mu = .01;
  Vector meas;
  Vector LCbot;
  string folder ;
    

  // Parse user's inputs
  if (argc > 1) {
    folder = argv[1];  // input dataset filename
  } else {
  	folder = "Current";
  }

  string g2oFile = "/home/robolab/catkin_ws/src/ajh_swarm_slam/optimization/Data/" + folder + "/cppgraph.csv";
  string Out1 = "/home/robolab/catkin_ws/src/ajh_swarm_slam/optimization/Data/" + folder + "/DR_IneqOut.csv"; 

  string Out2 = "/home/robolab/catkin_ws/src/ajh_swarm_slam/optimization/Data/" + folder + "/DR_LC_IneqOut.csv"; 

  std::ifstream file(g2oFile);
  if (!file.is_open()) {
  	std::cerr << "Fehler beim Öffnen der Datei!" << std::endl;
        return 1;
  }
  
  std::vector<Pose> poses;
  
  std::string line2;
  while (std::getline(file, line2)) {
        std::istringstream iss(line2);
        std::vector<std::string> tokens;
        std::string token;

        while (iss >> token) {  // Liest die Zeile in einzelne Strings ein (getrennt durch Leerzeichen)
            tokens.push_back(token);
        }

        if (tokens.size() >= 4 && tokens[0] == "VERTEX_SE2" && tokens[1] == "1000") {  // Prüft, ob es eine gültige Zeile ist
            double x = std::stod(tokens[tokens.size() - 3]);
            double y = std::stod(tokens[tokens.size() - 2]);
            double theta = std::stod(tokens[tokens.size() - 1]);

            poses.push_back({x, y, theta});
        }
        
        if (tokens.size() >= 4 && tokens[0] == "VERTEX_SE2" && tokens[1] == "2000") {  // Prüft, ob es eine gültige Zeile ist
            double x = std::stod(tokens[tokens.size() - 3]);
            double y = std::stod(tokens[tokens.size() - 2]);
            double theta = std::stod(tokens[tokens.size() - 1]);

            poses.push_back({x, y, theta});
        }
    }

    file.close();
  
  // reading file and creating factor graph
  NonlinearFactorGraph::shared_ptr graph1;
  NonlinearFactorGraph::shared_ptr graph2;
  Values::shared_ptr initial1;
  Values::shared_ptr initial2;

  bool is3D = false;
  std::tie(graph1, initial1) = readG2o(g2oFile, is3D);
  std::tie(graph2, initial2) = readG2o(g2oFile, is3D);

 
  // Add prior on the pose having index (key) = 1000
  auto priorModel =  //
      noiseModel::Diagonal::Sigmas(Vector3(.001, .001, .005));
  
  graph1->addPrior(1000, Pose2(poses[0].x, poses[0].y, poses[0].theta), priorModel);
  graph1->addPrior(2000, Pose2(poses[1].x, poses[1].y, poses[1].theta), priorModel);
  /*graph1->addPrior(3000, Pose2(-1.0,0.0,0.0), priorModel);
  graph1->addPrior(4000, Pose2(-1.0,0.5,0.0), priorModel);
  graph1->addPrior(5000, Pose2(-1.0,1.0,0.0), priorModel);
  graph1->addPrior(6000, Pose2(-0.5,-1.0,0.0), priorModel);
  graph1->addPrior(7000, Pose2(-0.5,-0.5,0.0), priorModel);
  graph1->addPrior(8000, Pose2(-0.5,0.0,0.0), priorModel);
  graph1->addPrior(9000, Pose2(-0.5,0.5,0.0), priorModel);
  graph1->addPrior(10000, Pose2(-0.5,1.0,0.0), priorModel);
  graph1->addPrior(11000, Pose2(0.0,-1.0,0.0), priorModel);
  graph1->addPrior(12000, Pose2(0.0,-0.5,0.0), priorModel);
  graph1->addPrior(13000, Pose2(0.0,0.0,0.0), priorModel);
  graph1->addPrior(14000, Pose2(0.0,0.5,0.0), priorModel);
  graph1->addPrior(15000, Pose2(0.0,1.0,0.0), priorModel);
  graph1->addPrior(16000, Pose2(0.5,-1.0,0.0), priorModel);
  graph1->addPrior(17000, Pose2(0.5,-0.5,0.0), priorModel);
  graph1->addPrior(18000, Pose2(0.5,0.0,0.0), priorModel);
  graph1->addPrior(19000, Pose2(0.5,0.5,0.0), priorModel);
  graph1->addPrior(20000, Pose2(0.5,1.0,0.0), priorModel);*/
  graph1->addPrior(21000, Pose2(0.0,0.0,0.0), priorModel);

  graph2->addPrior(1000, Pose2(poses[0].x, poses[0].y, poses[0].theta), priorModel);
  graph2->addPrior(2000, Pose2(poses[1].x, poses[1].y, poses[1].theta), priorModel);
  /*graph2->addPrior(3000, Pose2(-1.0,0.0,0.0), priorModel);
  graph2->addPrior(4000, Pose2(-1.0,0.5,0.0), priorModel);
  graph2->addPrior(5000, Pose2(-1.0,1.0,0.0), priorModel);
  graph2->addPrior(6000, Pose2(-0.5,-1.0,0.0), priorModel);
  graph2->addPrior(7000, Pose2(-0.5,-0.5,0.0), priorModel);
  graph2->addPrior(8000, Pose2(-0.5,0.0,0.0), priorModel);
  graph2->addPrior(9000, Pose2(-0.5,0.5,0.0), priorModel);
  graph2->addPrior(10000, Pose2(-0.5,1.0,0.0), priorModel);
  graph2->addPrior(11000, Pose2(0.0,-1.0,0.0), priorModel);
  graph2->addPrior(12000, Pose2(0.0,-0.5,0.0), priorModel);
  graph2->addPrior(13000, Pose2(0.0,0.0,0.0), priorModel);
  graph2->addPrior(14000, Pose2(0.0,0.5,0.0), priorModel);
  graph2->addPrior(15000, Pose2(0.0,1.0,0.0), priorModel);
  graph2->addPrior(16000, Pose2(0.5,-1.0,0.0), priorModel);
  graph2->addPrior(17000, Pose2(0.5,-0.5,0.0), priorModel);
  graph2->addPrior(18000, Pose2(0.5,0.0,0.0), priorModel);
  graph2->addPrior(19000, Pose2(0.5,0.5,0.0), priorModel);
  graph2->addPrior(20000, Pose2(0.5,1.0,0.0), priorModel);*/
  graph2->addPrior(21000, Pose2(0.0,0.0,0.0), priorModel);
  

  std::cout << "Adding priors " << std::endl;
  
  fstream Commin;
  fstream Hopin;
  fstream LCin;
  

  Commin.open("/home/robolab/catkin_ws/src/ajh_swarm_slam/optimization/Data/" + folder + "/cppcomm.csv", ios::in);
  
  LCin.open("/home/robolab/catkin_ws/src/ajh_swarm_slam/optimization/Data/" + folder + "/loopclosure.csv", ios::in);
  
  vector<string> row;
  string line, word, temp;
  
  auto commmodel = noiseModel::Isotropic::Sigma(1, 0.4);
  auto LCmodel = noiseModel::Isotropic::Sigma(1, 0.4);//was (1, 0.6 from adam)
  //auto LCmodel = noiseModel::Diagonal::Sigmas(Vector3(.5, .5, .5));

  
        
  while (getline(Commin,line)) {
     
    stringstream s(line);
    
     
    while(getline(s,word, ' ')){
      row.push_back(word);
      }
          
    Key key1 = stoi(row[0]);
    Key key2 = stoi(row[1]);
    float rng = stof(row[2]);

    
    Pose2 k1_1 = initial1->at<Pose2>(key1);
    Pose2 k2_1 = initial1->at<Pose2>(key2);

    Pose2 k1_2 = initial2->at<Pose2>(key1);
    Pose2 k2_2 = initial2->at<Pose2>(key2);

    
    double delta_x1 = k1_1.x() - k2_1.x();
    double delta_y1 = k1_1.y() - k2_1.y();
    double range1 = (sqrt(pow(delta_x1,2) + pow(delta_y1,2)));

    double delta_x2 = k1_2.x() - k2_2.x();
    double delta_y2 = k1_2.y() - k2_2.y();
    double range2 = (sqrt(pow(delta_x2,2) + pow(delta_y2,2)));
    
    graph1->add(RangeFactor<Pose2, Pose2>(key1,key2, rng, commmodel));
    
    graph2->add(RangeFactor<Pose2, Pose2>(key1,key2, rng, commmodel));
    	    	

    
    row.clear();
    
  }
  
  
  while (getline(LCin,line)) {
     
    stringstream s(line);
    
    
    
    while(getline(s,word, ' ')){
      row.push_back(word);
      }
     
         
    
    Key key1 = stoi(row[0]);
    Key key2 = stoi(row[1]);
    
    
    
    std::cout << "Parsed keys: " << key1 << ", " << key2 << std::endl;

    
    Pose2 k1 = initial2->at<Pose2>(key1);
    Pose2 k2 = initial2->at<Pose2>(key2);
    float rng2 = stof(row[2]);

    
    
    double delta_x = k1.x() - k2.x();
    double delta_y = k1.y() - k2.y();
    double range = (sqrt(pow(delta_x,2) + pow(delta_y,2))); 
    
    double x = k2.x();
    double y = k2.y();
    
    graph2->add(RangeFactor<Pose2, Pose2>(key1,key2, rng2, LCmodel));
       
    row.clear();
    
  } 
 
 
  
  GaussNewtonParams params;
  params.setVerbosity("TERMINATION");
  if (argc > 3) {
    params.maxIterations = maxIterations;
    std::cout << "User required to perform maximum  " << params.maxIterations
              << " iterations " << std::endl;
  }

  std::cout << "Optimizing the factor graph" << std::endl;
  GaussNewtonOptimizer optimizer1(*graph1, *initial1, params);
  Values result1 = optimizer1.optimize();

  GaussNewtonOptimizer optimizer2(*graph2, *initial2, params);
  Values result2 = optimizer2.optimize();

  
  std::cout << "Optimization complete" << std::endl;

  

  result1.print("result");
  result2.print("result");
  
  const string outputFile1 = Out1;
  const string outputFile2 = Out2;

  std::cout << "Writing results." << std::endl;
  NonlinearFactorGraph::shared_ptr graphNoKernel1;
  Values::shared_ptr Initial1;
  std::tie(graphNoKernel1, Initial1) = readG2o(g2oFile);
  writeG2o(*graphNoKernel1, result1, outputFile1);
  std::cout << "Initial error=" << graph1->error(*initial1) << std::endl;
  std::cout << "Final error=" << graph1->error(result1) << std::endl;

  NonlinearFactorGraph::shared_ptr graphNoKernel2;
  Values::shared_ptr Initial2;
  std::tie(graphNoKernel2, Initial2) = readG2o(g2oFile);
  writeG2o(*graphNoKernel2, result2, outputFile2);
  std::cout << "Initial error with LC=" << graph2->error(*initial2) << std::endl;
  std::cout << "Final error with LC=" << graph2->error(result2) << std::endl;

  std::cout << "done! " << std::endl;

  return 0;
}
