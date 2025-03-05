#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/numericalDerivative.h>
using namespace gtsam;

namespace gtsam{

    
  double node_d(const Pose2& a, const Pose2& b) { 
    double delta_x = a.x() - b.x();
    double delta_y = a.y() - b.y();
    
    double dist = (sqrt(pow(delta_x,2) + pow(delta_y,2)));
        
  return dist; 
  };

class LoopClosure: public gtsam::NoiseModelFactor1<Pose2> {

private:
  double comm_;
  int mu = .5;
  const double x_;
  const double y_;
  const double r_;
  
  
public:
  //float error;

  /**
   * Constructor
   * @param pose1	rob1 pose
   * @param pose2	rob2 pose
   * @param model	noise model for constrained factor 
   * @param c		comm distance
   */

  LoopClosure(Key pose1, double x, double y, const double& c, SharedNoiseModel model, const double& range):
  		gtsam::NoiseModelFactor1<gtsam::Pose2>(model, pose1), x_(x), y_(y), comm_(c), r_(range){}
  
  // error function
  Vector evaluateError(const Pose2& X1, OptionalMatrixType J1) const override {

    Vector error(1);
    
     
    //double dist = node_d(X1,p_);
     
    error << r_ - comm_;
    
    //if (J1) *J1 = (Matrix13() << 2*(p_.x() - X1.x()) * (1-(1/dist)) , 2*(p_.y() -X1.y() ) * (1-(1/dist)), 0).finished();
    
    if (J1) *J1 = (Matrix13() << 2*(x_ - X1.x() ) * (1/r_) , 2*(y_ -X1.y() ) * (1/r_), 0).finished();
 

      
                  
    
/*
  if (err <= 0.0){
    error << (0.0);
    if (J1) *J1 = (Matrix13() << 0 , 0, 0).finished();
    }
   
 */    
  return error;
  }
};
};
