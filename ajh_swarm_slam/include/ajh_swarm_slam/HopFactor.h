#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/numericalDerivative.h>


namespace gtsam{

//template<class T1, class T2>
double range_trait_1(const Pose2& a, const Pose2& b) { 
  double delta_x = a.x() - b.x();
  double delta_y = a.y() - b.y();
    
  double range = (sqrt(pow(delta_x,2) + pow(delta_y,2))) - 1;
        
  return range; 
  };
  
double range_trait2(const Pose2& a, const Pose2& b) { 
  double delta_x = a.x() - b.x();
  double delta_y = a.y() - b.y();
    
  double range = (sqrt(pow(delta_x,2) + pow(delta_y,2))) - 2;
        
  return range; 
  };
  
  
double node_dist1(const Pose2& a, const Pose2& b) { 
  double delta_x = a.x() - b.x();
  double delta_y = a.y() - b.y();
    
  double dist = (sqrt(pow(delta_x,2) + pow(delta_y,2)));
        
  return dist; 
  };

class HopFactor: public gtsam::NoiseModelFactor1<Pose2> {

private:
  double comm_;
  int mu = .5;
  const Pose2 p_;
  double r_;
  

  
  
public:
  
  //float error;

  /**
   * Constructor
   * @param pose1	rob1 pose
   * @param pose2	rob2 pose
   * @param model	noise model for constrained factor 
   * @param c		comm distance
   */

  HopFactor(Key pose1, Pose2 pose2, const double& c, SharedNoiseModel model, const double& range):
  		gtsam::NoiseModelFactor2<gtsam::Pose2>(model, pose1), p_(pose2), comm_(c), r_(range){}
  
  // error function
  Vector evaluateError(const Pose2& X1, OptionalMatrixType J2) const override{
    
    
    
    Vector error(1);
    
    //double dist = node_dist1(X1,p_);
    //double err = range_trait2(X1,p_);
    //error << err;
    

  if (r_ > 2*comm_){
    
    error << r_ - 2*comm_;
    

  } else if (r_ < comm_){
  
    error << r_ - comm_;
    
  }
  
  if (J2) *J2 = (Matrix13() << 2*(p_.x() - X1.x()) * (1/r_) , 2*(p_.y() -X1.y() ) * (1/r_), 0).finished();
  
  return error;
    
  }
};
};


                      
   
