#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/numericalDerivative.h>

namespace gtsam{


  double range_trait1(const Pose2& a, const Pose2& b) { 
    double delta_x = a.x() - b.x();
    double delta_y = a.y() - b.y();
    
    double range = (sqrt(pow(delta_x,2) + pow(delta_y,2)))-1;
        
    return range; 
    };
    
  double node_dist(const Pose2& a, const Pose2& b) { 
    double delta_x = a.x() - b.x();
    double delta_y = a.y() - b.y();
    
    double dist = (sqrt(pow(delta_x,2) + pow(delta_y,2)));
        
  return dist; 
  };

class CommFactor: public gtsam::NoiseModelFactor1<Pose2> {

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

  CommFactor(Key pose1, Pose2 pose2, const double& c, SharedNoiseModel model, const double& range):
  		gtsam::NoiseModelFactor1<gtsam::Pose2>(model, pose1), p_(pose2), comm_(c), r_(range){}
  
  // error function
  Vector evaluateError(const Pose2& X1, OptionalMatrixType J1) const override {
    
    
    
    Vector error(1);

    //error <<  mu*pow((range-comm_),2) ;
    //error <<  sqrt(mu)*(range-comm_) ;

     
    //double dist = node_dist(X1,p_);
    double err = r_ - comm_;
     
    error << err;
     
    if (J1) *J1 = (Matrix13() << 2*(X1.x() - p_.x()) * (1-(comm_/r_)) , 2*(X1.y() - p_.y()) * (1-(comm_/r_)), 0).finished();
    
	//if (J1) *J1 = (Matrix13() << 2*(X1.x() - p_.x()) * (1/r_) , 2*(X1.y() -p_.y() ) * (1/r_), 0).finished();
    
    

      
                  
   
  //std::cout << X1 << std::endl;
  //if (err <= 0.0){
    //error << (0.0);
    //if (J1) *J1 = (Matrix13() << 0 , 0, 0).finished();


    //} 
  
  return error;
  
  }
};
};
