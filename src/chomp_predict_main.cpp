#include <chomp_predict/chomp_predict.h>

int main(int argc,char *argv[]){

    ros::init(argc,argv,"chomp_predict_module");

    CHOMP::ChompForecaster chomp_predictor;    
    
    
    chomp_predictor.run(); //previous version 
    
    /**
    ros::Rate loop_rate(20);
    while(ros::ok()){
        chomp_predictor.session();
        loop_rate.sleep();
        ros::spinOnce();        
    }
    **/

    return 0;
}
