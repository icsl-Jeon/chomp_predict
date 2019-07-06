#include <chomp_predict/chomp_predict.h>

int main(int argc,char *argv[]){

    ros::init(argc,argv,"chomp_predict_module");

    CHOMP::ChompForecaster chomp_predictor;    
    chomp_predictor.run();
    
    return 0;
}