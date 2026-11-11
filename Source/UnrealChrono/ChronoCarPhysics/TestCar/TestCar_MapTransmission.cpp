#include "TestCar_MapTransmission.h"
#include "Common.h"

using namespace chrono;
using namespace vehicle;


TestCar_MapTransmission::TestCar_MapTransmission(const std::string& name) : ChAutomaticTransmissionSimpleMap(name)
{

}

void TestCar_MapTransmission::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -0.15; //reverse
    
    fwd.push_back(0.157);  //1st
    fwd.push_back(0.275);  //2nd
    fwd.push_back(0.415);  //3rd
    fwd.push_back(0.588);  //4th
    fwd.push_back(0.787);  //5th
    fwd.push_back(1.0);    //6th
    fwd.push_back(1.19);   //7th
    fwd.push_back(1.4925); //8th   
}

void TestCar_MapTransmission::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) 
{
    shift_bands.push_back({1200 * rpm2rads, 6699 * rpm2rads});
    shift_bands.push_back({1300 * rpm2rads, 4800 * rpm2rads});
    shift_bands.push_back({1400 * rpm2rads, 4600 * rpm2rads});
    shift_bands.push_back({1500 * rpm2rads, 4400 * rpm2rads});
    shift_bands.push_back({1600 * rpm2rads, 4200 * rpm2rads});
    shift_bands.push_back({1700 * rpm2rads, 4000 * rpm2rads});
    shift_bands.push_back({1800 * rpm2rads, 3800 * rpm2rads});
    shift_bands.push_back({1800 * rpm2rads, 3600 * rpm2rads}); 
}
