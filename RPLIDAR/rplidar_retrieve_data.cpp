//#include "RPLidar_lib/include/rplidar_driver.h"
/*give a pointer in which store the data
*
*/
#include "RPLidar_lib/include/sl_lidar.h"
#include "RPLidar_lib/include/sl_lidar_driver.h"
#include <assert.h>
#include <iostream>

int main() {
    std::cout << "Hello \n";
    return 0;
}
using namespace sl;
void lid(){
 	///  Create a communication channel instance
    //sl::IChannel* _channel;
    Result<IChannel*> channel = createSerialPortChannel("/dev/ttyUSB0", 256000);//check baudrate
    assert((bool)channel);
    assert(*channel);

    ///  Create a LIDAR driver instance
  	auto lidar = createLidarDriver();
    assert((bool)lidar);
    assert(*lidar); 

    auto res = (*lidar)->connect(*channel);
    assert(SL_IS_OK(res));

    sl_lidar_response_device_info_t deviceInfo;
    res = (*lidar)->getDeviceInfo(deviceInfo);
    assert(SL_IS_OK(res));
    
    if(SL_IS_OK(res)){
        sl_lidar_response_device_info_t deviceInfo;
        res = (*lidar)->getDeviceInfo(deviceInfo);
        if(SL_IS_OK(res)){
            printf("Model: %d, Firmware Version: %d.%d, Hardware Version: %d\n",
            deviceInfo.model, 
            deviceInfo.firmware_version >> 8, deviceInfo.firmware_version & 0xffu,
            deviceInfo.hardware_version);
        }else{
            fprintf(stderr, "Failed to get device information from LIDAR %08x\r\n", res);
        }
    }else{
        fprintf(stderr, "Failed to connect to LIDAR %08x\r\n", res);
    }
    //start motor
    //lidar->startMotor();

    //start scan
    //std::vector<LidarScanMode> scanModes;
    //lidar_drv->getAllSupportedScanModes(scanModes);

    LidarScanMode scanMode;//normal scan

    (*lidar)->startScan(false, true, 0, &scanMode);

    //grab scan data
    //sl_lidar_response_measurement_node_hq_t nodes[8192];
    //size_t nodeCount = sizeof(nodes)/sizeof(sl_lidar_response_measurement_node_hq_t);
    /*
    res = lidar->grabScanDataHq(nodes, nodeCount);

    if (IS_FAIL(res))
    {
        int i = 0;// failed to get scan data
    }
    */

    // TODO

    //lidar->stopMotor();	
    /// Delete Lidar Driver and channel Instance
    delete *lidar;
    delete *channel;
}


 