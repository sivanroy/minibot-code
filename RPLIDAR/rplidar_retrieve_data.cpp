#include "RPLidar_lib/include/rplidar_driver.h"
#include "RPLidar_lib/include/sl_lidar.h"
#include "RPLidar_lib/include/sl_lidar_driver.h"
#include <assert.h>

/*give a pointer in which store the data
*
*/

 void create_lidar_instance(){
 	///  Create a communication channel instance
    //sl::IChannel* _channel;
    sl::Result<sl::IChannel*> channel = sl::createSerialPortChannel("/dev/ttyUSB0", 256000);//check baudrate
    assert((bool)channel);
    assert(*channel);

    ///  Create a LIDAR driver instance
  	auto lidar = sl::createLidarDriver();
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
    lidar->startMotor();

    //start scan
    std::vector<LidarScanMode> scanModes;
    lidar_drv->getAllSupportedScanModes(scanModes);

    lidar->startScan(false, true, 0, &scanMode); //typical scan mode

    //grab scan data

    // TODO

    lidar->stopMotor();	
    /// Delete Lidar Driver and channel Instance
    * delete *lidar;
    * delete *channel;
}