#include <RPLidarC1M1.h>

RPLidarC1M1 lidar;

void setup() {
  Serial.begin(115200);
  lidar.begin(Serial1);

  delay(1000);

  rplidar_response_device_info_t info;
  if (IS_OK(lidar.getDeviceInfo(info, 100))) {
      Serial.println("Device OK");
      lidar.reset();
      delay(1000);
      lidar.startScan();
  }
}

void loop() {
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
    bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
    byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement
    
    Serial.println(String(distance));
  }
}
