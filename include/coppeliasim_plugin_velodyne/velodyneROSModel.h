#pragma once

#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>

class CVelodyneROSModel
{
  public:
    CVelodyneROSModel(const int visionSensorHandles[4], float frequency, int options, float pointSize,
                      float coloringDistances[2], float scalingFactor, int local_frame_handle, std::string frameId,
                      std::string topic);

    virtual ~CVelodyneROSModel();

    int getVelodyneHandle();

    bool areVisionSensorsExplicitelyHandled();

    bool doAllObjectsExistAndAreVisionSensors();

    bool handle(float dt);

    void addPointsToBuffer(std::vector<float> &pts, sensor_msgs::PointCloud2 &buff);

  private:

    int _visionSensorHandles[4];
    float _frequency;
    int _velodyneHandle;
    bool _cartesianCoords;
    float lastScanAngle;
    int _local_frame_handle;

    sensor_msgs::PointCloud2 _buffer;
    float _RANGE;
    ros::Publisher _pubVelodyne;

    static int _nextVelodyneHandle;
};
