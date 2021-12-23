#include "../include/coppeliasim_plugin_velodyne/velodyneROSModel.h"
#include "../include/v_repLib.h"
#include <math.h>
#include "../include/coppeliasim_plugin_velodyne/ros_server_velodyne.h"

#define PI_VAL (3.14159265f)

// if you want to use /velodyne_points topic locally, put it true
#define _B_LOCALLY true

int CVelodyneROSModel::_nextVelodyneHandle=0;



CVelodyneROSModel::CVelodyneROSModel(const int visionSensorHandles[4],float frequency,int options,float pointSize,float coloringDistances[2],float scalingFactor,int newPointCloudHandle, int local_frame_handle)
{
    for (int i=0;i<4;i++)
        _visionSensorHandles[i]=visionSensorHandles[i];


    _frequency=frequency;
    _displayScalingFactor=1.0f;
    _displayPts=(options&1)==0;
    _displayOnlyCurrent=(options&2)!=0;
    _cartesianCoords=(options&4)==0;
    _emissivePts=(options&8)!=0;
    _pointSize=pointSize;
    _displayScalingFactor=scalingFactor;
    _ptCloudHandle=-1;
    _newPtCloudHandle=newPointCloudHandle;
    _coloringDistances[0]=coloringDistances[0];
    _coloringDistances[1]=coloringDistances[1];
    lastScanAngle=0.0f;
    _velodyneHandle=_nextVelodyneHandle++;

    _local_frame_handle = local_frame_handle;


    _RANGE=0;
    _pubVelodyne =ROS_server::getPublisher();

    //initialize the fixed fields of the output PointCloud2 message
    //_buffer.header.frame_id="espeleo_robo/os1_sensor";
    _buffer.header.frame_id="lidar";
    _buffer.height=1; //unordered data
    _buffer.fields.resize(3); //convert x/y/z to fields
    _buffer.fields[0].name = "x"; _buffer.fields[1].name = "y"; _buffer.fields[2].name = "z";
    int offset=0;
    for (size_t d =0; d < _buffer.fields.size(); ++d, offset +=4)
    {
        _buffer.fields[d].offset = offset;
        _buffer.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
        _buffer.fields[d].count = 1;
    }
    _buffer.point_step = offset;
    _buffer.is_bigendian = false;
    _buffer.is_dense = false;

}

CVelodyneROSModel::~CVelodyneROSModel()
{
    if (_ptCloudHandle>=0)
        simModifyPointCloud(_ptCloudHandle,0,0,0);
    if (_newPtCloudHandle>=0)
        simRemovePointsFromPointCloud(_newPtCloudHandle,0,0,0,0.0,0);
}

int CVelodyneROSModel::getVelodyneHandle()
{
    return(_velodyneHandle);
}

bool CVelodyneROSModel::areVisionSensorsExplicitelyHandled()
{
    for (int i=0;i<4;i++)
    {
        int r=simGetExplicitHandling(_visionSensorHandles[i]);
        if (r==-1)
            return(false);
        if ((r&1)==0)
            return(false);
    }
    return(true);
}

bool CVelodyneROSModel::doAllObjectsExistAndAreVisionSensors()
{
    for (int i=0;i<4;i++)
    {
        if (simGetObjectType(_visionSensorHandles[i])!=sim_object_visionsensor_type)
            return(false);
    }
    if (_newPtCloudHandle!=-1)
    {
        if (simGetObjectType(_newPtCloudHandle)!=sim_object_pointcloud_type)
            return(false);
        float maxVoxelS;
        int maxPtsPerVoxel;
        int opt;
        float ptS;
        simGetPointCloudOptions(_newPtCloudHandle,&maxVoxelS,&maxPtsPerVoxel,&opt,&ptS,0);
        opt|=16;
        opt-=16;
        if (_emissivePts)
            opt|=16;
        simSetPointCloudOptions(_newPtCloudHandle,maxVoxelS,maxPtsPerVoxel,opt,_pointSize,0);
    }
    return(true);
}

bool CVelodyneROSModel::handle(float dt)
{
    std::vector<float> pts ;
    //pts.clear();
    bool retVal=true;
    if (doAllObjectsExistAndAreVisionSensors()&&areVisionSensorsExplicitelyHandled())
    {
        float scanRange=_frequency*dt*2.0f*PI_VAL;
        _RANGE+=scanRange; //total swept angle
        float startAnglePlusMinusPi=lastScanAngle-PI_VAL;
        if (scanRange>=2.0f*PI_VAL)
            scanRange=2.0f*PI_VAL;
        if (_displayPts)
            _removePointsBetween(startAnglePlusMinusPi,scanRange);
        float quadrantsLowLimits[8]={-0.25f*PI_VAL,0.25f*PI_VAL,0.75f*PI_VAL,-0.75f*PI_VAL}; //{-45º,45º,135º,-135º}

        float mainSensTr[12];
        float mainSensTrInv[12];
        simGetObjectMatrix(_visionSensorHandles[0],-1,mainSensTr);
        simGetObjectMatrix(_visionSensorHandles[0],-1,mainSensTrInv);
        simInvertMatrix(mainSensTrInv);

        // get pointcloud locally
        float local_frame_mat[12];
        float local_frame_mat_inverse[12];
        simGetObjectMatrix(_local_frame_handle, -1, local_frame_mat);
        simGetObjectMatrix(_local_frame_handle, -1, local_frame_mat_inverse);
        simInvertMatrix(local_frame_mat_inverse);

        if (_ptCloudHandle>=0)
            simModifyPointCloud(_ptCloudHandle,0,0,0);
        if (_newPtCloudHandle>=0)
            simRemovePointsFromPointCloud(_newPtCloudHandle,0,0,0,0.0,0);
        _ptCloudHandle=-1;
        int existingDisplayPointsSize=int(_displayPtsXyz.size());
        for (int i=0;i<4;i++)
        {
            bool doIt=false;
            float dal=scanRange/8.0f;
            float quadrantL=quadrantsLowLimits[i];
            for (int ml=0;ml<8;ml++)
            {
                float ll=startAnglePlusMinusPi+dal*float(ml);
                if (ll>=PI_VAL)
                    ll-=2.0f*PI_VAL;
                if (   ((ll>=quadrantL)&&(ll<quadrantL+PI_VAL*0.5f)) || ((ll<=quadrantL)&&(ll<quadrantL-1.5f*PI_VAL))   )
                {
                    doIt=true;
                    break;
                }
            }
            if (doIt)
            {
                float* data;
                int* dataSize;
                if (0<=simHandleVisionSensor(_visionSensorHandles[i],&data,&dataSize))
                {
                    float farClippingPlane;
                    simGetObjectFloatParameter(_visionSensorHandles[i],1001,&farClippingPlane);
                    float RR=(farClippingPlane*0.99f)*(farClippingPlane*0.99f);
                    float m[12];
                    simGetObjectMatrix(_visionSensorHandles[i],-1,m);
                    if (dataSize[0]>1)
                    {
                        int off=dataSize[1];
                        if (dataSize[2]>1)
                        {
                            int ptsX=int(data[off+0]+0.5f);
                            int ptsY=int(data[off+1]+0.5f);
                            off+=2;
                            unsigned char col[3];
                            for (int j=0;j<ptsX*ptsY;j++)
                            {
                                float p[3]={data[off+4*j+0],data[off+4*j+1],data[off+4*j+2]};
                                float rr=p[0]*p[0]+p[1]*p[1]+p[2]*p[2];
                                if (rr<RR)
                                {
                                    float dp[3]={p[0],p[1],p[2]};

                                    // if true, point clouds are published locally. otherwise globally
                                    if (_B_LOCALLY){
                                        m[3] = 0;
                                        m[7] = 0;
                                        m[11] = 0;
                                        mainSensTrInv[3] = 0;
                                        mainSensTrInv[7] = 0;
                                        mainSensTrInv[11] = 0;

                                    }

                                    simTransformVector(m,p); //directly relative to /odom (which in simulation is actually /map)


                                    if(_B_LOCALLY){
                                        local_frame_mat_inverse[3] = 0;
                                        local_frame_mat_inverse[7] = 0;
                                        local_frame_mat_inverse[11] = 0;
                                        simTransformVector(local_frame_mat_inverse, p);
                                    }

                                    float abs_p[3]={p[0],p[1],p[2]};

                                    if(!_B_LOCALLY){
                                        simTransformVector(mainSensTrInv,p);
                                    }

                                    float a=atan2(p[0],p[2]);

                                    /* if (   ((a>=startAnglePlusMinusPi)&&(a<startAnglePlusMinusPi+scanRange)) || ((a<=startAnglePlusMinusPi)&&(a<startAnglePlusMinusPi+scanRange-2.0f*PI_VAL))   ) */
                                    if(true)
                                    {
                                        float r=sqrt(rr);
                                        if (_cartesianCoords)
                                        {
                                            pts.push_back(abs_p[0]);
                                            pts.push_back(abs_p[1]);
                                            pts.push_back(abs_p[2]);
                                        }
                                        else
                                        {
                                            pts.push_back(a);
                                            pts.push_back(0.5f*PI_VAL-atan2(p[1],sqrt(p[0]*p[0]+p[2]*p[2])));
                                            pts.push_back(r);
                                        }
                                        if (_displayPts)
                                        {
                                            dp[0]*=_displayScalingFactor;
                                            dp[1]*=_displayScalingFactor;
                                            dp[2]*=_displayScalingFactor;
                                            simTransformVector(m,dp);
                                            _displayPtsA.push_back(a);
                                            if(_B_LOCALLY){
                                                // compenstate xyz coordinates when you are using Local Frame
                                                _displayPtsXyz.push_back(dp[0] + local_frame_mat[3]);
                                                _displayPtsXyz.push_back(dp[1] + local_frame_mat[7]);
                                                _displayPtsXyz.push_back(dp[2] + local_frame_mat[11]);
                                            }
                                            else{
                                                _displayPtsXyz.push_back(dp[0]);
                                                _displayPtsXyz.push_back(dp[1]);
                                                _displayPtsXyz.push_back(dp[2]);
                                            }
                                            _getColorFromIntensity(1.0f-((r-_coloringDistances[0])/(_coloringDistances[1]-_coloringDistances[0])),col);
                                            _displayPtsCol.push_back(col[0]);
                                            _displayPtsCol.push_back(col[1]);
                                            _displayPtsCol.push_back(col[2]);
                                        }
                                    }
                                }
                            }
                        }
                        else
                            retVal=false;
                    }
                    else
                        retVal=false;
                    simReleaseBuffer((char*)data);
                    simReleaseBuffer((char*)dataSize);
                }
                else
                    retVal=false;
            }
        }//Finished detecting points

        if (_displayPts&&(_displayPtsXyz.size()>0))
        {
            char zeroCol[12]={0,0,0,0,0,0,0,0,0,0,0,0};
            int options=2;
            if (_emissivePts)
                options|=4;
            if (_displayOnlyCurrent)
            {
                if (int(_displayPtsXyz.size())>existingDisplayPointsSize)
                {
                    // Using the new or old pt cloud functionality?
                    if (_newPtCloudHandle>=0)
                        simInsertPointsIntoPointCloud(_newPtCloudHandle,2,&_displayPtsXyz[existingDisplayPointsSize],((int)_displayPtsXyz.size()/3)-existingDisplayPointsSize/3,&_displayPtsCol[existingDisplayPointsSize],0);
                    else
                        _ptCloudHandle=simAddPointCloud(0,255,-1,options,_pointSize,((int)_displayPtsXyz.size()/3)-existingDisplayPointsSize/3,&_displayPtsXyz[existingDisplayPointsSize],zeroCol,(char*)&_displayPtsCol[existingDisplayPointsSize],0);
                }
            }
            else

            {
                    // Using the new or old pt cloud functionality?
                    if (_newPtCloudHandle>=0)
                        simInsertPointsIntoPointCloud(_newPtCloudHandle,2,&_displayPtsXyz[0],(int)_displayPtsXyz.size()/3,&_displayPtsCol[0],0);
                    else
                        _ptCloudHandle=simAddPointCloud(0,255,-1,options,_pointSize,(int)_displayPtsXyz.size()/3,&_displayPtsXyz[0],zeroCol,(char*)&_displayPtsCol[0],0);
            }
/*
            if (_displayOnlyCurrent)
            {
                if (int(_displayPtsXyz.size())>existingDisplayPointsSize)
                    _ptCloudHandle=simAddPointCloud(0,255,-1,options,_pointSize,((int)_displayPtsXyz.size()/3)-existingDisplayPointsSize/3,&_displayPtsXyz[existingDisplayPointsSize],zeroCol,(char*)&_displayPtsCol[existingDisplayPointsSize],0);
            }
            else
                _ptCloudHandle=simAddPointCloud(0,255,-1,options,_pointSize,(int)_displayPtsXyz.size()/3,&_displayPtsXyz[0],zeroCol,(char*)&_displayPtsCol[0],0);
                */
        }

        lastScanAngle=fmod(lastScanAngle+scanRange,2.0f*PI_VAL);

        addPointsToBuffer(pts,_buffer);

        if(_RANGE>=2*PI_VAL)//one revolution
        {
            //retVal=true; //(by default)

            _RANGE=0;
            _pubVelodyne.publish(_buffer);
            _buffer.data.clear();
            _buffer.width=0;


        }
        else
            retVal=false;
    }
    else
        retVal=false;
    return(retVal);
}

void CVelodyneROSModel::_removePointsBetween(float lowAngle,float range)
{
    std::vector<float> displayPtsXyz(_displayPtsXyz);
    std::vector<float> displayPtsA(_displayPtsA);
    std::vector<unsigned char> displayPtsCol(_displayPtsCol);
    _displayPtsXyz.clear();
    _displayPtsA.clear();
    _displayPtsCol.clear();
    for (int i=0;i<int(displayPtsA.size());i++)
    {
        if ( ((displayPtsA[i]<lowAngle)||(displayPtsA[i]>=lowAngle+range)) && ((displayPtsA[i]>lowAngle)||(displayPtsA[i]>=lowAngle+range-2.0f*PI_VAL)) )
        {
            _displayPtsA.push_back(displayPtsA[i]);
            _displayPtsXyz.push_back(displayPtsXyz[3*i+0]);
            _displayPtsXyz.push_back(displayPtsXyz[3*i+1]);
            _displayPtsXyz.push_back(displayPtsXyz[3*i+2]);
            _displayPtsCol.push_back(displayPtsCol[3*i+0]);
            _displayPtsCol.push_back(displayPtsCol[3*i+1]);
            _displayPtsCol.push_back(displayPtsCol[3*i+2]);
        }
    }
}

void CVelodyneROSModel::_getColorFromIntensity(float intensity,unsigned char col[3])
{
    if (intensity>1.0f)
        intensity=1.0f;
    if (intensity<0.0f)
        intensity=0.0f;
    const float c[12]={0.0f,0.0f,1.0f,1.0f,0.0f,1.0f,1.0f,0.0f,0.0f,1.0f,1.0f,0.0f};
    int d=int(intensity*3);
    if (d>2)
        d=2;
    float r=(intensity-float(d)/3.0f)*3.0f;
    col[0]=(unsigned char)(255.0f*(c[3*d+0]*(1.0f-r)+c[3*(d+1)+0]*r));
    col[1]=(unsigned char)(255.0f*(c[3*d+1]*(1.0f-r)+c[3*(d+1)+1]*r));
    col[2]=(unsigned char)(255.0f*(c[3*d+2]*(1.0f-r)+c[3*(d+1)+2]*r));
}

void CVelodyneROSModel::addPointsToBuffer(std::vector<float> & pts, sensor_msgs::PointCloud2 & buff)
{
    int n_points = pts.size()/3;
    int prev_width= buff.width*buff.point_step;
    buff.width += n_points;
    buff.row_step = buff.point_step*buff.width;
    buff.data.resize(buff.row_step*buff.height);
    buff.header.stamp=ros::Time::now();

    //copy data points
    for (int cp = 0; cp < n_points; ++cp)
    {
        memcpy(&buff.data[prev_width + cp * buff.point_step + buff.fields[0].offset], &pts[3*cp+0], sizeof(float));
        memcpy(&buff.data[prev_width + cp * buff.point_step + buff.fields[1].offset], &pts[3*cp+1], sizeof(float));
        memcpy(&buff.data[prev_width + cp * buff.point_step + buff.fields[2].offset], &pts[3*cp+2], sizeof(float));
    }
}
