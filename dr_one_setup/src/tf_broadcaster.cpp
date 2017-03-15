#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"tf_publisher");
    ros::NodeHandle n("~");
    ros::Rate r(100);
    tf::TransformBroadcaster broadcaster;
    double _laser_tr_x, _laser_tr_y, _laser_tr_z, _laser_rot_x, _laser_rot_y, _laser_rot_z,
           _cam_tr_x, _cam_tr_y, _cam_tr_z, _cam_rot_x, _cam_rot_y, _cam_rot_z;

    //Reading from the parameter server
    //the parameters are set in tf_parameters.yaml file!
    n.getParam("laser_tran_x",_laser_tr_x);
    n.getParam("laser_tran_y",_laser_tr_y);
    n.getParam("laser_tran_z",_laser_tr_z);
    n.getParam("laser_rot_x",_laser_rot_x);
    n.getParam("laser_rot_y",_laser_rot_y);
    n.getParam("laser_rot_z",_laser_rot_z);

    n.getParam("camera_tran_x",_cam_tr_x);
    n.getParam("camera_tran_y",_cam_tr_y);
    n.getParam("camera_tran_z",_cam_tr_z);
    n.getParam("camera_rot_x",_cam_rot_x);
    n.getParam("camera_rot_y",_cam_rot_y);
    n.getParam("camera_rot_z",_cam_rot_z);


    //Conversion from deg to rad
    _laser_rot_x *= M_PI/180.0;
    _laser_rot_y *= M_PI/180.0;
    _laser_rot_z *= M_PI/180.0;
    _cam_rot_x *= M_PI/180.0;
    _cam_rot_y *= M_PI/180.0;
    _cam_rot_z *= M_PI/180.0;

    //Conversion from mm to m
    _laser_tr_x /= 1000.0;
    _laser_tr_y /= 1000.0;
    _laser_tr_z /= 1000.0;
    _cam_tr_x /= 1000.0;
    _cam_tr_y /= 1000.0;
    _cam_tr_z /= 1000.0;


    while (n.ok())
    {
        //Transform for laser
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(
                            tf::createQuaternionFromRPY(_laser_rot_x,_laser_rot_y,_laser_rot_z),
                            tf::Vector3(_laser_tr_x,_laser_tr_y,_laser_tr_z)),
                            ros::Time::now(),"base_link","laser" ));

//          //Transform for camera
//        broadcaster.sendTransform(
//            tf::StampedTransform(
//                tf::Transform(
//                            tf::createQuaternionFromRPY(_cam_rot_x,_cam_rot_y,_cam_rot_z) ,
//                            tf::Vector3(_cam_tr_x,_cam_tr_y,_cam_tr_z)),
//                            ros::Time::now(),"base_link","camera_link" ));

        r.sleep();
    }
}
