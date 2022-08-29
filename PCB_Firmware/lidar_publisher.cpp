/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Eric Perko, Chad Rockey
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Case Western Reserve University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/asio.hpp>
#include <open_tof_lidar.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_publisher");  //iniiate a new node named lidr publisher
  ros::NodeHandle n;  //opens a new node named n
  ros::NodeHandle priv_nh("~"); //spcifies namespace

  std::string port;         //initialize various paramters
  int baud_rate;
  std::string frame_id;
  int start_angle;
  int stop_angle;
  double corr_angle;

  priv_nh.param("port", port, std::string("/dev/ttyUSB0"));  //feed the parameters into node handle
  priv_nh.param("baud_rate", baud_rate, 500000);
  priv_nh.param("frame_id", frame_id, std::string("/lidar"));
  priv_nh.param("start_angle", start_angle, 0);
  priv_nh.param("stop_angle", stop_angle, 360);
  priv_nh.param("corr_angle", corr_angle, 0.0);

  boost::asio::io_service io;           //something with I/O context

  try {
    lidar_driver::openTOFLidarDriver laser(port, baud_rate, io); //feed the laser parameters
    laser.SetStartStopAngles(start_angle, stop_angle); //set angles
    laser.SetCorrRotationAngle(corr_angle);            //correct rotation
    ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1000);  //creates publisher object on topic "scan"
    ROS_INFO("OpenTOFLidar driver laser started"); //just notifies that we've begun

    while (ros::ok()) {                                             //as long as the ros node is running, will return false in case node stops sending signal
      sensor_msgs::LaserScan::Ptr scan(new sensor_msgs::LaserScan); //create ew object of class scan named scan
      scan->header.frame_id = frame_id; //set frame
      scan->header.stamp = ros::Time::now();    //set timestamp
      laser.poll(scan);
      laser_pub.publish(scan);  //sends the message in "scan"
    }
    laser.close();
    return 0;
  } catch (boost::system::system_error ex) {        //catches for system error
    ROS_ERROR("Error during lidar driver work. Are you sure you have the correct port and baud rate? Error was %s", ex.what());
    return -1;
  }
}
