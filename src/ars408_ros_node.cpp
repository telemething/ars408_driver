// Copyright 2021 Perception Engine, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ars408_ros/ars408_ros_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <unordered_map>

PeContinentalArs408Node::PeContinentalArs408Node(const rclcpp::NodeOptions & node_options)
: Node("ars408_node", node_options)
{
  GenerateUUIDTable();
  Run();
}

//for now we only allow this fixed config. In the future maybe we can make this configurable.
void PeContinentalArs408Node::RunConfig(const can_msgs::msg::Frame::SharedPtr can_msg)
{
  // try to fetch the current radar state
  if(ars408_driver_.GetCurrentRadarState(ars408_state_))
  {
    // if the radar is not set to output objects, set it to output objects
    if( ars408_state_.OutputType !=  ars408::RadarState::OBJECTS )
    {
      RCLCPP_INFO(this->get_logger(), "Setting radar to output objects");

      ars408::RadarCfg radar_new_config;
      radar_new_config.OutputType = ars408::RadarCfg::OBJECTS;
      //radar_new_config.OutputType = ars408::RadarCfg::NONE;
      radar_new_config.UpdateOutputType = true;

      radar_new_config.StoreInNVM = true;
      radar_new_config.UpdateStoreInNVM = true;

      radar_new_config.SendQuality = true;
      radar_new_config.UpdateSendQuality = true;

      radar_new_config.SendExtInfo = true;
      radar_new_config.UpdateSendExtInfo = true;

      radar_new_config.RadarPower = ars408::RadarCfg::RadarPowerConfig::MINUS_9dB_GAIN;
      radar_new_config.UpdateRadarPower = true;

      radar_new_config.RCS_Status = ars408::RadarCfg::RCS_Threshold::HIGH_SENSITIVITY;
      radar_new_config.UpdateRCS_Threshold = true;

      radar_new_config.SortIndex = ars408::RadarCfg::Sorting::BY_RANGE;
      radar_new_config.UpdateSortIndex = true;

      radar_new_config.MaxDistance = 256;
      radar_new_config.UpdateMaxDistance = true;

      auto can_command = ars408_driver_.GenerateRadarConfiguration(radar_new_config);

      can_msgs::msg::Frame config_can_frame;
      config_can_frame.header = can_msg->header;
      config_can_frame.data = can_command;
      config_can_frame.id = ars408::RADAR_CFG;
      config_can_frame.dlc = ars408::RADAR_CFG_BYTES;

      //cansend can0 200#F8000000089C0000 // Objects detection with all extended properties
      //cansend can0 200#F8000000109C0000 // Clusters detection with all extended properties

      SendCanMessage(std::make_shared<can_msgs::msg::Frame>(config_can_frame));
    }

    // don't configure again
    run_config_ = false;
  }  
}

void PeContinentalArs408Node::CanFrameCallback(const can_msgs::msg::Frame::SharedPtr can_msg)
{
  if (!can_msg->data.empty()) {
    can_data_ = can_msg;
    ars408_driver_.Parse(can_msg->id, can_msg->data, can_msg->dlc);

    // if we need to configure the radar, do it now
    if(run_config_)
      RunConfig(can_msg);
  }
}

uint32_t PeContinentalArs408Node::ConvertRadarClassToAwSemanticClass(
  const ars408::Obj_3_Extended::ObjectClassProperty & in_radar_class)
{
  switch (in_radar_class) {
    case ars408::Obj_3_Extended::BICYCLE:
      return 32006;
      break;
    case ars408::Obj_3_Extended::CAR:
      return 32001;
      break;
    case ars408::Obj_3_Extended::TRUCK:
      return 32002;
      break;
    case ars408::Obj_3_Extended::MOTORCYCLE:
      return 32005;
      break;
    case ars408::Obj_3_Extended::POINT:
    case ars408::Obj_3_Extended::RESERVED_01:
    case ars408::Obj_3_Extended::WIDE:
    case ars408::Obj_3_Extended::RESERVED_02:
    default:
      return 32000;
      break;
  }
}

radar_msgs::msg::RadarTrack PeContinentalArs408Node::ConvertRadarObjectToRadarTrack(
  const ars408::RadarObject & in_object)
{
  radar_msgs::msg::RadarTrack out_object;
  out_object.uuid = UUID_table_[in_object.id];

  out_object.position.x = in_object.distance_long_x;
  out_object.position.y = in_object.distance_lat_y;

  out_object.velocity.x = in_object.speed_long_x;
  out_object.velocity.y = in_object.speed_lat_y;
  out_object.velocity_covariance.at(0) = 0.1;

  out_object.acceleration.x = in_object.rel_acceleration_long_x;
  out_object.acceleration.y = in_object.rel_acceleration_lat_y;

  out_object.size.x = size_x_;
  out_object.size.y = size_y_;
  out_object.size.z = 1.0;

  out_object.classification = ConvertRadarClassToAwSemanticClass(in_object.object_class);

  return out_object;
}

radar_msgs::msg::RadarReturn PeContinentalArs408Node::ConvertRadarObjectToRadarReturn(
  const ars408::RadarObject & in_object)
{
  radar_msgs::msg::RadarReturn radar_return;
  radar_return.range = std::sqrt(
    in_object.distance_long_x * in_object.distance_long_x +
    in_object.distance_lat_y * in_object.distance_lat_y);
  radar_return.azimuth = std::atan2(in_object.distance_lat_y, in_object.distance_long_x);
  radar_return.doppler_velocity = in_object.speed_long_x / std::cos(radar_return.azimuth);
  radar_return.elevation = 0.0;
  radar_return.amplitude = 0.0;
  return radar_return;
}

// send a message to the radar over the CAN bus
void PeContinentalArs408Node::SendCanMessage(const can_msgs::msg::Frame::SharedPtr can_msg)
{
  can_messages_out_->publish(*can_msg);
}

void PeContinentalArs408Node::RadarDetectedObjectsCallback(
  const std::unordered_map<uint8_t, ars408::RadarObject> & detected_objects)
{
  radar_msgs::msg::RadarTracks output_objects;
  output_objects.header.frame_id = output_frame_;
  output_objects.header.stamp = can_data_->header.stamp;

  radar_msgs::msg::RadarScan output_scan;
  output_scan.header.frame_id = output_frame_;
  output_scan.header.stamp = can_data_->header.stamp;

  //***************************************************************************************

  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.clear();

  //delete old marker
  visualization_msgs::msg::Marker ma;
  ma.action=3;
  marker_array.markers.push_back(ma);
  marker_array_publisher_->publish(marker_array);
  marker_array.markers.clear();

  //marker for ego car
  visualization_msgs::msg::Marker mEgoCar;

  mEgoCar.header.stamp = can_data_->header.stamp;
  mEgoCar.header.frame_id = output_frame_;
  mEgoCar.ns = "";
  mEgoCar.id = 999;

  //if you want to use a cube comment out the next 2 lines
  //mEgoCar.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  //mEgoCar.mesh_resource = "file://"+ament_index_cpp::get_package_share_directory("radar_conti_ars408")+"/resources/low_poly_911.dae";
  // mEgoCar.type = 1; // cube
  mEgoCar.action = 0; // add/modify
  mEgoCar.pose.position.x = 0.0;
  mEgoCar.pose.position.y = 0.0;
  mEgoCar.pose.position.z = 0.0;

  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, M_PI/2);
  //myQuaternion.setRPY(0, 0, 0);

  mEgoCar.pose.orientation.w = myQuaternion.getW();
  mEgoCar.pose.orientation.x = myQuaternion.getX();
  mEgoCar.pose.orientation.y = myQuaternion.getY();
  mEgoCar.pose.orientation.z = myQuaternion.getZ();
  mEgoCar.scale.x = 1.0;
  mEgoCar.scale.y = 1.0;
  mEgoCar.scale.z = 1.0;
  mEgoCar.color.r = 0.0;
  mEgoCar.color.g = 0.0;
  mEgoCar.color.b = 1.0;
  mEgoCar.color.a = 1.0;
  mEgoCar.lifetime = rclcpp::Duration(0.2,0);
  mEgoCar.frame_locked = false;

  marker_array.markers.push_back(mEgoCar);

  int32_t marker_index = 100;

  //****************************************************************************************

  for (const auto & object : detected_objects) 
  {
    if (publish_radar_track_) 
    {
      output_objects.tracks.emplace_back(ConvertRadarObjectToRadarTrack(object.second));
    };
    if (publish_radar_scan_) 
    {
      output_scan.returns.emplace_back(ConvertRadarObjectToRadarReturn(object.second));
    }

    //****************************************************************************************

    if(true)
    {
      visualization_msgs::msg::Marker mobject;
      /*visualization_msgs::msg::Marker mtext;

      mtext.header.stamp = can_data_->header.stamp;
      mtext.header.frame_id = output_frame_;
      mtext.ns = "";
      mtext.id = (object.first+100);
      mtext.type = 1; //Cube
      mtext.action = 0; // add/modify
      mtext.pose.position.x = object.second.distance_long_x;
      mtext.pose.position.y = object.second.distance_lat_y;
      mtext.pose.position.z = 4.0;

                  
      //myQuaternion.setRPY(M_PI / 2, 0, 0);
      myQuaternion.setRPY(0, 0, 0);

      mtext.pose.orientation.w = myQuaternion.getW();
      mtext.pose.orientation.x = myQuaternion.getX();
      mtext.pose.orientation.y = myQuaternion.getY();
      mtext.pose.orientation.z = myQuaternion.getZ();
      mtext.scale.x = 1.0;
      mtext.scale.y = 1.0;
      mtext.scale.z = 2.0;
      mtext.color.r = 1.0;
      mtext.color.g = 1.0;
      mtext.color.b = 1.0;
      mtext.color.a = 1.0;
      mtext.lifetime = rclcpp::Duration(0.2,0);
      mtext.frame_locked = false;
      mtext.type=9;
      mtext.text= "object_" + std::to_string(object.first) + ": \n" 
        + " RCS: " + std::to_string(object.second.rcs) + "dBm^2" + " \n" 
        + " V_long: " +   std::to_string(object.second.speed_long_x) + "m/s" + " \n" 
        + " V_lat: " + std::to_string(object.second.speed_lat_y) + "m/s" + " \n" 
        + " Orientation: " + std::to_string(object.second.orientation_angle) + "degree";

      marker_array.markers.push_back(mtext);*/

      mobject.header.stamp = can_data_->header.stamp;
      mobject.header.frame_id = output_frame_;
      mobject.ns = "";
      mobject.id = object.first;
      mobject.type = 1; //Cube
      mobject.action = 0; // add/modify
      mobject.pose.position.x = object.second.distance_long_x;
      mobject.pose.position.y = object.second.distance_lat_y;
      mobject.pose.position.z = 1.0;

      myQuaternion.setRPY(0, 0, 0);

      mobject.pose.orientation.w = myQuaternion.getW();
      mobject.pose.orientation.x = myQuaternion.getX();
      mobject.pose.orientation.y = myQuaternion.getY();
      mobject.pose.orientation.z = myQuaternion.getZ();
      mobject.scale.x = object.second.length;
      mobject.scale.y = object.second.width;
      mobject.scale.z = 1.0;
      mobject.color.r = 0.0;
      mobject.color.g = 1.0;
      mobject.color.b = 0.0;
      mobject.color.a = 1.0;
      mobject.lifetime = rclcpp::Duration(0.2,0);
      mobject.frame_locked = false;

      marker_array.markers.push_back(mobject);
    }

  //****************************************************************************************
  }

  if (publish_radar_track_) 
  {
    publisher_radar_tracks_->publish(output_objects);
  }
  if (publish_radar_scan_) 
  {
    publisher_radar_scan_->publish(output_scan);
  }  
  if (true) 
  {
    marker_array_publisher_->publish(marker_array);
  }


}

unique_identifier_msgs::msg::UUID PeContinentalArs408Node::GenerateRandomUUID()
{
  unique_identifier_msgs::msg::UUID uuid;
  std::mt19937 gen(std::random_device{}());
  std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
  std::generate(uuid.uuid.begin(), uuid.uuid.end(), bit_eng);
  return uuid;
}

void PeContinentalArs408Node::GenerateUUIDTable()
{
  for (size_t i = 0; i <= max_radar_id; i++) {
    UUID_table_.emplace_back(PeContinentalArs408Node::GenerateRandomUUID());
  }
}

void PeContinentalArs408Node::Run()
{
  output_frame_ = this->declare_parameter<std::string>("output_frame", "ars408");
  publish_radar_track_ = this->declare_parameter<bool>("publish_radar_track", true);
  publish_radar_scan_ = this->declare_parameter<bool>("publish_radar_scan", false);
  sequential_publish_ = this->declare_parameter<bool>("sequential_publish", false);
  size_x_ = this->declare_parameter<double>("size_x", 1.8);
  size_y_ = this->declare_parameter<double>("size_y", 1.8);

  ars408_driver_.RegisterDetectedObjectsCallback(
    std::bind(&PeContinentalArs408Node::RadarDetectedObjectsCallback, this, std::placeholders::_1),
    sequential_publish_);

  //subscription_ = this->create_subscription<can_msgs::msg::Frame>(
  //  "~/input/frame", 10,
  //  std::bind(&PeContinentalArs408Node::CanFrameCallback, this, std::placeholders::_1));

  // TODO: make configurable
  subscription_ = this->create_subscription<can_msgs::msg::Frame>(
    "/from_can_bus", 10,
    std::bind(&PeContinentalArs408Node::CanFrameCallback, this, std::placeholders::_1));

  // TODO: make configurable
  can_messages_out_ = this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 10);

  publisher_radar_tracks_ =
    this->create_publisher<radar_msgs::msg::RadarTracks>("~/output/objects", 10);
  publisher_radar_scan_ = this->create_publisher<radar_msgs::msg::RadarScan>("~/output/scan", 10);
  marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/output/markers", 10);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PeContinentalArs408Node)
