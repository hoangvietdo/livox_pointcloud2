// MIT License
//
// Copyright (c) 2024 Hoang Viet Do
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Eigen>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/point_field_conversion.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>

struct LoamLivoxPointCloudType {
  float_t x;
  float_t y;
  float_t z;
  float_t intensity;
  uint tag;
  uint line;
};

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(LoamLivoxPointCloudType,
    (float_t, x, x)
    (float_t, y, y)
    (float_t, z, z)
    (float_t, intensity, intensity)
    (uint, tag, tag)
    (uint, line, line))
// clang-format on

int main() {
  sensor_msgs::PointCloud2 output;
  rosbag::Bag bag;
  bag.open("/home/vietdo/Downloads/bag_in.bag", rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(std::string("/livox/lidar"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  rosbag::Bag bag_write;
  bag_write.open("/home/vietdo/Downloads/bag_out.bag", rosbag::bagmode::Write);

  for (const rosbag::MessageInstance &m : view) {
    auto time_stamp = m.getTime();
    const auto topic = m.getTopic();
    if (topic == "/livox/lidar") {
      const auto lidar_scan = m.instantiate<sensor_msgs::PointCloud2>();
      if (lidar_scan != NULL) {
        auto livox_PCL_msg(new pcl::PointCloud<LoamLivoxPointCloudType>);
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*lidar_scan, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, *livox_PCL_msg);
        output.header = lidar_scan->header;
        output.fields.resize(6);

        // x field
        output.fields[0].name = "x";
        output.fields[0].offset = 0;
        output.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        output.fields[0].count = 1;

        // y field
        output.fields[1].name = "y";
        output.fields[1].offset = 4;
        output.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        output.fields[1].count = 1;

        // z field
        output.fields[2].name = "z";
        output.fields[2].offset = 8;
        output.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        output.fields[2].count = 1;

        // intensity field
        output.fields[3].name = "intensity";
        output.fields[3].offset = 12;
        output.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
        output.fields[3].count = 1;

        // tag field
        output.fields[4].name = "tag";
        output.fields[4].offset = 16;
        output.fields[4].datatype = sensor_msgs::PointField::UINT8;
        output.fields[4].count = 1;

        // line field
        output.fields[5].name = "line";
        output.fields[5].offset = 17;
        output.fields[5].datatype = sensor_msgs::PointField::UINT8;
        output.fields[5].count = 1;

        output.point_step = 18; // Size of a point in bytes
        output.row_step = lidar_scan->row_step;
        output.data.resize(output.row_step);
        output.width = lidar_scan->width;
        output.height = 1; // Unordered point cloud
        output.is_bigendian = false;
        output.is_dense = true;

        uint8_t *raw_data_ptr = output.data.data();

        for (uint i = 0; i < livox_PCL_msg->size(); ++i) {
          auto target = livox_PCL_msg->at(i);
          if ((target.x > (2.727) && target.x < (16.002)) &&
              (target.y > (1.442) && target.y < (10.206)) &&
              (target.z > (-4.521) && target.z < (2.132))) {
            *(reinterpret_cast<float *>(raw_data_ptr + 0)) = target.x;
            *(reinterpret_cast<float *>(raw_data_ptr + 4)) = target.y;
            *(reinterpret_cast<float *>(raw_data_ptr + 8)) = target.z;
            *(reinterpret_cast<float *>(raw_data_ptr + 12)) =
                static_cast<float>(target.intensity);
            *(raw_data_ptr + 16) = target.tag;
            *(raw_data_ptr + 17) = target.line;

            raw_data_ptr += output.point_step;
          } else {
            *(reinterpret_cast<float *>(raw_data_ptr + 0)) = 0.0;
            *(reinterpret_cast<float *>(raw_data_ptr + 4)) = 0.0;
            *(reinterpret_cast<float *>(raw_data_ptr + 8)) = 0.0;
            *(reinterpret_cast<float *>(raw_data_ptr + 12)) = 0.0;
            *(raw_data_ptr + 16) = 0;
            *(raw_data_ptr + 17) = 0;

            raw_data_ptr += output.point_step;
          }
        }
        bag_write.write("/livox/lidar", time_stamp, output);
      }
    }
  }
  bag.close();
  bag_write.close();
  return 0;
}
