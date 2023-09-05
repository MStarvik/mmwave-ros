#include "mmwave/mmwave.h"
#include "mmwave/mmwave_cfg.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <fcntl.h>
#include <unistd.h>

static std::vector<sensor_msgs::msg::PointField> default_point_fields() {
  std::vector<sensor_msgs::msg::PointField> fields;
  fields.resize(6);
  fields[0].name = "x";
  fields[0].offset = 0;
  fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  fields[0].count = 1;
  fields[1].name = "y";
  fields[1].offset = 4;
  fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  fields[1].count = 1;
  fields[2].name = "z";
  fields[2].offset = 8;
  fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  fields[2].count = 1;
  fields[3].name = "velocity";
  fields[3].offset = 12;
  fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  fields[3].count = 1;
  fields[4].name = "snr";
  fields[4].offset = 16;
  fields[4].datatype = sensor_msgs::msg::PointField::FLOAT32;
  fields[4].count = 1;
  fields[5].name = "noise";
  fields[5].offset = 20;
  fields[5].datatype = sensor_msgs::msg::PointField::FLOAT32;
  fields[5].count = 1;
  return fields;
}

static sensor_msgs::msg::PointCloud2 empty_point_cloud(const std::string &frame_id, const int num_points) {
  static const auto fields = default_point_fields();

  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = frame_id;
  msg.height = 1;
  msg.width = num_points;
  msg.fields = fields;
  msg.is_bigendian = false;
  msg.point_step = 24;
  msg.row_step = 24 * num_points;
  msg.is_dense = true;
  
  return msg;
}

class MMWaveDevice {
 public:
  MMWaveDevice(const std::string &cfg, const std::string &data) : cfg(cfg), data(data) {}
  MMWaveDevice(const std::string &id) :
    MMWaveDevice(
      "/dev/serial/by-id/usb-Silicon_Labs_CP2105_Dual_USB_to_UART_Bridge_Controller_" + id + "-if00-port0",
      "/dev/serial/by-id/usb-Silicon_Labs_CP2105_Dual_USB_to_UART_Bridge_Controller_" + id + "-if01-port0"
    ) {}
  ~MMWaveDevice() { close(); }

  bool is_open() { return fp != nullptr; }
  void open() {
    if (is_open()) {
      close();
    }
    fp = mmwave_open(data.c_str());
  }
  void close() { 
    if (is_open()) {
      mmwave_close(fp);
      fp = nullptr;
    }
  }

  void start() { mmwave_cfg_start(cfg.c_str()); }
  void stop() { mmwave_cfg_stop(cfg.c_str()); }
  bool config(const std::string &config_file) {
    if (is_open()) {
      return mmwave_cfg_write_file(cfg.c_str(), config_file.c_str()) == 0;
    }
    return false;
  }
  
  mmwave_object *read(int *num_objects) {
    int len;
    uint8_t *data = mmwave_read(fp, &len);
    if (data == nullptr) {
      close();
      return nullptr;
    }
    return mmwave_decode(data, len, num_objects);
  }

 private:
  const std::string cfg;
  const std::string data;

  FILE *fp = nullptr;
};

class MMWaveNode : public rclcpp::Node {
 public:
  MMWaveNode(const std::string &name) : Node(name) {
    declare_parameter<std::string>("device_id", "");
    get_parameter<std::string>("device_id", device_id);

    declare_parameter<std::string>("config_file", "");
    get_parameter<std::string>("config_file", config_file);

    declare_parameter<std::string>("frame_id", "mmwave");
    get_parameter<std::string>("frame_id", frame_id);

    if (device_id.empty()) {
      throw std::runtime_error("device_id is not set");
    }

    if (config_file.empty()) {
      throw std::runtime_error("config_file is not set");
    }

    int fd = open(config_file.c_str(), O_RDONLY);
    if (fd < 0) {
      throw std::runtime_error("Failed to open config file: " + config_file + ": " + strerror(errno));
    }
    close(fd);

    device = std::make_unique<MMWaveDevice>(device_id);
    pub = create_publisher<sensor_msgs::msg::PointCloud2>("points", 10);

    thread = std::thread([this]() {
      while (rclcpp::ok()) {
        device->open();
        if (!device->is_open()) {
          RCLCPP_WARN(get_logger(), "Failed to open device: %s", strerror(errno));
          std::this_thread::sleep_for(std::chrono::seconds(1));
          continue;
        }

        if (!device->config(config_file)) {
          RCLCPP_WARN(get_logger(), "Failed to configure device");
          std::this_thread::sleep_for(std::chrono::seconds(1));
          continue;
        }

        while (rclcpp::ok()) {
          int num_objects;
          mmwave_object* objects = device->read(&num_objects);
          if (objects == nullptr) {
            RCLCPP_WARN(get_logger(), "Failed to read data: %s", strerror(errno));
            break;
          }

          auto msg = empty_point_cloud(frame_id, num_objects);
          msg.data.resize(24 * num_objects);
          for (int i = 0; i < num_objects; i++) {
            memcpy(&msg.data[24 * i], &objects[i], 24);
          }

          pub->publish(msg);
        }
      }
    });
  }
  ~MMWaveNode() { thread.join(); }

 private:
  std::string device_id;
  std::string frame_id;
  std::string config_file;

  std::unique_ptr<MMWaveDevice> device;
  std::vector<sensor_msgs::msg::PointField> fields;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;

  std::thread thread;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MMWaveNode>("mmwave_node"));
  rclcpp::shutdown();
  return 0;
}
