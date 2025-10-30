#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <iomanip>

namespace oxts_ntrip {

  class GgaGenerator : public rclcpp::Node
  {
    public:
      GgaGenerator(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("gga_generator", options)
      {
        sub_fix = create_subscription<sensor_msgs::msg::NavSatFix>("fix", 1, std::bind(&GgaGenerator::recv_fix, this, std::placeholders::_1));
        pub_gga = create_publisher<std_msgs::msg::String>("nmea", 1);
      }

    private:
      rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_fix;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_gga;

      void recv_fix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
        std_msgs::msg::String gga_msg;
        generate_gga_string(msg->latitude, msg->longitude, msg->altitude, msg->header.stamp, gga_msg.data);
        pub_gga->publish(gga_msg);
      }

      static inline void generate_gga_string(double latitude, double longitude, double altitude, const rclcpp::Time& utc_time, std::string& gga_msg)
      {
        std::stringstream msg;
        std::stringstream gga_payload;

        int lat_deg = (int)std::abs(latitude);
        float lat_min = 60.0 * fmod(std::abs(latitude), 1.0);
        int lon_deg = (int)std::abs(longitude);
        float lon_min = 60.0 * fmod(std::abs(longitude), 1.0);

        double utc_timestamp = utc_time.seconds();
        uint8_t utc_hours = (uint8_t)(fmod(utc_timestamp / 3600.0, 24.0));
        uint8_t utc_minutes = (uint8_t)(60.0 * fmod(utc_timestamp / 3600.0, 1.0));
        uint8_t utc_secs = (uint8_t)(3600.0 * fmod(utc_timestamp / 3600.0, 24.0) - 3600.0 * utc_hours - 60.0 * utc_minutes);

        gga_payload.precision(3);
        gga_payload << std::setfill('0');
        gga_payload << "GPGGA," << std::setw(2) << (int)utc_hours << std::setw(2) << (int)utc_minutes << std::setw(2) << (int)utc_secs << ",";
        gga_payload << std::setw(2) << lat_deg << std::fixed << std::setw(6) << lat_min;

        if (latitude > 0) {
          gga_payload << ",N,";
        } else {
          gga_payload << ",S,";
        }

        gga_payload << std::setw(3) << lon_deg << std::fixed << std::setw(6) << lon_min;
        if (longitude > 0) {
          gga_payload << ",E,";
        } else {
          gga_payload << ",W,";
        }

        gga_payload.precision(1);
        gga_payload << "1,4,1.5,";
        gga_payload << std::setfill('0');
        gga_payload << std::fixed << std::setw(4) << altitude;
        gga_payload << ",M,0,M,,";

        uint8_t checksum = 0;
        for (size_t i = 0; i < gga_payload.str().size(); i++) {
          checksum ^= gga_payload.str()[i];
        }

        msg << "$" << gga_payload.str() << "*" << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << (int)checksum << "\r\n";
        gga_msg = msg.str();
      }

  };

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(oxts_ntrip::GgaGenerator)
