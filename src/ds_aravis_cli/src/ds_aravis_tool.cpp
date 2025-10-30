#include <algorithm>
#include <cstdio>
#include <fstream>
#include <string>
#include <filesystem>

#include <arv.h>
#include <nlohmann/json.hpp>

#define PIXEL_FORMATS \
    X(MONO_8                    , ((ArvPixelFormat)0x01080001u))\
    X(MONO_8_SIGNED             , ((ArvPixelFormat)0x01080002u))\
    X(MONO_10                   , ((ArvPixelFormat)0x01100003u))\
    X(MONO_10_PACKED            , ((ArvPixelFormat)0x010c0004u))\
    X(MONO_12                   , ((ArvPixelFormat)0x01100005u))\
    X(MONO_12_PACKED            , ((ArvPixelFormat)0x010c0006u))\
    X(MONO_14                   , ((ArvPixelFormat)0x01100025u))\
    X(MONO_16                   , ((ArvPixelFormat)0x01100007u))\
    X(BAYER_GR_8                , ((ArvPixelFormat)0x01080008u))\
    X(BAYER_RG_8                , ((ArvPixelFormat)0x01080009u))\
    X(BAYER_GB_8                , ((ArvPixelFormat)0x0108000au))\
    X(BAYER_BG_8                , ((ArvPixelFormat)0x0108000bu))\
    X(BAYER_GR_10               , ((ArvPixelFormat)0x0110000cu))\
    X(BAYER_RG_10               , ((ArvPixelFormat)0x0110000du))\
    X(BAYER_GB_10               , ((ArvPixelFormat)0x0110000eu))\
    X(BAYER_BG_10               , ((ArvPixelFormat)0x0110000fu))\
    X(BAYER_GR_12               , ((ArvPixelFormat)0x01100010u))\
    X(BAYER_RG_12               , ((ArvPixelFormat)0x01100011u))\
    X(BAYER_GB_12               , ((ArvPixelFormat)0x01100012u))\
    X(BAYER_BG_12               , ((ArvPixelFormat)0x01100013u))\
    X(BAYER_GR_16               , ((ArvPixelFormat)0x0110002eu))\
    X(BAYER_RG_16               , ((ArvPixelFormat)0x0110002fu))\
    X(BAYER_GB_16               , ((ArvPixelFormat)0x01100030u))\
    X(BAYER_BG_16               , ((ArvPixelFormat)0x01100031u))\
    X(BAYER_BG_10P              , ((ArvPixelFormat)0x010a0052u))\
    X(BAYER_GB_10P              , ((ArvPixelFormat)0x010a0054u))\
    X(BAYER_GR_10P              , ((ArvPixelFormat)0x010a0056u))\
    X(BAYER_RG_10P              , ((ArvPixelFormat)0x010a0058u))\
    X(BAYER_BG_12P              , ((ArvPixelFormat)0x010c0053u))\
    X(BAYER_GB_12P              , ((ArvPixelFormat)0x010c0055u))\
    X(BAYER_GR_12P              , ((ArvPixelFormat)0x010c0057u))\
    X(BAYER_RG_12P              , ((ArvPixelFormat)0x010c0059u))\
    X(BAYER_GR_12_PACKED        , ((ArvPixelFormat)0x010c002au))\
    X(BAYER_RG_12_PACKED        , ((ArvPixelFormat)0x010c002bu))\
    X(BAYER_GB_12_PACKED        , ((ArvPixelFormat)0x010c002cu))\
    X(BAYER_BG_12_PACKED        , ((ArvPixelFormat)0x010c002du))\
    X(BAYER_GR_10_PACKED        , ((ArvPixelFormat)0x010c0026u))\
    X(BAYER_RG_10_PACKED        , ((ArvPixelFormat)0x010c0027u))\
    X(BAYER_GB_10_PACKED        , ((ArvPixelFormat)0x010c0028u))\
    X(BAYER_BG_10_PACKED        , ((ArvPixelFormat)0x010c0029u))\
    X(RGB_8_PACKED              , ((ArvPixelFormat)0x02180014u))\
    X(BGR_8_PACKED              , ((ArvPixelFormat)0x02180015u))\
    X(RGBA_8_PACKED             , ((ArvPixelFormat)0x02200016u))\
    X(BGRA_8_PACKED             , ((ArvPixelFormat)0x02200017u))\
    X(RGB_10_PACKED             , ((ArvPixelFormat)0x02300018u))\
    X(BGR_10_PACKED             , ((ArvPixelFormat)0x02300019u))\
    X(RGB_12_PACKED             , ((ArvPixelFormat)0x0230001au))\
    X(BGR_12_PACKED             , ((ArvPixelFormat)0x0230001bu))\
    X(YUV_411_PACKED            , ((ArvPixelFormat)0x020c001eu))\
    X(YUV_422_PACKED            , ((ArvPixelFormat)0x0210001fu))\
    X(YUV_444_PACKED            , ((ArvPixelFormat)0x02180020u))\
    X(RGB_8_PLANAR              , ((ArvPixelFormat)0x02180021u))\
    X(RGB_10_PLANAR             , ((ArvPixelFormat)0x02300022u))\
    X(RGB_12_PLANAR             , ((ArvPixelFormat)0x02300023u))\
    X(RGB_16_PLANAR             , ((ArvPixelFormat)0x02300024u))\
    X(YUV_422_YUYV_PACKED       , ((ArvPixelFormat)0x02100032u))\
    X(CUSTOM_BAYER_GR_12_PACKED , ((ArvPixelFormat)0x810c0001u))\
    X(CUSTOM_BAYER_RG_12_PACKED , ((ArvPixelFormat)0x810c0002u))\
    X(CUSTOM_BAYER_GB_12_PACKED , ((ArvPixelFormat)0x810c0003u))\
    X(CUSTOM_BAYER_BG_12_PACKED , ((ArvPixelFormat)0x810c0004u))\
    X(CUSTOM_YUV_422_YUYV_PACKED, ((ArvPixelFormat)0x82100005u))\
    X(CUSTOM_BAYER_GR_16        , ((ArvPixelFormat)0x81100006u))\
    X(CUSTOM_BAYER_RG_16        , ((ArvPixelFormat)0x81100007u))\
    X(CUSTOM_BAYER_GB_16        , ((ArvPixelFormat)0x81100008u))\
    X(CUSTOM_BAYER_BG_16        , ((ArvPixelFormat)0x81100009u))

enum struct pixel_format : ArvPixelFormat
{
    #define X(name, value) name = value,
    PIXEL_FORMATS
    #undef X
};
// Could not use "#define X" as aabove to supply to this macro, is there a better way to do this
// short of pulling in a reflection library?
NLOHMANN_JSON_SERIALIZE_ENUM(pixel_format, {
                    { pixel_format::MONO_8                    , "MONO_8"                     },
                    { pixel_format::MONO_8_SIGNED             , "MONO_8_SIGNED"              },
                    { pixel_format::MONO_10                   , "MONO_10"                    },
                    { pixel_format::MONO_10_PACKED            , "MONO_10_PACKED"             },
                    { pixel_format::MONO_12                   , "MONO_12"                    },
                    { pixel_format::MONO_12_PACKED            , "MONO_12_PACKED"             },
                    { pixel_format::MONO_14                   , "MONO_14"                    },
                    { pixel_format::MONO_16                   , "MONO_16"                    },
                    { pixel_format::BAYER_GR_8                , "BAYER_GR_8"                 },
                    { pixel_format::BAYER_RG_8                , "BAYER_RG_8"                 },
                    { pixel_format::BAYER_GB_8                , "BAYER_GB_8"                 },
                    { pixel_format::BAYER_BG_8                , "BAYER_BG_8"                 },
                    { pixel_format::BAYER_GR_10               , "BAYER_GR_10"                },
                    { pixel_format::BAYER_RG_10               , "BAYER_RG_10"                },
                    { pixel_format::BAYER_GB_10               , "BAYER_GB_10"                },
                    { pixel_format::BAYER_BG_10               , "BAYER_BG_10"                },
                    { pixel_format::BAYER_GR_12               , "BAYER_GR_12"                },
                    { pixel_format::BAYER_RG_12               , "BAYER_RG_12"                },
                    { pixel_format::BAYER_GB_12               , "BAYER_GB_12"                },
                    { pixel_format::BAYER_BG_12               , "BAYER_BG_12"                },
                    { pixel_format::BAYER_GR_16               , "BAYER_GR_16"                },
                    { pixel_format::BAYER_RG_16               , "BAYER_RG_16"                },
                    { pixel_format::BAYER_GB_16               , "BAYER_GB_16"                },
                    { pixel_format::BAYER_BG_16               , "BAYER_BG_16"                },
                    { pixel_format::BAYER_BG_10P              , "BAYER_BG_10P"               },
                    { pixel_format::BAYER_GB_10P              , "BAYER_GB_10P"               },
                    { pixel_format::BAYER_GR_10P              , "BAYER_GR_10P"               },
                    { pixel_format::BAYER_RG_10P              , "BAYER_RG_10P"               },
                    { pixel_format::BAYER_BG_12P              , "BAYER_BG_12P"               },
                    { pixel_format::BAYER_GB_12P              , "BAYER_GB_12P"               },
                    { pixel_format::BAYER_GR_12P              , "BAYER_GR_12P"               },
                    { pixel_format::BAYER_RG_12P              , "BAYER_RG_12P"               },
                    { pixel_format::BAYER_GR_12_PACKED        , "BAYER_GR_12_PACKED"         },
                    { pixel_format::BAYER_RG_12_PACKED        , "BAYER_RG_12_PACKED"         },
                    { pixel_format::BAYER_GB_12_PACKED        , "BAYER_GB_12_PACKED"         },
                    { pixel_format::BAYER_BG_12_PACKED        , "BAYER_BG_12_PACKED"         },
                    { pixel_format::BAYER_GR_10_PACKED        , "BAYER_GR_10_PACKED"         },
                    { pixel_format::BAYER_RG_10_PACKED        , "BAYER_RG_10_PACKED"         },
                    { pixel_format::BAYER_GB_10_PACKED        , "BAYER_GB_10_PACKED"         },
                    { pixel_format::BAYER_BG_10_PACKED        , "BAYER_BG_10_PACKED"         },
                    { pixel_format::RGB_8_PACKED              , "RGB_8_PACKED"               },
                    { pixel_format::BGR_8_PACKED              , "BGR_8_PACKED"               },
                    { pixel_format::RGBA_8_PACKED             , "RGBA_8_PACKED"              },
                    { pixel_format::BGRA_8_PACKED             , "BGRA_8_PACKED"              },
                    { pixel_format::RGB_10_PACKED             , "RGB_10_PACKED"              },
                    { pixel_format::BGR_10_PACKED             , "BGR_10_PACKED"              },
                    { pixel_format::RGB_12_PACKED             , "RGB_12_PACKED"              },
                    { pixel_format::BGR_12_PACKED             , "BGR_12_PACKED"              },
                    { pixel_format::YUV_411_PACKED            , "YUV_411_PACKED"             },
                    { pixel_format::YUV_422_PACKED            , "YUV_422_PACKED"             },
                    { pixel_format::YUV_444_PACKED            , "YUV_444_PACKED"             },
                    { pixel_format::RGB_8_PLANAR              , "RGB_8_PLANAR"               },
                    { pixel_format::RGB_10_PLANAR             , "RGB_10_PLANAR"              },
                    { pixel_format::RGB_12_PLANAR             , "RGB_12_PLANAR"              },
                    { pixel_format::RGB_16_PLANAR             , "RGB_16_PLANAR"              },
                    { pixel_format::YUV_422_YUYV_PACKED       , "YUV_422_YUYV_PACKED"        },
                    { pixel_format::CUSTOM_BAYER_GR_12_PACKED , "CUSTOM_BAYER_GR_12_PACKED"  },
                    { pixel_format::CUSTOM_BAYER_RG_12_PACKED , "CUSTOM_BAYER_RG_12_PACKED"  },
                    { pixel_format::CUSTOM_BAYER_GB_12_PACKED , "CUSTOM_BAYER_GB_12_PACKED"  },
                    { pixel_format::CUSTOM_BAYER_BG_12_PACKED , "CUSTOM_BAYER_BG_12_PACKED"  },
                    { pixel_format::CUSTOM_YUV_422_YUYV_PACKED, "CUSTOM_YUV_422_YUYV_PACKED" },
                    { pixel_format::CUSTOM_BAYER_GR_16        , "CUSTOM_BAYER_GR_16"         },
                    { pixel_format::CUSTOM_BAYER_RG_16        , "CUSTOM_BAYER_RG_16"         },
                    { pixel_format::CUSTOM_BAYER_GB_16        , "CUSTOM_BAYER_GB_16"         },
                    { pixel_format::CUSTOM_BAYER_BG_16        , "CUSTOM_BAYER_BG_16"         },
})

std::string pixel_format_to_str(pixel_format format)
{
    switch(format)
    {
        #define X(name, value) case pixel_format::name: return #name;
        PIXEL_FORMATS
        #undef X
    default:
        return {};
    }
}

enum struct aravis_auto
{
    OFF = ARV_AUTO_OFF,
    ON  = ARV_AUTO_CONTINUOUS
};
NLOHMANN_JSON_SERIALIZE_ENUM(aravis_auto, {
                            { aravis_auto::OFF, "off" },
                            { aravis_auto::ON , "on" }
})

std::string aravis_auto_to_str(aravis_auto auto_mode)
{
    switch(auto_mode)
    {
    case aravis_auto::OFF:
            return "off";
            break;
    case aravis_auto::ON:
            return "on";
            break;
    default:
        return "off";
    }
}
struct aravis_config
{
        // Lucid    TRIO32S: 2048 x 1536, 1024 x 768
        // Lucid    TRIO28S: 1936 x 1464,  968 x 732
        // Blackfly 27S5C-C: 1936 x 1464,  968 x 732
        std::string  name;
        pixel_format format;
        int          x_offset;
        int          y_offset;
        int          width;
        int          height;
        int          x_binning;
        int          y_binning;
        double       frame_rate;
        double       gain;
        double       exposure;
        aravis_auto  auto_gain;
        aravis_auto  auto_exposure;
        int          mtu;
        bool         ptp_enable;
        bool         ptp_slave_only;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(aravis_config,
                                   name,
                                   format,
                                   x_offset,
                                   y_offset,
                                   width,
                                   height,
                                   x_binning,
                                   y_binning,
                                   frame_rate,
                                   gain, exposure,
                                   auto_gain,
                                   auto_exposure,
                                   mtu,
                                   ptp_enable,
                                   ptp_slave_only)

int main(int argc, char *argv[])
{
    nlohmann::json data;
    if (argc >= 2)
    {
        std::filesystem::path config_file_path   = argv[1];

        std::ifstream config_file(config_file_path);

        data = nlohmann::json::parse(config_file);

        config_file.close();
    }
    else
    {
        printf("Camera name not provided, exiting\n");
        return 1;
    }

    aravis_config desired_configuration = data.get<aravis_config>();

    ArvCamera  *camera;
    ArvDevice  *dev;
    GError     *error = NULL;

    printf("Camera Name: %s\n\n", desired_configuration.name.c_str());

    bool boson = false;
    if(desired_configuration.name.find("Boson") != std::string::npos)
    {
        boson = true;
    }

    if(desired_configuration.name != "")
    {
        camera = arv_camera_new(desired_configuration.name.c_str(), &error);
    }
    else
    {
        camera = arv_camera_new(NULL, &error);
    }

    aravis_config current_config;
    aravis_config new_config;

    if(ARV_IS_CAMERA(camera))
    {
        bool gain_available;
        bool exposure_available;
        bool auto_gain_available;
        bool auto_exposure_available;
        bool offset_available;
        bool binning_available;
        bool frame_rate_available;
        bool ptp_enable_available;
        bool ptp_slave_only_available;
        bool format_supported;

        gint64 * pixel_formats_available;
        int      x_offset_max;
        int      x_offset_min;
        int      y_offset_max;
        int      y_offset_min;
        int      width_max;
        int      width_min;
        int      height_max;
        int      height_min;
        int      x_binning_max;
        int      x_binning_min;
        int      y_binning_max;
        int      y_binning_min;
        double   frame_rate_max;
        double   frame_rate_min;
        double   gain_min;
        double   gain_max;
        double   exposure_min;
        double   exposure_max;
        int      x_offset_increment;
        int      y_offset_increment;
        int      width_increment;
        int      height_increment;
        int      x_binning_increment;
        int      y_binning_increment;
        int      sensor_width;
        int      sensor_height;

        guint number_of_formats = 0;
        dev = arv_camera_get_device(camera);

        if(!error) current_config.format = pixel_format{arv_camera_get_pixel_format(camera,
                                                                                   &error)};
        if(!error) arv_camera_get_region(camera,
                                        &current_config.x_offset,
                                        &current_config.y_offset,
                                        &current_config.width,
                                        &current_config.height,
                                        &error);
        if(!error) arv_camera_get_binning(camera,
                                         &current_config.x_binning,
                                         &current_config.y_binning,
                                         &error);

        if(!error) current_config.auto_gain = aravis_auto{
            arv_camera_get_gain_auto(camera, &error)};
        if(!error) current_config.auto_exposure = aravis_auto{
            arv_camera_get_exposure_time_auto(camera, &error)};

        if(!error) current_config.frame_rate     = arv_camera_get_frame_rate            (camera, &error);
        if(!error) current_config.gain           = arv_camera_get_gain                  (camera, &error);
        if(!error) current_config.exposure       = arv_camera_get_exposure_time         (camera, &error);
        if(!error) current_config.mtu            = arv_camera_gv_get_packet_size        (camera, &error);
        if(!error) current_config.ptp_enable     = arv_device_get_boolean_feature_value (dev, "PtpEnable", &error);
        if(!error) current_config.ptp_slave_only = arv_device_get_boolean_feature_value (dev, "PtpSlaveOnly", &error);
        if(!error) offset_available              = arv_camera_is_region_offset_available(camera, &error);
        if(!error) binning_available             = arv_camera_is_binning_available      (camera, &error);
        if(!error) frame_rate_available          = arv_camera_is_frame_rate_available   (camera, &error);
        if(!error) ptp_enable_available          = arv_device_is_feature_available      (dev, "PtpEnable", &error);
        if(!error) ptp_slave_only_available      = arv_device_is_feature_available      (dev, "PtpSlaveOnly", &error);
        if(!error) auto_gain_available           = arv_camera_is_gain_auto_available    (camera, &error);
        if(!error) auto_exposure_available       = arv_camera_is_exposure_auto_available(camera, &error);
        if(!error) gain_available                = arv_camera_is_gain_available         (camera, &error);
        if(!error) exposure_available            = arv_camera_is_exposure_time_available(camera, &error);

        auto status_buf = arv_device_get_string_feature_value(dev, "PtpStatus", &error);
        if(error != NULL)
        {
            printf("Error: %s\n", error->message);
        }
        printf("Current PTP slave status: %s\n", status_buf);

        if(boson)
        {
            auto_gain_available     = false;
            auto_exposure_available = false;
        }

        if(!error)
        {
            number_of_formats = 0;

            pixel_formats_available = arv_camera_dup_available_pixel_formats(camera,
                                                                            &number_of_formats,
                                                                            &error);

            format_supported = false;
            for(guint i = 0; i < number_of_formats; i++)
            {
                if(pixel_formats_available[i] == (ArvPixelFormat)desired_configuration.format)
                {
                    format_supported = true;
                    break;
                }
            }
        }

        if(!error) arv_camera_get_x_offset_bounds     (camera, &x_offset_min,   &x_offset_max,   &error);
        if(!error) arv_camera_get_y_offset_bounds     (camera, &y_offset_min,   &y_offset_max,   &error);
        if(!error) arv_camera_get_width_bounds        (camera, &width_min,      &width_max,      &error);
        if(!error) arv_camera_get_height_bounds       (camera, &height_min,     &height_max,     &error);
        if(!error) arv_camera_get_x_binning_bounds    (camera, &x_binning_min,  &x_binning_max,  &error);
        if(!error) arv_camera_get_y_binning_bounds    (camera, &y_binning_min,  &y_binning_max,  &error);
        if(!error) arv_camera_get_frame_rate_bounds   (camera, &frame_rate_min, &frame_rate_max, &error);
        if(!error) arv_camera_get_gain_bounds         (camera, &gain_min,       &gain_max,       &error);
        if(!error) arv_camera_get_exposure_time_bounds(camera, &exposure_min,   &exposure_max,   &error);
        if(!error) arv_camera_get_sensor_size         (camera, &sensor_width,   &sensor_height,  &error);
        if(!error) x_offset_increment  = arv_camera_get_x_offset_increment (camera, &error);
        if(!error) y_offset_increment  = arv_camera_get_y_offset_increment (camera, &error);
        if(!error) width_increment     = arv_camera_get_width_increment    (camera, &error);
        if(!error) height_increment    = arv_camera_get_height_increment   (camera, &error);
        if(!error) x_binning_increment = arv_camera_get_x_binning_increment(camera, &error);
        if(!error) y_binning_increment = arv_camera_get_y_binning_increment(camera, &error);

        if(!error)
        {
            if(desired_configuration.x_offset < x_offset_min
            || desired_configuration.x_offset > x_offset_max)
            {
                printf("X Offset %i is out of bounds, clamping between %i & %i\n",
                        desired_configuration.x_offset,
                        x_offset_min,
                        x_offset_max);

                desired_configuration.x_offset = std::clamp(desired_configuration.x_offset,
                                                            x_offset_min,
                                                            x_offset_max);
            }
            if(desired_configuration.y_offset < y_offset_min
            || desired_configuration.y_offset > y_offset_max)
            {
                printf("Y Offset %i is out of bounds, clamping between %i & %i\n\n",
                        desired_configuration.y_offset,
                        y_offset_min,
                        y_offset_max);

                desired_configuration.y_offset = std::clamp(desired_configuration.y_offset,
                                                            y_offset_min,
                                                            y_offset_max);
            }
            if(desired_configuration.width < width_min
            || desired_configuration.width > sensor_width)
            {
                printf("Width %i is out of bounds, clamping between %i & %i\n\n",
                        desired_configuration.width,
                        width_min,
                        sensor_width);

                desired_configuration.width = std::clamp(desired_configuration.width,
                                                         width_min,
                                                         sensor_width);
            }
            if(desired_configuration.height < height_min
            || desired_configuration.height > sensor_height)
            {
                printf("height %i is out of bounds, clamping between %i & %i\n\n",
                        desired_configuration.height,
                        height_min,
                        sensor_height);

                desired_configuration.height = std::clamp(desired_configuration.height,
                                                          height_min,
                                                          sensor_height);
            }
            if(desired_configuration.x_binning < x_binning_min
            || desired_configuration.x_binning > x_binning_max)
            {
                printf("X Binning %i is out of bounds, clamping between %i & %i\n\n",
                        desired_configuration.x_binning,
                        x_binning_min,
                        x_binning_max);

                desired_configuration.x_binning = std::clamp(desired_configuration.x_binning,
                                                             x_binning_min,
                                                             x_binning_max);
            }
            if(desired_configuration.y_binning < y_binning_min
            || desired_configuration.y_binning > y_binning_max)
            {
                printf("Y Binning %i is out of bounds, clamping between %i & %i\n\n",
                        desired_configuration.y_binning,
                        y_binning_min,
                        y_binning_max);

                desired_configuration.y_binning = std::clamp(desired_configuration.y_binning,
                                                             y_binning_min,
                                                             y_binning_max);
            }
            if(desired_configuration.frame_rate < frame_rate_min
            || desired_configuration.frame_rate > frame_rate_max)
            {
                printf("Frame Rate %f is out of bounds, clamping between %f & %f\n\n",
                        desired_configuration.frame_rate,
                        frame_rate_min,
                        frame_rate_max);

                desired_configuration.frame_rate = std::clamp(desired_configuration.frame_rate,
                                                              frame_rate_min,
                                                              frame_rate_max);
            }
            if(desired_configuration.gain < gain_min || desired_configuration.gain > gain_max)
            {
                printf("Gain %f is out of bounds, clamping between %f & %f\n\n",
                        desired_configuration.gain,
                        gain_min,
                        gain_max);

                desired_configuration.gain = std::clamp(desired_configuration.gain,
                                                        gain_min,
                                                        gain_max);
            }
            if(desired_configuration.exposure < exposure_min
            || desired_configuration.exposure > exposure_max)
            {
                printf("Exposure %f is out of bounds, clamping between %f & %f\n\n",
                        desired_configuration.exposure,
                        exposure_min,
                        exposure_max);

                desired_configuration.exposure = std::clamp(desired_configuration.exposure,
                                                            exposure_min,
                                                            exposure_max);
            }

            int remainder = (desired_configuration.x_offset  - x_offset_min)  % x_offset_increment;
                desired_configuration.x_offset  -= remainder;
                remainder = (desired_configuration.y_offset  - y_offset_min)  % y_offset_increment;
                desired_configuration.y_offset  -= remainder;
                remainder = (desired_configuration.width     - width_min)     % width_increment;
                desired_configuration.width     -= remainder;
                remainder = (desired_configuration.height    - height_min)    % height_increment;
                desired_configuration.height    -= remainder;
                remainder = (desired_configuration.x_binning - x_binning_min) % x_binning_increment;
                desired_configuration.x_binning -= remainder;
                remainder = (desired_configuration.y_binning - y_binning_min) % y_binning_increment;
                desired_configuration.y_binning -= remainder;
        }

        if(!error)
        {
            if(format_supported)
                arv_camera_set_pixel_format(camera,
                                           (ArvPixelFormat)desired_configuration.format,
                                           &error);
            else printf("Pixel Format %s is not supported by this camera\n\n",
                         pixel_format_to_str(desired_configuration.format).c_str());
        }
        if(!error)
        {
            if(binning_available)
                arv_camera_set_binning(camera,
                                       desired_configuration.x_binning,
                                       desired_configuration.y_binning,
                                       &error);
            else if(desired_configuration.x_binning != 0 || desired_configuration.y_binning != 0)
                printf("Binning is not available for this camera\n\n");
        }
        if(!error)
        {
            if(offset_available) arv_camera_set_region(camera,
                                                       desired_configuration.x_offset,
                                                       desired_configuration.y_offset,
                                                       desired_configuration.width,
                                                       desired_configuration.height,
                                                       &error);
            else if(desired_configuration.x_offset != 0 || desired_configuration.y_offset != 0)
            {
                printf("Region offset is not available for this camera\n\n");

                arv_camera_set_region(camera,
                                      0,
                                      0,
                                      desired_configuration.width,
                                      desired_configuration.height,
                                     &error);
            }
        }
        if(!error)
        {
            if(frame_rate_available)
                arv_camera_set_frame_rate(camera, desired_configuration.frame_rate, &error);
            else if(desired_configuration.frame_rate != 0.0)
                printf("Frame Rate is not available for this camera\n\n");
        }
        if(!error)
        {
            if(gain_available && desired_configuration.auto_gain != aravis_auto::ON)
                arv_camera_set_gain(camera, desired_configuration.gain, &error);
            else if(desired_configuration.auto_gain == aravis_auto::ON)
                printf("Gain is not available while auto gain is on\n\n");
            else if(desired_configuration.gain != 0.0)
                printf("Gain is not available for this camera\n\n");
        }
        if(!error)
        {
            if(exposure_available && desired_configuration.auto_exposure != aravis_auto::ON)
                arv_camera_set_exposure_time(camera, desired_configuration.exposure, &error);
            else if(desired_configuration.auto_exposure == aravis_auto::ON)
                printf("Gain is not available while auto exposure is on\n\n");
            else if(desired_configuration.exposure != 0.0)
                printf("Exposure is not available for this camera\n\n");
        }
        if(!error)
        {
            if(auto_gain_available)
            {
                // Converting from scoped c++ enum to c enum. This is gross.
                ArvAuto auto_val = static_cast<ArvAuto>
                    ((int)static_cast<typename std::underlying_type<aravis_auto>::type>
                        (desired_configuration.auto_gain));
                arv_camera_set_gain_auto(camera, auto_val, &error);
            }
            else printf("Auto Gain is not available for this camera\n\n");
        }
        if(!error)
        {
            if(auto_exposure_available)
            {
                // Converting from scoped c++ enum to c enum. This is gross.
                ArvAuto auto_val = static_cast<ArvAuto>
                    ((int)static_cast<typename std::underlying_type<aravis_auto>::type>
                        (desired_configuration.auto_exposure));
                arv_camera_set_exposure_time_auto(camera, auto_val, &error);
            }
            else printf("Auto Exposure is not available for this camera\n\n");
        }
        if(!error) arv_camera_gv_set_packet_size(camera, desired_configuration.mtu, &error);

        if (!error) {
            if (ptp_enable_available) {
                arv_device_set_boolean_feature_value(dev, "PtpEnable", desired_configuration.ptp_enable, &error);
            }
        }
        if (!error) {
            if (ptp_slave_only_available) {
                arv_device_set_boolean_feature_value(dev, "PtpSlaveOnly", desired_configuration.ptp_slave_only, &error);
            }
        }

        if(!error) new_config.format = pixel_format{arv_camera_get_pixel_format(camera, &error)};
        if(!error) arv_camera_get_region(camera,
                                        &new_config.x_offset,
                                        &new_config.y_offset,
                                        &new_config.width,
                                        &new_config.height,
                                        &error);
        if(!error) arv_camera_get_binning(camera,
                                         &new_config.x_binning,
                                         &new_config.y_binning,
                                         &error);
        if(!error) new_config.frame_rate     = arv_camera_get_frame_rate    (camera, &error);
        if(!error) new_config.gain           = arv_camera_get_gain          (camera, &error);
        if(!error) new_config.exposure       = arv_camera_get_exposure_time (camera, &error);
        if(!error) new_config.mtu            = arv_camera_gv_get_packet_size(camera, &error);
        if(!error) new_config.ptp_enable     = arv_device_get_boolean_feature_value (dev, "PtpEnable", &error);
        if(!error) new_config.ptp_slave_only = arv_device_get_boolean_feature_value (dev, "PtpSlaveOnly", &error);
        if(!error)
            new_config.auto_gain = aravis_auto{arv_camera_get_gain_auto(camera, &error)};
        if(!error)
            new_config.auto_exposure = aravis_auto{arv_camera_get_exposure_time_auto(camera,
                                                                                    &error)};

        if(!error)
        {
            printf("Camera Capabilities:\n");
            printf("    Pixel Format:  %s\n",
                   pixel_format_to_str((pixel_format)pixel_formats_available[0]).c_str());
            for(guint i = 1; i < number_of_formats; i++)
            {
                if(pixel_format_to_str((pixel_format)pixel_formats_available[i]).length() > 1)
                {
                    printf("                   %s\n",
                           pixel_format_to_str((pixel_format)pixel_formats_available[i]).c_str());
                }
            }
            printf("    Offset X:      %i <-> %i\n", x_offset_min,   x_offset_max);
            printf("    Offset Y:      %i <-> %i\n", y_offset_min,   y_offset_max);
            printf("    Width:         %i <-> %i\n", width_min,      width_max);
            printf("    Height:        %i <-> %i\n", height_min,     height_max);
            printf("    Binning X:     %i <-> %i\n", x_binning_min,  x_binning_max);
            printf("    Binning Y:     %i <-> %i\n", y_binning_min,  y_binning_max);
            printf("    Frame Rate:    %f <-> %f\n", frame_rate_min, frame_rate_max);
            printf("    Gain:          %f <-> %f\n", gain_min,       gain_max);
            printf("    Exposure Time: %f <-> %f\n", exposure_min,   exposure_max);
            printf("    Auto Gain:     %s\n"  , auto_gain_available     == true ? "true" : "false");
            printf("    Auto Exposure: %s\n", auto_exposure_available == true ? "true" : "false");
            printf("    PTP Enable:    %s\n", ptp_enable_available == true ? "true" : "false");
            printf("    Slave Only:    %s\n", ptp_slave_only_available == true ? "true" : "false");
            printf("Current Values:\n");
            printf("    Pixel Format:  %s\n", pixel_format_to_str(new_config.format).c_str());
            printf("    Offset X:      %i\n", new_config.x_offset);
            printf("    Offset Y:      %i\n", new_config.y_offset);
            printf("    Width:         %i\n", new_config.width);
            printf("    Height:        %i\n", new_config.height);
            printf("    Binning X:     %i\n", new_config.x_binning);
            printf("    Binning Y:     %i\n", new_config.y_binning);
            printf("    Frame Rate:    %f\n", new_config.frame_rate);
            printf("    Gain:          %f\n", new_config.gain);
            printf("    Exposure Time: %f\n", new_config.exposure);
            printf("    Auto Gain:     %s\n",
                   aravis_auto_to_str((aravis_auto)new_config.auto_gain).c_str());
            printf("    Auto Exposure: %s\n",
                   aravis_auto_to_str((aravis_auto)new_config.auto_exposure).c_str());
            printf("    MTU:           %u\n", new_config.mtu);
            printf("    PTP Enabled:   %s\n", new_config.ptp_enable ? "yes" : "no");
            printf("    Slave Only:    %s\n", new_config.ptp_slave_only ? "yes" : "no");
            printf("\n");
            if(current_config.format != new_config.format)
                printf("Pixel Format:  %s -> %s\n",
                        pixel_format_to_str(current_config.format).c_str(),
                        pixel_format_to_str(new_config.format).c_str());
            if(current_config.x_offset != new_config.x_offset)
                printf("Offset X:      %i -> %i\n", current_config.x_offset,   new_config.x_offset);
            if(current_config.y_offset != new_config.y_offset)
                printf("Offset Y:      %i -> %i\n", current_config.y_offset,   new_config.y_offset);
            if(current_config.width != new_config.width)
                printf("Width:         %i -> %i\n", current_config.width,      new_config.width);
            if(current_config.height != new_config.height)
                printf("Height:        %i -> %i\n", current_config.height,     new_config.height);
            if(current_config.x_binning != new_config.x_binning)
                printf("Binning X:     %i -> %i\n", current_config.x_binning,  new_config.x_binning);
            if(current_config.y_binning != new_config.y_binning)
                printf("Binning Y:     %i -> %i\n", current_config.y_binning,  new_config.y_binning);
            if(current_config.frame_rate != new_config.frame_rate)
                printf("Frame Rate:    %f -> %f\n", current_config.frame_rate, new_config.frame_rate);
            if(current_config.gain != new_config.gain)
                printf("Gain:          %f -> %f\n", current_config.gain,       new_config.gain);
            if(current_config.exposure != new_config.exposure)
                printf("Exposure Time: %f -> %f\n", current_config.exposure,   new_config.exposure);
            if(current_config.auto_gain != new_config.auto_gain)
                printf("Auto Gain:     %s -> %s\n",
                        aravis_auto_to_str((aravis_auto)current_config.auto_gain).c_str(),
                        aravis_auto_to_str((aravis_auto)new_config.auto_gain).c_str());
            if(current_config.auto_exposure != new_config.auto_exposure)
                printf("Auto Exposure: %s -> %s\n",
                        aravis_auto_to_str((aravis_auto)current_config.auto_exposure).c_str(),
                        aravis_auto_to_str((aravis_auto)new_config.auto_exposure).c_str());
            if(current_config.mtu != new_config.mtu)
                printf("MTU:           %u -> %u\n", current_config.mtu,        new_config.mtu);
            if(current_config.ptp_enable != new_config.ptp_enable)
                printf("PTP Enabled:   %s -> %s\n", current_config.ptp_enable ? "yes" : "no", new_config.ptp_enable ? "yes" : "no");
            if(current_config.ptp_slave_only != new_config.ptp_slave_only)
                printf("Slave Only:    %s -> %s\n", current_config.ptp_slave_only ? "yes" : "no", new_config.ptp_slave_only ? "yes" : "no");
        }

        g_clear_object(&dev);
        g_clear_object(&camera);
    }

    if(error != NULL)
    {
        printf("Error: %s\n", error->message);

        return 1;
    }
}
