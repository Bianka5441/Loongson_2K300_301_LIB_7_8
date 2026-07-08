#include "car_runtime.hpp"

#include "lq_common.hpp"

#ifdef LQ_HAVE_OPENCV
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#endif

namespace
{
bool car_runtime_initialized = false;
bool car_runtime_motor_enabled = false;
int car_runtime_motor_init_duty = 1000;

#ifdef LQ_HAVE_OPENCV
#ifndef LQ_TELEMETRY_HOST_IP
#define LQ_TELEMETRY_HOST_IP "192.168.31.33"
#endif

constexpr const char* kTelemetryHostIp = LQ_TELEMETRY_HOST_IP;
constexpr const char* kTelemetryImageMode = "binary_track";
constexpr uint16_t kTelemetryImagePort = 8080;
constexpr uint16_t kTelemetryStatusPort = 8083;
constexpr int kTelemetryJpegQuality = 85;
constexpr uint32_t kTelemetryFrameDivisor = 3;

int telemetry_socket_fd = -1;
sockaddr_in telemetry_image_addr{};
sockaddr_in telemetry_status_addr{};
uint32_t telemetry_image_seq = 0;
uint32_t telemetry_frame_divider = 0;

void close_telemetry_socket()
{
    if (telemetry_socket_fd >= 0)
    {
        close(telemetry_socket_fd);
        telemetry_socket_fd = -1;
    }
}

bool init_telemetry_socket()
{
    if (telemetry_socket_fd >= 0)
    {
        return true;
    }

    telemetry_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (telemetry_socket_fd < 0)
    {
        lq_log_error("Failed to create telemetry UDP socket");
        return false;
    }

    std::memset(&telemetry_image_addr, 0, sizeof(telemetry_image_addr));
    telemetry_image_addr.sin_family = AF_INET;
    telemetry_image_addr.sin_port = htons(kTelemetryImagePort);

    std::memset(&telemetry_status_addr, 0, sizeof(telemetry_status_addr));
    telemetry_status_addr.sin_family = AF_INET;
    telemetry_status_addr.sin_port = htons(kTelemetryStatusPort);

    if (inet_pton(AF_INET, kTelemetryHostIp, &telemetry_image_addr.sin_addr) != 1 ||
        inet_pton(AF_INET, kTelemetryHostIp, &telemetry_status_addr.sin_addr) != 1)
    {
        lq_log_error("Invalid telemetry host IP");
        close_telemetry_socket();
        return false;
    }

    return true;
}

void append_le32(std::vector<uint8_t>& data, uint32_t value)
{
    data.push_back(static_cast<uint8_t>((value >> 0) & 0xFF));
    data.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    data.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
    data.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
}

const char* road_type_name(RoadType_e road_type)
{
    switch (road_type)
    {
    case Normol:
        return "normal";
    case Straight:
        return "straight";
    case Cross:
        return "cross";
    case Ramp:
        return "ramp";
    case LeftCirque:
        return "left_ring";
    case RightCirque:
        return "right_ring";
    case Forkin:
        return "fork_in";
    case Forkout:
        return "fork_out";
    case Barn_out:
        return "barn_out";
    case Barn_in:
        return "barn_in";
    case Cross_ture:
        return "cross_true";
    default:
        return "unknown";
    }
}

void send_status_packet(uint32_t image_seq)
{
    char status[768];
    const int center_error = ImageStatus.Det_True - ImageStatus.MiddleLine;
    const int len = std::snprintf(status, sizeof(status),
                                  "status_image_seq=%u\n"
                                  "image_mode=%s\n"
                                  "road_type=%s\n"
                                  "err=%d\n"
                                  "det_true=%d\n"
                                  "middle_line=%u\n"
                                  "tow_point=%d\n"
                                  "off_line=%u\n"
                                  "white_line=%u\n"
                                  "pid_l=%d\n"
                                  "pid_r=%d\n"
                                  "spd_l=%d\n"
                                  "spd_r=%d\n"
                                  "target_l=%d\n"
                                  "target_r=%d\n"
                                  "pwm_max=%d\n"
                                  "terminal_line=err=%4d  pidL=%6d  pidR=%6d  spdL=%4d  spdR=%4d\n",
                                  image_seq,
                                  kTelemetryImageMode,
                                  road_type_name(ImageStatus.Road_type),
                                  center_error,
                                  ImageStatus.Det_True,
                                  ImageStatus.MiddleLine,
                                  ImageStatus.TowPoint_True,
                                  ImageStatus.OFFLine,
                                  ImageStatus.WhiteLine,
                                  Speed_PID_OUT_l,
                                  Speed_PID_OUT_r,
                                  encoder_Left,
                                  encoder_Right,
                                  Diff_SpeedL_expect,
                                  Diff_SpeedR_expect,
                                  PWM_Max,
                                  center_error,
                                  Speed_PID_OUT_l,
                                  Speed_PID_OUT_r,
                                  encoder_Left,
                                  encoder_Right);
    if (len > 0)
    {
        sendto(telemetry_socket_fd,
               status,
               static_cast<size_t>(len),
               0,
               reinterpret_cast<sockaddr*>(&telemetry_status_addr),
               sizeof(telemetry_status_addr));
    }
}

void mark_pixel(cv::Mat& image, int row, int col, const cv::Vec3b& color)
{
    if (row >= 0 && row < image.rows && col >= 0 && col < image.cols)
    {
        image.at<cv::Vec3b>(row, col) = color;
    }
}

void mark_column(cv::Mat& image, int row, int col, const cv::Vec3b& color)
{
    for (int dx = -1; dx <= 1; dx++)
    {
        mark_pixel(image, row, col + dx, color);
    }
}

void mark_row(cv::Mat& image, int row, const cv::Vec3b& color)
{
    if (row < 0 || row >= image.rows)
    {
        return;
    }

    for (int col = 0; col < image.cols; col += 2)
    {
        mark_pixel(image, row, col, color);
    }
}

cv::Mat build_binary_road_view()
{
    cv::Mat road_view(LCDH, LCDW, CV_8UC3);
    for (int row = 0; row < LCDH; row++)
    {
        cv::Vec3b* dst = road_view.ptr<cv::Vec3b>(row);
        for (int col = 0; col < LCDW; col++)
        {
            const uint8_t gray = Pixle[row][col] ? 255 : 0;
            dst[col] = cv::Vec3b(gray, gray, gray);
        }
    }

    const cv::Vec3b left_color(255, 0, 0);
    const cv::Vec3b right_color(0, 0, 255);
    const cv::Vec3b center_color(0, 255, 0);
    const cv::Vec3b target_color(0, 255, 255);
    const cv::Vec3b middle_color(255, 255, 0);
    const cv::Vec3b off_line_color(255, 0, 255);

    for (int row = ImageStatus.OFFLine; row < LCDH; row++)
    {
        mark_column(road_view, row, ImageDeal[row].LeftBorder, left_color);
        mark_column(road_view, row, ImageDeal[row].RightBorder, right_color);
        mark_pixel(road_view, row, ImageDeal[row].Center, center_color);
        mark_pixel(road_view, row, ImageStatus.MiddleLine, middle_color);
    }

    const int tow_point = ImageStatus.TowPoint_True > 0 ? ImageStatus.TowPoint_True : ImageStatus.TowPoint;
    mark_row(road_view, ImageStatus.OFFLine, off_line_color);
    mark_row(road_view, tow_point, target_color);

    for (int row = tow_point - 2; row <= tow_point + 2; row++)
    {
        mark_column(road_view, row, ImageStatus.Det_True, target_color);
    }

    return road_view;
}

void send_image_packet(const cv::Mat& frame)
{
    if (frame.empty())
    {
        return;
    }

    telemetry_frame_divider++;
    if (telemetry_frame_divider < kTelemetryFrameDivisor)
    {
        return;
    }
    telemetry_frame_divider = 0;

    if (!init_telemetry_socket())
    {
        return;
    }

    std::vector<uint8_t> jpeg;
    cv::Mat road_view = build_binary_road_view();
    const std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, kTelemetryJpegQuality};
    if (!cv::imencode(".jpg", road_view, jpeg, params) || jpeg.empty())
    {
        return;
    }

    telemetry_image_seq++;
    std::vector<uint8_t> packet;
    packet.reserve(12 + jpeg.size());
    packet.push_back('L');
    packet.push_back('Q');
    packet.push_back('I');
    packet.push_back('M');
    append_le32(packet, telemetry_image_seq);
    append_le32(packet, static_cast<uint32_t>(jpeg.size()));
    packet.insert(packet.end(), jpeg.begin(), jpeg.end());

    sendto(telemetry_socket_fd,
           packet.data(),
           packet.size(),
           0,
           reinterpret_cast<sockaddr*>(&telemetry_image_addr),
           sizeof(telemetry_image_addr));

    send_status_packet(telemetry_image_seq);
}
#endif
}

void CarRuntime_Init(bool enable_motor, int motor_init_duty)
{
    Data_Settings();
    car_runtime_motor_enabled = enable_motor;
    car_runtime_motor_init_duty = motor_init_duty;

    if (enable_motor)
    {
        Motor_Init1(motor_init_duty);
        Motor_Argument();
    }

    car_runtime_initialized = true;
}

void CarRuntime_Shutdown(bool disable_motor)
{
    if (disable_motor && car_runtime_motor_enabled)
    {
        Motor_Disable1();
    }

    cleanup();
#ifdef LQ_HAVE_OPENCV
    close_telemetry_socket();
#endif
    car_runtime_initialized = false;
}

#ifdef LQ_HAVE_OPENCV
bool CarRuntime_ProcessFrame(const cv::Mat& frame, bool enable_motor)
{
    if (!car_runtime_initialized || car_runtime_motor_enabled != enable_motor)
    {
        CarRuntime_Init(enable_motor, car_runtime_motor_init_duty);
    }

    First_image = frame;
    if (First_image.empty())
    {
        return false;
    }

    ImageProcess();
    if (enable_motor)
    {
        Motor_Control();
    }

    return true;
}
#endif

bool CarRuntime_RunCameraLoop(bool enable_motor,
                              uint16_t width,
                              uint16_t height,
                              uint16_t fps,
                              int motor_init_duty,
                              uint32_t empty_frame_delay_us,
                              uint32_t loop_delay_us)
{
#ifndef LQ_HAVE_OPENCV
    (void)enable_motor;
    (void)width;
    (void)height;
    (void)fps;
    (void)motor_init_duty;
    (void)empty_frame_delay_us;
    (void)loop_delay_us;
    lq_log_error("OpenCV not enabled, car runtime is unavailable");
    return false;
#else
    lq_camera cam(width, height, fps);
    if (!cam.is_opened())
    {
        lq_log_error("Failed to open camera for car runtime");
        return false;
    }

    CarRuntime_Init(enable_motor, motor_init_duty);

    while (1)
    {
        cv::Mat frame = cam.get_raw_frame();
        if (!CarRuntime_ProcessFrame(frame, enable_motor))
        {
            usleep(empty_frame_delay_us);
            continue;
        }

        send_image_packet(frame);
        usleep(loop_delay_us);
    }

    return true;
#endif
}
