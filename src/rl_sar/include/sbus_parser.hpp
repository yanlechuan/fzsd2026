/**
 * @file sbus_parser.hpp
 * @brief ET08A W.BUS (S.BUS兼容) 协议解析器
 *
 * 协议: 25字节帧, 100000波特率
 * 帧格式: [00 0F header] [22 bytes S.BUS payload] [03 end]
 * S.BUS payload: 16通道 × 11-bit = 176 bits = 22 bytes
 *
 * ET08A 通道映射 (默认):
 *   CH1=副翼(Roll)  CH2=升降(Pitch)  CH3=油门(Throttle)  CH4=方向(Yaw)
 *   CH5=SA(2档)     CH6=SB(3档)      CH7=SC(3档)          CH8=SD(2档)
 * 开关值: HIGH=353, MID=1024, LOW=1694
 * SA/SD(2档): 切换时出现瞬态1024 (无物理卡口)
 */

#ifndef SBUS_PARSER_HPP
#define SBUS_PARSER_HPP

#include <atomic>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>
#include <cstdio>
#include <cerrno>

#ifdef __linux__
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

// 手动定义 termios2, 避免 asm/termbits.h 与 termios.h 冲突
#ifndef TCGETS2
#define TCGETS2 0x802C542A
#endif
#ifndef TCSETS2
#define TCSETS2 0x402C542B
#endif
#ifndef BOTHER
#define BOTHER  0x1000   // 0o010000 = 4096
#endif
#ifndef CBAUD
#define CBAUD   0x100F   // 0o010017 = 4111
#endif

struct termios2
{
    tcflag_t c_iflag;
    tcflag_t c_oflag;
    tcflag_t c_cflag;
    tcflag_t c_lflag;
    cc_t     c_line;
    cc_t     c_cc[19];
    speed_t  c_ispeed;
    speed_t  c_ospeed;
};
#endif

namespace sbus
{

// ============================================================
// 协议常量
// ============================================================
constexpr int    SBUS_FRAME_SIZE    = 25;   // 完整帧长度
constexpr int    SBUS_PAYLOAD_SIZE  = 22;   // S.BUS 有效载荷 (16ch × 11bit)
constexpr int    SBUS_NUM_CHANNELS  = 16;   // 通道数
constexpr int    SBUS_CHANNEL_BITS  = 11;   // 每通道位数
constexpr int    SBUS_HEADER_OFFSET = 0;    // 帧头偏移
constexpr int    SBUS_PAYLOAD_OFFSET= 2;    // 载荷起始偏移 (跳过 00 0F)
constexpr int    SBUS_FLAGS_OFFSET  = 24;   // 标志位偏移
constexpr uint8_t SBUS_HEADER_BYTE0 = 0x00;
constexpr uint8_t SBUS_HEADER_BYTE1 = 0x0F;
constexpr uint8_t SBUS_END_BYTE     = 0x03; // ET08A 使用 0x03, 标准 S.BUS 使用 0x00

// 固定 trailer, 用于帧同步 (channels 9-16 未连接时的固定位模式)
constexpr uint8_t FIXED_TRAILER[12] = {
    0x00, 0x04, 0x20, 0x00, 0x01, 0x08,
    0x40, 0x00, 0x02, 0x10, 0x00, 0x03
};
constexpr int    FIXED_TRAILER_SIZE = 12;
constexpr int    TRAILER_HEADER_DIST = 13; // trailer_start - header_start = 13

// 通道值范围
constexpr int    SBUS_MIN_VAL  = 192;
constexpr int    SBUS_MAX_VAL  = 1792;
constexpr int    SBUS_MID_VAL  = 1024;
constexpr int    SWITCH_HIGH   = 353;
constexpr int    SWITCH_MID    = 1024;
constexpr int    SWITCH_LOW    = 1694;

// 位掩码
constexpr uint32_t CHANNEL_MASK = 0x07FF; // 11-bit

// ============================================================
// 帧结构
// ============================================================
struct Frame
{
    uint16_t channels[SBUS_NUM_CHANNELS] = {0};
    bool frame_lost = false;
    bool failsafe   = false;
    bool valid      = false;

    void reset()
    {
        std::memset(channels, 0, sizeof(channels));
        frame_lost = false;
        failsafe   = false;
        valid      = false;
    }
};

// ============================================================
// 解码函数: 从22字节载荷解出16通道
// ============================================================
inline void decodePayload(const uint8_t *payload, Frame &frame)
{
    // 标准 S.BUS 位提取: 每通道11-bit, 按位偏移从字节流中提取
    for (int i = 0; i < SBUS_NUM_CHANNELS; ++i)
    {
        int start_bit  = i * SBUS_CHANNEL_BITS;
        int byte_idx   = start_bit / 8;
        int bit_offset = start_bit % 8;

        if (byte_idx + 2 < SBUS_PAYLOAD_SIZE)
        {
            uint32_t val = static_cast<uint32_t>(payload[byte_idx])
                         | (static_cast<uint32_t>(payload[byte_idx + 1]) << 8)
                         | (static_cast<uint32_t>(payload[byte_idx + 2]) << 16);
            frame.channels[i] = static_cast<uint16_t>((val >> bit_offset) & CHANNEL_MASK);
        }
        else if (byte_idx + 1 < SBUS_PAYLOAD_SIZE)
        {
            uint32_t val = static_cast<uint32_t>(payload[byte_idx])
                         | (static_cast<uint32_t>(payload[byte_idx + 1]) << 8);
            frame.channels[i] = static_cast<uint16_t>((val >> bit_offset) & CHANNEL_MASK);
        }
        else
        {
            frame.channels[i] = 0;
        }
    }

    // 标志位: byte 23 (帧内偏移)
    // 标准 S.BUS: bit2=frame_lost, bit3=failsafe
    // ET08A: byte[24]=0x03, 这里从 flags 字节读取
}

// ============================================================
// 帧查找: 在字节流中搜索完整帧
// 返回: true=找到帧, false=需要更多数据
// ============================================================
inline bool findFrame(const uint8_t *data, size_t len, size_t &consumed, Frame &frame)
{
    if (len < SBUS_FRAME_SIZE)
    {
        consumed = 0;
        return false;
    }

    // 搜索固定 trailer
    for (size_t i = 0; i + SBUS_FRAME_SIZE <= len; ++i)
    {
        // 检查 trailer 是否匹配
        if (std::memcmp(data + i + TRAILER_HEADER_DIST, FIXED_TRAILER, FIXED_TRAILER_SIZE) != 0)
            continue;

        // 验证 header
        if (data[i] != SBUS_HEADER_BYTE0 || data[i + 1] != SBUS_HEADER_BYTE1)
            continue;

        // 找到有效帧
        const uint8_t *frame_ptr = data + i;

        // 解码载荷
        decodePayload(frame_ptr + SBUS_PAYLOAD_OFFSET, frame);

        // 标志位
        uint8_t flags = frame_ptr[SBUS_FLAGS_OFFSET];
        frame.frame_lost = (flags & 0x04) != 0;
        frame.failsafe   = (flags & 0x08) != 0;
        frame.valid      = true;

        consumed = i + SBUS_FRAME_SIZE;
        return true;
    }

    // 未找到帧，保留尾部数据
    consumed = (len >= SBUS_FRAME_SIZE) ? (len - SBUS_FRAME_SIZE + 1) : 0;
    return false;
}

// ============================================================
// 串口读取器 (Linux termios)
// 使用后台线程持续读取串口并解析帧
// ============================================================
class Reader
{
public:
    Reader() = default;
    ~Reader() { stop(); }

    // 打开串口 (返回是否成功)
    bool open(const std::string &port, int baud_rate)
    {
        port_ = port;
        baud_ = baud_rate;

#ifdef __linux__
        fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0)
        {
            std::perror(("[SBUS] open(" + port_ + ")").c_str());
            return false;
        }

        // 使用 termios2 设置自定义波特率 (ET08A = 100000)
        struct termios2 tty2;
        std::memset(&tty2, 0, sizeof(tty2));
        if (ioctl(fd_, TCGETS2, &tty2) != 0)
        {
            std::perror("[SBUS] TCGETS2");
            ::close(fd_);
            fd_ = -1;
            return false;
        }

        tty2.c_cflag &= ~CBAUD;
        tty2.c_cflag |= BOTHER;
        tty2.c_ispeed = static_cast<speed_t>(baud_);
        tty2.c_ospeed = static_cast<speed_t>(baud_);

        // 8N2 (W.BUS 使用 2 个停止位)
        tty2.c_cflag &= ~PARENB;
        tty2.c_cflag &= ~CSIZE;
        tty2.c_cflag |= CS8;
        tty2.c_cflag |= CSTOPB;   // 2 stop bits
        tty2.c_cflag &= ~CRTSCTS; // 无硬件流控
        tty2.c_cflag |= CREAD | CLOCAL;

        tty2.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty2.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | ISTRIP);
        tty2.c_oflag &= ~OPOST;
        tty2.c_cc[VMIN]  = 0;
        tty2.c_cc[VTIME] = 1;  // 100ms 超时

        if (ioctl(fd_, TCSETS2, &tty2) != 0)
        {
            std::perror("[SBUS] TCSETS2");
            ::close(fd_);
            fd_ = -1;
            return false;
        }

        // 清空缓冲区
        tcflush(fd_, TCIOFLUSH);

        opened_ = true;
        return true;
#else
        // 非 Linux 平台: 仅记录, 实际不打开
        opened_ = true;
        return true;
#endif
    }

    // 启动后台读取线程 (open 之后调用)
    void start()
    {
        if (!opened_ || running_.load())
            return;
        running_.store(true);
        thread_ = std::thread(&Reader::readLoop, this);
    }

    // 停止后台线程并关闭串口
    void stop()
    {
        running_.store(false);
        if (thread_.joinable())
            thread_.join();
#ifdef __linux__
        if (fd_ >= 0)
        {
            ::close(fd_);
            fd_ = -1;
        }
#endif
        opened_ = false;
    }

    // 获取最新解码帧 (非阻塞)
    bool getLatest(Frame &frame)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!latest_frame_.valid)
            return false;
        frame = latest_frame_;
        return true;
    }

    bool isRunning() const { return running_.load(); }
    bool isOpen()    const { return opened_; }

private:
    void readLoop()
    {
        std::vector<uint8_t> buffer;
        buffer.reserve(4096);

        while (running_.load())
        {
#ifdef __linux__
            uint8_t tmp[256];
            ssize_t n = ::read(fd_, tmp, sizeof(tmp));
            if (n > 0)
                buffer.insert(buffer.end(), tmp, tmp + n);
            else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
                break;
#else
            // 非 Linux: 休眠
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
#endif

            // 解析所有完整帧
            size_t offset = 0;
            while (offset + SBUS_FRAME_SIZE <= buffer.size())
            {
                size_t consumed = 0;
                Frame frame;
                if (findFrame(buffer.data() + offset, buffer.size() - offset, consumed, frame))
                {
                    if (frame.valid && !frame.frame_lost && !frame.failsafe)
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        latest_frame_ = frame;
                    }
                    offset += consumed;
                }
                else
                {
                    break;
                }
            }

            // 清理已处理数据
            if (offset > 0)
                buffer.erase(buffer.begin(), buffer.begin() + static_cast<long>(offset));

            // 防止缓冲区无限增长
            if (buffer.size() > 8192)
            {
                // 搜索 header 00 0F, 从那里开始
                auto it = std::search(buffer.begin(), buffer.end(),
                                       &SBUS_HEADER_BYTE0, &SBUS_HEADER_BYTE0 + 2);
                if (it != buffer.end())
                    buffer.erase(buffer.begin(), it);
                else
                    buffer.clear();
            }
        }
    }

    std::string port_;
    int         baud_   = 100000;
    bool        opened_ = false;
    std::atomic<bool> running_{false};
    std::thread thread_;
    std::mutex  mutex_;
    Frame       latest_frame_;
    int         fd_ = -1;
};

} // namespace sbus

#endif // SBUS_PARSER_HPP
