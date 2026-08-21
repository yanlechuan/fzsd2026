/**
 * @file rc_input_mapper.hpp
 * @brief RC遥控器通道 → Gamepad/摇杆映射
 *
 * ET08A 通道分配:
 *   摇杆:  CH1=右摇杆横  CH2=左摇杆纵  CH4=左摇杆横
 *   旋钮:  CH8=RD (连续, 0~1, 映射到 knobs)
 *   开关:  CH5=SA(2档)  CH6=SB(3档)  CH7=SC(3档)
 *
 * 开关值: HIGH=353, MID=1024, LOW=1694
 * SA(2档): 切换时出现瞬态1024 (无物理卡口)
 *
 * 去抖策略:
 *   SA(2档): 连续 N 帧稳定在端点(HIGH/LOW)才确认切换,
 *             中间值(1024±容差)被视为瞬态, 保持上一有效状态
 *   SB/SC(3档): 连续 N 帧稳定在同一档位才确认切换
 */

#ifndef RC_INPUT_MAPPER_HPP
#define RC_INPUT_MAPPER_HPP

#include "sbus_parser.hpp"
#include "rl_sdk.hpp"  // Input::Gamepad
#include <cstdlib>     // std::abs

namespace rc
{

// ============================================================
// 开关档位枚举
// ============================================================
enum class SwitchPos
{
    HIGH = 0,  // 353
    MID  = 1,  // 1024 (仅3档开关)
    LOW  = 2,  // 1694
    UNKNOWN = 3
};

// ============================================================
// 单通道去抖状态
// ============================================================
struct ChannelDebounce
{
    int         stable_count  = 0;        // 当前值连续稳定帧数
    SwitchPos   current_pos   = SwitchPos::UNKNOWN;
    SwitchPos   pending_pos   = SwitchPos::UNKNOWN;
    int         raw_pending   = 0;
    bool        is_2pos       = false;    // 是否为2档开关

    // 根据原始值判断档位
    SwitchPos classify(int raw, int tolerance) const
    {
        if (std::abs(raw - sbus::SWITCH_HIGH) < tolerance) return SwitchPos::HIGH;
        if (std::abs(raw - sbus::SWITCH_LOW)  < tolerance) return SwitchPos::LOW;
        if (!is_2pos && std::abs(raw - sbus::SWITCH_MID) < tolerance) return SwitchPos::MID;
        return SwitchPos::UNKNOWN;
    }

    // 更新去抖状态, 返回是否发生了有效切换
    bool update(int raw, bool &changed, SwitchPos &new_pos, int debounce_frames, int tolerance)
    {
        changed = false;
        SwitchPos classified = classify(raw, tolerance);

        if (is_2pos && classified == SwitchPos::UNKNOWN)
        {
            // 2档开关瞬态: 保持当前状态, 重置计数
            stable_count = 0;
            pending_pos  = current_pos;
            return false;
        }

        if (classified == SwitchPos::UNKNOWN)
        {
            stable_count = 0;
            return false;
        }

        if (classified == pending_pos)
        {
            stable_count++;
            if (stable_count >= debounce_frames && classified != current_pos)
            {
                current_pos  = classified;
                changed      = true;
                new_pos      = classified;
                stable_count = 0;
                return true;
            }
        }
        else
        {
            pending_pos  = classified;
            stable_count = 1;
        }

        return false;
    }

    void reset(SwitchPos initial = SwitchPos::UNKNOWN)
    {
        stable_count = 0;
        current_pos  = initial;
        pending_pos  = initial;
    }
};

// ============================================================
// RC → Gamepad 映射器
// ============================================================
class Mapper
{
public:
    Mapper() = default;

    // ============================================================
    // 设置通道映射 (0-based 索引, 对应 S.BUS CH1-16)
    // ============================================================
    void setChannelMap(int ch_roll, int ch_pitch, int ch_throttle,
                       int ch_yaw, int ch_sa, int ch_sb,
                       int ch_sc, int ch_rd)
    {
        ch_roll_     = ch_roll;
        ch_pitch_    = ch_pitch;
        ch_throttle_ = ch_throttle;
        ch_yaw_      = ch_yaw;
        ch_sa_       = ch_sa;
        ch_sb_       = ch_sb;
        ch_sc_       = ch_sc;
        ch_rd_       = ch_rd;

        // 标记2档开关 (RD 是连续旋钮, 不去抖)
        debounce_[ch_sa_].is_2pos = true;
        debounce_[ch_sb_].is_2pos = false;
        debounce_[ch_sc_].is_2pos = false;
    }

    // ============================================================
    // 处理一帧数据, 输出 Gamepad、摇杆值、旋钮值
    // rd: RD 旋钮 [0, 1], 用于高度/速度等连续调节
    // 返回 true 表示数据有效
    // ============================================================
    bool process(const sbus::Frame &frame, Input::Gamepad &gp,
                 float &x, float &y, float &yaw, float &rd)
    {
        if (!frame.valid || frame.frame_lost || frame.failsafe)
            return false;

        // 读取各通道原始值
        int raw_roll     = getChannel(frame, ch_roll_);
        int raw_pitch    = getChannel(frame, ch_pitch_);
        int raw_throttle = getChannel(frame, ch_throttle_);
        int raw_yaw      = getChannel(frame, ch_yaw_);
        int raw_sa       = getChannel(frame, ch_sa_);
        int raw_sb       = getChannel(frame, ch_sb_);
        int raw_sc       = getChannel(frame, ch_sc_);
        int raw_rd       = getChannel(frame, ch_rd_);

        // --- 摇杆映射: S.BUS值 → [-1, 1] ---
        // ET08A Mode 2 布局:
        //   CH1 = 右摇杆横 (roll)   → yaw (机身旋转)
        //   CH2 = 左摇杆纵 (pitch)  → x   (前进/后退)
        //   CH4 = 左摇杆横 (yaw)    → y   (左/右平移)
        yaw = -mapStick(raw_roll,    sbus::SBUS_MID_VAL);  // CH1 → yaw (旋转)
        x   = -mapStick(raw_pitch,   sbus::SBUS_MID_VAL);  // CH2 → x (前进/后退)
        y   = -mapStick(raw_yaw,     sbus::SBUS_MID_VAL);  // CH4 → y (左/右平移)

        // RD 旋钮: S.BUS 352~1694 → [0, 1]
        rd  =  mapKnob(raw_rd);

        // 摇杆死区
        if (std::abs(x)   < stick_deadzone_) x   = 0.0f;
        if (std::abs(y)   < stick_deadzone_) y   = 0.0f;
        if (std::abs(yaw) < stick_deadzone_) yaw = 0.0f;

        // --- 开关去抖 ---
        bool sa_changed = false, sb_changed = false;
        bool sc_changed = false;
        SwitchPos sa_pos = SwitchPos::UNKNOWN, sb_pos = SwitchPos::UNKNOWN;
        SwitchPos sc_pos = SwitchPos::UNKNOWN;

        debounce_[ch_sa_].update(raw_sa, sa_changed, sa_pos, debounce_frames_, tolerance_);
        debounce_[ch_sb_].update(raw_sb, sb_changed, sb_pos, debounce_frames_, tolerance_);
        debounce_[ch_sc_].update(raw_sc, sc_changed, sc_pos, debounce_frames_, tolerance_);

        // --- 开关 → Gamepad 映射 (边沿触发: 只在变化时输出, 保持时不重复发送) ---
        // 优先级: SC (急停/起身) > SB (电机使能/禁用) > SA (步态切换)

        Input::Gamepad new_gp = Input::Gamepad::None;

        // SC: LOW → LB_X (Passive, 急停), HIGH → A (GetUp)
        if (sc_pos == SwitchPos::LOW)
            new_gp = Input::Gamepad::LB_X;
        else if (sc_pos == SwitchPos::HIGH)
            new_gp = Input::Gamepad::A;
        // SB: HIGH → LB_A (电机使能 fix.sh), LOW → LB_B (电机禁用)
        else if (sb_pos == SwitchPos::HIGH)
            new_gp = Input::Gamepad::LB_A;
        else if (sb_pos == SwitchPos::LOW)
            new_gp = Input::Gamepad::LB_B;
        // SA: HIGH → RB_DPadUp (RL1), LOW → RB_DPadDown (RL2)
        else if (sa_pos == SwitchPos::HIGH)
            new_gp = Input::Gamepad::RB_DPadUp;
        else if (sa_pos == SwitchPos::LOW)
            new_gp = Input::Gamepad::RB_DPadDown;

        // 边沿检测: 只在 Gamepad 值变化时输出, 保持时输出 None
        if (new_gp != last_gamepad_)
        {
            last_gamepad_ = new_gp;
            gp = new_gp;
        }
        else
        {
            gp = Input::Gamepad::None;
        }

        return true;
    }

    // ============================================================
    // 配置调整
    // ============================================================
    void setStickDeadzone(float dz)        { stick_deadzone_ = dz; }
    void setDebounceFrames(int frames)     { debounce_frames_ = frames; }
    void setTolerance(int tolerance)       { tolerance_ = tolerance; }

    // 获取当前去抖后的开关状态 (用于调试)
    SwitchPos getSaPos() const { return debounce_[ch_sa_].current_pos; }
    SwitchPos getSbPos() const { return debounce_[ch_sb_].current_pos; }
    SwitchPos getScPos() const { return debounce_[ch_sc_].current_pos; }

private:
    // 通道索引 (0-based)
    int ch_roll_     = 0;
    int ch_pitch_    = 1;
    int ch_throttle_ = 2;
    int ch_yaw_      = 3;
    int ch_sa_       = 4;
    int ch_sb_       = 5;
    int ch_sc_       = 6;
    int ch_rd_       = 7;  // RD 旋钮 (原 SD)

    // 每个通道的去抖状态 (按通道索引)
    ChannelDebounce debounce_[16];

    float stick_deadzone_   = 0.05f;
    int   debounce_frames_  = 20;   // 去抖帧数 (~200ms @ 100Hz), 防止2档开关瞬态误触发
    int   tolerance_        = 120;  // 档位判断容差
    Input::Gamepad last_gamepad_ = Input::Gamepad::None;  // 边沿触发记忆

    // 安全读取通道值
    static int getChannel(const sbus::Frame &frame, int ch_idx)
    {
        if (ch_idx < 0 || ch_idx >= sbus::SBUS_NUM_CHANNELS)
            return 0;
        return static_cast<int>(frame.channels[ch_idx]);
    }

    // 摇杆映射: raw → [-1, 1]
    // center_val: 摇杆中位值 (1024)
    static float mapStick(int raw, int center_val)
    {
        if (raw <= center_val)
        {
            // 负方向: [192, center]
            float range = static_cast<float>(center_val - sbus::SBUS_MIN_VAL);
            if (range < 1.0f) return 0.0f;
            return -static_cast<float>(center_val - raw) / range;
        }
        else
        {
            // 正方向: (center, 1792]
            float range = static_cast<float>(sbus::SBUS_MAX_VAL - center_val);
            if (range < 1.0f) return 0.0f;
            return static_cast<float>(raw - center_val) / range;
        }
    }

    // 旋钮映射: raw → [0, 1], S.BUS 352~1694 → [0, 1]
    static float mapKnob(int raw)
    {
        constexpr float kmin = 352.0f;
        constexpr float kmax = 1694.0f;
        float val = static_cast<float>(raw);
        if (val < kmin) val = kmin;
        if (val > kmax) val = kmax;
        return (val - kmin) / (kmax - kmin);
    }
};

} // namespace rc

#endif // RC_INPUT_MAPPER_HPP
