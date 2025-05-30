#include "mbed.h"
#include "C620.hpp"
#include "PID_new.hpp"
#include <map>
#include <array>

BufferedSerial pc(USBTX, USBRX, 115200);
CAN can1(PA_11, PA_12, 1e6);
dji::C620 robomas(PB_12, PB_13);
PidGain roger_gain = {1, 1, 1};
PidGain lift_gain = {1, 1, 1};
PidGain roller_gain = {1, 1, 1};
constexpr int robomas_amount = 6;

std::array<Pid, robomas_amount> pid =
    {Pid({roger_gain, -1, 1}),
    Pid({roger_gain, -1, 1}),
    Pid({lift_gain, -1, 1}),
    Pid({lift_gain, -1, 1}),
    Pid({roller_gain, -1, 1}),
    Pid({roller_gain, -1, 1})};

constexpr int can_id = 3;
CANMessage msg1;

bool readline(BufferedSerial &serial, char *buffer, size_t size, bool is_integar = false, bool is_float = false);
float duration_to_sec(const std::chrono::duration<float> &duration);


enum class state
{
    FRONT,
    STOP,
    BACK
};
struct Ps5
{
    int8_t lstick_x = 0;
    int8_t lstick_y = 0;
    int8_t rstick_x = 0;
    int8_t rstick_y = 0;
    uint8_t l2 = 0;
    uint8_t r2 = 0;

    bool right = 0;
    bool up = 0;
    bool left = 0;
    bool down = 0;
    bool circle = 0;
    bool triangle = 0;
    bool square = 0;
    bool cross = 0;
    bool l1 = 0;
    bool r1 = 0;
    bool l3 = 0;
    bool r3 = 0;
    bool option = 0;
    bool share = 0;

    void parse(CANMessage msg)
    {
        switch (msg.id)
        {
            case 50:
            lstick_x = msg.data[0];
            lstick_y = msg.data[1];
            rstick_x = msg.data[2];
            rstick_y = msg.data[3];
            l2 = msg.data[4];
            r2 = msg.data[5];
            break;

            case 51:
            right = msg.data[0] >> 3 & 1;
            up = msg.data[0] >> 2 & 1;
            left = msg.data[0] >> 1 & 1;
            down = msg.data[0] & 1;
            circle = msg.data[1] >> 3 & 1;
            triangle = msg.data[1] >> 2 & 1;
            square = msg.data[1] >> 1 & 1;
            cross = msg.data[1] & 1;
            l1 = msg.data[2];
            r1 = msg.data[3];
            l3 = msg.data[4];
            r3 = msg.data[5];
            option = msg.data[6];
            share = msg.data[7];
            break;
        }
    }

    bool read(CAN& can)
    {
        CANMessage msg;
        if (can.read(msg); msg.id == 50 || msg.id == 51)
        {
            parse(msg);
            return true;
        }
        return false;
    }
};

constexpr int CROW_SPEED = 15000;
constexpr int PYLON_SPEED = 10000;
constexpr int ROGER_SPEED = 5000;
constexpr int ROLLER_PUSH = 7000;
constexpr int ROLLER_ROT_SPEED = 5000;
constexpr int MURE_LIFT_SPEED = 2500;

const std::map<state, int> CROW_SPEED_MAP = 
{
    {state::FRONT, CROW_SPEED},
    {state::BACK, -CROW_SPEED},
    {state::STOP, 0}
};

const std::map<state, int> PYLON_SPEED_MAP = 
{
    {state::FRONT, PYLON_SPEED},
    {state::BACK, -PYLON_SPEED},
    {state::STOP, 0}
};

const std::map<state, int> ROLLER_PUSH_MAP = 
{
    {state::FRONT, ROLLER_PUSH},
    {state::BACK, -ROLLER_PUSH},
    {state::STOP, 0}
};

const std::map<state, int> ROLLER_ROT_SPEED_MAP = 
{
    {state::FRONT, ROLLER_ROT_SPEED},
    {state::BACK, -ROLLER_ROT_SPEED},
};

const std::map<state, int> ROGER_SPEED_MAP = 
{
    {state::FRONT, ROGER_SPEED},
    {state::BACK, -ROGER_SPEED},
    {state::STOP, 0}
};

const std::map<state, int> MURE_LIFT_SPEED_MAP = 
{
    {state::FRONT, MURE_LIFT_SPEED},
    {state::BACK, -MURE_LIFT_SPEED},
    {state::STOP, 0}
};

int main()
{
    Ps5 ps5;
    auto zozo_crow = state::STOP;
    auto pylon_rack = state::STOP;
    auto roller_push = state::STOP;
    auto roller_rot = state::STOP;
    auto roger = state::STOP;
    auto lift = state::STOP;
    int pillar_push = 0;

    constexpr int max_pillar_pwr = 10000;


    int16_t can_pwr1[4] = {0};
    // id1, 2: Roger id3, 4: むれ持ち上げ id5, 6: むれローラー
    int16_t robomas_rpm[6] = {0};

    for (int i = 0; i < robomas_amount; ++i)
    {
        pid[i].reset();
    }

    while(1)
    {
        auto now = HighResClock::now();
        static auto pre = now;
        static bool pre_square = 0;
        static bool pre_circle = 0;
        static bool pre_triangle = 0;

        if (ps5.read(can1))
        {
            pillar_push = ps5.l2 > ps5.r2 ? ps5.l2 / 255.0 * max_pillar_pwr : ps5.r2 / 255.0 * max_pillar_pwr * -1;
            roller_push = ps5.left ? state::FRONT : ps5.right ? state::BACK : state::STOP;

            roger = ps5.l1 ? state::FRONT : ps5.l2 ? state::BACK : state::STOP;
            lift = ps5.up ? state::FRONT : ps5.down ? state::BACK : state::STOP;

            if (ps5.square == 1 && pre_square == 0)
            {
                switch (pylon_rack)
                {
                    case state::STOP:
                    pylon_rack = state::FRONT;
                    break;
                    case state::FRONT:
                    pylon_rack = state::BACK;
                    break;
                    case state::BACK:
                    pylon_rack = state::STOP;
                    break;
                }
            }
            if (ps5.circle == 1 && pre_circle == 0)
            {
                switch (zozo_crow)
                {
                    case state::STOP:
                    zozo_crow = state::FRONT;
                    break;
                    case state::FRONT:
                    zozo_crow = state::BACK;
                    break;
                    case state::BACK:
                    zozo_crow = state::STOP;
                    break;
                }
            }
            if (ps5.triangle && !pre_triangle)
            {
                switch (roller_rot)
                {
                    case state::STOP:
                    roller_rot = state::FRONT;
                    break;
                    case state::FRONT:
                    roller_rot = state::STOP;
                    break;
                }
            }
            pre_square = ps5.square;
            pre_circle = ps5.circle;
            pre_triangle = ps5.triangle;
        }

        can_pwr1[0] = CROW_SPEED_MAP.at(pylon_rack);
        can_pwr1[1] = pillar_push;
        can_pwr1[2] = ROLLER_PUSH_MAP.at(roller_push);
        can_pwr1[3] = PYLON_SPEED_MAP.at(zozo_crow);

        robomas_rpm[0] = ROGER_SPEED_MAP.at(roger);
        robomas_rpm[1] = -ROGER_SPEED_MAP.at(roger);
        robomas_rpm[2] = MURE_LIFT_SPEED_MAP.at(lift);
        robomas_rpm[3] = -MURE_LIFT_SPEED_MAP.at(lift);
        robomas_rpm[4] = roller_rot == state::FRONT ? ROLLER_ROT_SPEED : 0;
        robomas_rpm[5] = roller_rot == state::FRONT ? -ROLLER_ROT_SPEED : 0;

        if(now - pre > 10ms) // CAN送信など制御信号の送信を行うスコープ
        {
            float elapsed = duration_to_sec(now - pre);

            for (int i = 0; i < robomas_amount; i++)
            {
                constexpr float rmp_to_rad = 2 * M_PI / 60;
                float motor_dps = robomas.get_rpm(i + 1) * rmp_to_rad;
                const float percent = pid[i].calc(robomas_rpm[i], motor_dps, elapsed);
                // printf("dps: %f, goal: %d, out: %d\n", motor_dps, goal_ang_vel, out);

                robomas.set_output_percent(percent, i + 1);
            }
            robomas.write();

            CANMessage msg1(can_id, (const uint8_t *)&can_pwr1, 8);
            can1.write(msg1);
            pre = now;
        }
    }
}


bool readline(BufferedSerial &serial, char *buffer, const size_t size, const bool is_integar, const bool is_float)
{
    int i = 0;       // 繰り返し変数
    char buff = '0'; // シリアル受信

    if (not serial.readable())
    {
        return 1;
    }

    while ((buff != '\n') and i < (int)size)
    {
        serial.read(&buff, sizeof(buff)); // シリアル受信
        // printf("%c", buff);

        if (buff != '\n' && buff != '\r')
        {
            buffer[i] = buff; // 受信データ保存

            if (is_integar)
            {
                if (((buff < '0' || buff > '9') && buff != '-'))
                {
                    printf("error\n");
                    return 1;
                }
            }
            if (is_float)
            {
                if (((buff < '0' || buff > '9') && buff != '.' && buff != '-'))
                {
                    printf("error\n");
                    return 1;
                }
            }
        }
        i++;
    }
    // printf("\n");
    return 0;
}

float duration_to_sec(const std::chrono::duration<float> &duration)
{
    return duration.count();
}