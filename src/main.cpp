#include "mbed.h"
#include "C620.hpp"
#include "PID_new.hpp"
#include <map>

BufferedSerial pc(USBTX, USBRX, 115200);
CAN can1(PA_11, PA_12, 1e6);
constexpr int can_id[2] = {0};
CANMessage msg1;
CANMessage msg2;

bool readline(BufferedSerial &serial, char *buffer, size_t size, bool is_integar = false, bool is_float = false);

enum class state
{
    FRONT,
    STOP,
    BACK
};

constexpr int CROW_SPEED = 15000;
constexpr int PYLON_SPEED = 10000;

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

int main()
{
    auto zozo_crow = state::STOP;
    auto pylon_rack = state::STOP;

    int16_t can_pwr1[4] = {0};
    int16_t can_pwr2[4] = {0};

    while(1)
    {
        auto now = HighResClock::now();
        static auto pre = now;

        char received[15] = "";
        int pillar_push = 0;


        if(readline(pc, received, sizeof(received)) == 0) //UART受取成功時のスコープ
        {
            if(strcmp(received, "front_crow") == 0){
                zozo_crow = state::FRONT;
            }
            else if(strcmp(received, "back_crow") == 0){
                zozo_crow = state::BACK;
            }
            else if(strcmp(received, "stop_crow") == 0){
                zozo_crow = state::STOP;
            }

            if (strcmp(received, "R2") == 0)
            {
                char pwr[8] = "";
                if(readline(pc, pwr, sizeof(pwr), true, false) == 0){
                    pillar_push = atoi(pwr);
                }
            }
            else if (strcmp(received, "L2") == 0)
            {
                char pwr[8] = "";
                if(readline(pc, pwr, sizeof(pwr), true, false) == 0){
                    pillar_push = atoi(pwr) * -1;
                }
                
            }
            if(strcmp(received, "go_pylon") == 0){
                pylon_rack = state::FRONT;
            }
            else if(strcmp(received, "back_pylon") == 0){
                pylon_rack = state::BACK;
            }
            else if(strcmp(received, "stop_pylon") == 0){
                pylon_rack = state::STOP;
            }
        }

        can_pwr1[0] = CROW_SPEED_MAP.at(zozo_crow);
        can_pwr1[1] = pillar_push;
        can_pwr2[0] = PYLON_SPEED_MAP.at(pylon_rack);

        if(now - pre > 10ms) // CAN送信など制御信号の送信を行うスコープ
        {
            pre = now;

            CANMessage msg1(can_id[0], (const uint8_t *)&can_pwr1, 8);
            CANMessage msg2(can_id[1], (const uint8_t *)&can_pwr2, 8);
            can1.write(msg1);
            can1.write(msg2);
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
