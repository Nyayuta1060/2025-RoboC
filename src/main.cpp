# include "mbed.h"

BufferedSerial pc(USBTX, USBRX, 115200);
CAN can1(PA_11, PA_12, 1e6);


bool readline(BufferedSerial &serial, char *buffer, size_t size, bool is_integar = false, bool is_float = false);

int main()
{
    while(1)
    {
        auto now = HighResClock::now();
        static auto pre = now;

        char received[15] = "";

        if(readline(pc, received, sizeof(received)) == 0) //UART受取成功時のスコープ
        {
            if(strcmp(received, " /*msg*/ "))
            {
                // msg受け取った際の処理
            }
        }
        if(now - pre > 10ms) // CAN送信など制御信号の送信を行うスコープ
        {
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

    while ((buff != '\n') and i < size)
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