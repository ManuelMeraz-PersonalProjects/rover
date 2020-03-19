#include <wiringPi.h>

int main()
{
    wiringPiSetup();
    pinMode(0, OUTPUT);

    for (;;)
    {
        digitalWrite(0, HIGH);
        delay(1000);
        digitalWrite(0, LOW);
        delay(1000);
    }
    return 0;
}


