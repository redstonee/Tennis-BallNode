#include <Arduino.h>

void setup()
{
    // put your setup code here, to run once:
    pinMode(PB1, OUTPUT);
}

void loop()
{
    // put your main code here, to run repeatedly:
    digitalWrite(PB1, HIGH);
    delay(1000);
    digitalWrite(PB1, LOW);
    delay(1000);
    
}