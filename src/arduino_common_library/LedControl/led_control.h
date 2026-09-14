#ifndef LED_CONTROL_H_
#define LED_CONTROL_H_

#include <Adafruit_NeoPixel.h>

#define RED     strip.Color(255,   0,   0) // Red
#define GREEN   strip.Color(  0, 255,   0) // Green
#define BLUE    strip.Color(  0,   0, 255) // Blue
#define WHITE   strip.Color(127, 127, 127)
#define YELLOW  strip.Color(255, 255, 0  )
#define ORANGE  strip.Color(255, 127, 0  )
#define VIOLET  strip.Color(160, 50 , 168)
#define BLACK   strip.Color(0  , 0  , 0  )

void blinkLed(int duration, int blink_interval, int r, int g, int b,int num_of_pin=0, int LedPins[]= {});
bool myColorWipe(uint32_t color, int wait, int num_of_pin=0, int LedPins[]={});
bool myColorIndex(uint32_t color, int wait, int startLED, int endLED, int invert, int invLength);
bool myRainbow(int wait, int num_of_pin=0, int LedPins[]={});
bool offAllLed(int wait, int num_of_pin=0, int LedPins[]={});

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
bool NewLedEffectRequest = false;
bool NewLedEffectDone = false;

int LED_INDEX = 0;

void blinkLed(int duration, int blink_interval, int r, int g, int b, int num_of_pin=0, int LedPins[]= {})
{
    uint32_t t = millis();
    static uint32_t t_start_blink = t;
    static int blink_state = 0;

    if (NewLedEffectRequest)
    {
        blink_state = 0;
    }

    if (blink_state == 0)
    {
        if (myColorWipe(strip.Color(r, g, b), duration, num_of_pin, LedPins))
        {
            blink_state = 1;
            t_start_blink = t;
        }
    }
    else if (blink_state == 1)
    {
        if (t - t_start_blink >= blink_interval)
        {
            blink_state = 2;
            t_start_blink = t;
        }
    }
    if (blink_state == 2)
    {
        if (myColorWipe(strip.Color(0, 0, 0), duration, num_of_pin, LedPins))
        {
            blink_state = 3;
            t_start_blink = t;
        }
    }
    else if (blink_state == 3)
    {
        if (t - t_start_blink >= blink_interval)
        {
            blink_state = 0;
            t_start_blink = t;
        }
    }
}

// num_of_pin :  number of Led pin that you want to ouput data
// LedPin[]: Array Led pin number on arduino that you want to ouput
bool myColorWipe(uint32_t color, int wait, int num_of_pin=0, int LedPins[]= {} )
{
    static uint32_t t = millis();
    static int i = 0;

    if (NewLedEffectRequest)
    {
        NewLedEffectRequest = false;
        i = 0;
    }
    if (wait == 0)
    {
        for (int j = 0; j < strip.numPixels(); j++)
        {
            strip.setPixelColor(j, color); //  Set pixel's color (in RAM)
        }
        if (num_of_pin == 0)
        {
            strip.setPin(LED_PIN);
            strip.show();
        }
        else
        {
            for ( int count = 0; count < num_of_pin; count++)
            {
                strip.setPin(LedPins[count]);
                strip.show();
            }
            strip.setPin(LED_PIN); // return control LED_PIN for other function

        }

        return true;
    }
    else if (i <= strip.numPixels())
    {
        if (millis() - t >= wait)
        {
            strip.setPixelColor(i, color); //  Set pixel's color (in RAM)
            if (num_of_pin == 0)
            {
                strip.setPin(LED_PIN);
                strip.show();
            }
            else
            {
                for ( int count = 0; count < num_of_pin; count++)
                {
                    strip.setPin(LedPins[count]);
                    strip.show();
                }
                strip.setPin(LED_PIN); // return control LED_PIN for other function
            }
            if (i++ == strip.numPixels())
            {
                i = 0;
                return true;
            }
            t = millis();
        }
    }
    return false;
}

// Select start and end of LED num

bool myColorIndex(uint32_t color, int wait, int startLED, int endLED, int offsetLED, int invert,int invLength)
{
    static uint32_t t = millis();
    static int i = startLED;

    if (NewLedEffectRequest)
    {
        NewLedEffectRequest = false;
        i = startLED;
    }
    if ((i >= startLED) && (i <= (endLED + 1))) // wait display and delay for endLED finish
    {
        if (millis() > t + wait)
        {
            if (i > endLED)
            {
                i = startLED;
                return true;
            }
            switch (invert)  // select led follow type invert
            {
            case 1:
                strip.setPixelColor((2 * startLED) + invLength - i - 1, color);
                strip.setPixelColor(i + offsetLED, color);
                break;

            case 2:
                strip.setPixelColor((2 * startLED) + invLength + i - 1, color);
                strip.setPixelColor(offsetLED - i, color);
                break;

            default:
                strip.setPixelColor(i, color);
                break;
            }

            strip.show();
            i++;
            t = millis();
            }
            // LED_INDEX = i;
        }
        return false;
}

// Off All Led in time
bool offAllLed(int wait,  int num_of_pin=0, int LedPins[]={})
{
    static uint32_t t;
    static bool OFF_DONE = false;
    strip.clear(); //off all led
    if (num_of_pin == 0)
    {
        strip.setPin(LED_PIN);
        strip.show();
    }
    else
    {
        for (int count = 0; count < num_of_pin; count++)
        {
            strip.setPin(LedPins[count]);
            strip.show();
        }
        strip.setPin(LED_PIN); // return control LED_PIN for other function
    }
    if (!OFF_DONE) // when not off led
    {
        OFF_DONE = true;
        t = millis();
    }
    else // when off led done
    {
        if (millis() >= t + wait)
        {
            OFF_DONE = false; // Wait finish
            return true;
        }
    }
    return false;
}

bool myRainbow(int wait, int num_of_pin=0, int LedPins[]= {})
{
    static uint32_t t = millis();
    static int i = 0;
    static long firstPixelHue = 0;

    if (NewLedEffectRequest)
    {
        NewLedEffectRequest = false;
        i = 0;
    }

    if (firstPixelHue < 5 * 65536)
    {
        for (int i = 0; i < strip.numPixels(); i++)
        {
            int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
            strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
        }
        if (num_of_pin == 0)
            {
                strip.setPin(LED_PIN);
                strip.show();
            }
        else
            {
                for ( int count = 0; count < num_of_pin; count++)
                {
                    strip.setPin(LedPins[count]);
                    strip.show();
                }
                strip.setPin(LED_PIN); // return control LED_PIN for other function
            }
        if (millis() - t >= wait)
        {
            firstPixelHue += 256;
            t = millis();
        }
    }
    else
    {
        firstPixelHue = 0;
        return true;
    }
    return false;
}

#endif