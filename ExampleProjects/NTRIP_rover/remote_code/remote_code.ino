

// Define the pin numbers for the output pins


const int outputPins[] = {2, 3, 4, 5, 6, 7, 21};
const int inputPin = 8;
const int tonePin = 10;
byte pauseState = 0;
byte receivedByte = 0xff;
static unsigned long writePauseTimer = 0;
static unsigned long toneTimer = 0;

void setup()
{
    // Initialize serial communication at 115200 baud

    // Set output pins as OUTPUT
    for (int i = 0; i < 8; i++)
    {
        pinMode(outputPins[i], OUTPUT);
    }
    pinMode(inputPin, INPUT);
    pinMode(tonePin, OUTPUT);

    writePauseTimer = millis();
    toneTimer = millis();
    tone(tonePin, 5000, 500);
    
    Serial.begin(115200);
}

void loop()
{

    if (Serial.available() > 0)
    {
        //     Read a byte from the serial port
        receivedByte = Serial.read();

        // Parse the byte and set corresponding pins high
        for (int i = 0; i < 8; i++)
        {
            // Check if the ith bit of receivedByte is set
            if (receivedByte & (1 << i))
            {
                // If the bit is set, set the corresponding pin high
                digitalWrite(outputPins[i], HIGH);
            }
            else
            {
                // If the bit is not set, set the corresponding pin low
                digitalWrite(outputPins[i], LOW);
            }
        }
    }
    if ((millis() - writePauseTimer) > 500)
    {
        if (digitalRead(inputPin))
        {
            pauseState = 0xff;        }
        else
        {
            pauseState = 0;
        }
        Serial.write(pauseState);
        writePauseTimer = millis();
    }
    if ((millis() - toneTimer) > 1000)
    {
        if (digitalRead(inputPin))
        {
            pauseState = 0xff;
            tone(tonePin, 2500, 500);
                        
        }
        else
        {
            pauseState = 0;
            
        }
        Serial.write(pauseState);
        toneTimer = millis();
    }
}