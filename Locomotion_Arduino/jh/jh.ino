uint8_t buffer[4] = { 0 };
uint8_t data_counter = 0;
bool data_ready = false;

int8_t left_wheel_speed = 0;
int8_t right_wheel_speed = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (data_ready)
  {
      data_ready = false;

      if (buffer[0] == 0x59)  // Check whether the packet header is correct
      {
        /* Calculate sum of packet and verify checksum at buffer[10]
                  while (sum > 256) is for handling overflow
              */
        uint32_t sum = 0;

        int32_t buffer_data = 0;

        for (int x = 0; x < 3; x++)
          sum += buffer[x];

        // CheckSum Overflow
        sum %= 256;

        if (sum == buffer[3])
        {
          left_wheel_speed = buffer[1];
          right_wheel_speed = buffer[2];
        }
      }
}

void serialEvent()
{
  while (Serial.available())
  {
    buffer[data_counter] = (unsigned char)Serial.read();
    // The fist byte is not the packet header,
    // skip to wait till the packet header to arrive
    // Serial.println(buffer[0]);

    if (data_counter == 0 && buffer[0] != 0x59)
      return;
    data_counter++;
    if (data_counter == 4)  // All packet received, bool data_ready = true
    {
      data_counter = 0;
      data_ready = true;
    }
  }
}
