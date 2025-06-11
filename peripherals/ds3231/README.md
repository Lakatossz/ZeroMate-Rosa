# DS3231 RTC

The rtc communicates over I2C. 

## Configuration

```json
{
  "peripherals": [
    {
      "name": "DS3231 RTC",
      "connection": [ 3, 2, 104 ],
      "comment": "connection: [SCL, SDA, I2C addr]",
      "lib_dir": "peripherals",
      "lib_name": "ds3231"
    }
  ]
}
```