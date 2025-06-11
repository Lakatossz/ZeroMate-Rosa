# PCA 9685 PWM Regulator and MG996R Servo Motor

The PWM Regulator communicates over I2C and drives the motor. 

## Configuration

```json
{
  "peripherals": [
    {
	  "name" : "MG996R Servo Motor",
	  "connection" : [ 3, 2, 20 ],
	  "comment" : "connection: [SCL, SDA, I2C addr]",
	  "lib_dir" : "peripherals",
	  "lib_name" : "pca9685_mg996r"
	}
  ]
}
```
