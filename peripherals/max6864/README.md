# MAX6864 WDT

The wdt communicates over GPIO. 

## Configuration

```json
{
  "peripherals": [
    {
      "name" : "MAX6864 WDT",
      "connection" : [ 7, 8, 9 ],
      "lib_dir" : "peripherals",
      "lib_name" : "max6864"
    }
  ]
}
```