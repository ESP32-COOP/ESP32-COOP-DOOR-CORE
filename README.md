# [ESP32-COOP-DOOR](https://coop-door.vercel.app/)
Coop Door made for a Esp32 (lolin32 lite)



Apologies for the confusion. Here's an updated version of the README section with the truth table included:


## Time Threshold for Opening and Closing the Door

The following section explains how the time threshold is checked to determine whether the door should be open or closed based on the current time.

### Opening and Closing Conditions

The door opening and closing conditions are as follows:

```plaintext
Opening Condition: (TimeH > doorOpenTimeH or (TimeH == doorOpenTimeH and TimeM > doorOpenTimeM))
Closing Condition: (TimeH > doorCloseTimeH or (TimeH == doorCloseTimeH and TimeM > doorCloseTimeM))
```

- `TimeH` represents the current hour.
- `TimeM` represents the current minute.
- `doorOpenTimeH` represents the hour for the door to open.
- `doorOpenTimeM` represents the minute for the door to open.
- `doorCloseTimeH` represents the hour for the door to close.
- `doorCloseTimeM` represents the minute for the door to close.

### Truth Table

The following truth table illustrates the outcome (`door_open`) based on different time scenarios:

| TimeH | TimeM | doorOpenTimeH | doorOpenTimeM | doorCloseTimeH | doorCloseTimeM | door_open |
|-------|-------|---------------|---------------|----------------|----------------|-----------|
|   9   |   0   |       8       |       30      |        17      |       30       |   true    |
|  12   |  30   |       8       |       30      |        17      |       30       |   false   |
|  18   |  45   |       8       |       30      |        17      |       30       |   false   |
|  20   |  15   |       8       |       30      |        17      |       30       |   true    |
```
