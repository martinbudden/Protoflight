# Libraries used by Protoflight

| Library                | On github | release | platformio | Arduino | Releases |
| ---------------------- | --------- | ------- | ---------- | ------- | -------- |
| Filter                 | yes       | 0.0.1   | yes        | no      |  1       |
| PID                    | yes       | 0.0.1   | yes        | no      |  1       |
| VectorQuaternionMatrix | yes       | 0.4.10  | yes        | no      | 17       |
| MotorMixers            | yes       | 0.0.7   | yes        | no      |  6       |
| SensorFusion           | yes       | 0.2.14  | yes        | no      | 20       |
| StreamBuf              | yes       | 0.0.4   | yes        | no      |  4       |
| Sensors                | yes       | 0.0.6   | yes        | no      |  6       |
| TaskUtilities          | yes       | 0.0.1   | yes        | no      |  1       |
| StabilizedVehicle      | yes       | 0.5.25  | yes        | no      | 48       |
| Receiver               | yes       | 0.0.1   | yes        | no      |  1       |
| Backchannel            | yes       | 0.1.14  | yes        | no      | 25       |
| MultiWiiSerialProtocol | yes       | 0.0.17  | yes        | no      | 17       |
| Blackbox               | yes       | 0.0.25  | yes        | no      | 25       |
| FlashKLV               | yes       | 0.0.3   | yes        | no      |  3       |
|                        |           |         |            |         |          |
| IMU                    | yes       | 0.9.8  - superseded | yes        | no      |          |
| TaskBase               | yes       | 0.0.13 - superseded | yes        | no      | 13       |
| Receiver               | yes       | 0.5.14 - superseded | yes        | no      | 33       |
| PIDF                   | yes       | 0.4.2  - superseded | yes        | no      | 15       |
| Filters                | yes       | 0.9.4  - superseded | yes        | no      | 19       |

## Dependencies

```text
Libraries
├── Filter @ 0.0.1
├── PIDF @ 0.4.2
├── TaskUtilities @ 0.0.1
├── VectorQuaternionMatrix @ 0.4.10
├── FlashKLV @ 0.0.3
│
├── SensorFusion @ 0.2.14
│   └── VectorQuaternionMatrix @ 0.4.10
│
├── Sensors @ 0.0.6
│   └── VectorQuaternionMatrix @ 0.4.10
│
├── MotorMixers @ 0.0.6
│   ├── VectorQuaternionMatrix @ 0.4.7
│   ├── Filters @ 0.9.4
│   └── PIDF @ 0.4.2
│
├── StabilizedVehicle @ 0.5.25
│   ├── TasUtilities @ 0.0.1
│   ├── VectorQuaternionMatrix @ 0.4.10
│   ├── Sensors @ 0.0.6
│   │   └── VectorQuaternionMatrix @ 0.4.10
│   └── SensorFusion @ 0.2.14
│       └── VectorQuaternionMatrix @ 0.4.10
│
├── Receiver @ 0.0.1
│   └── TaskUtilities @ 0.0.1
│
└── Backchannel @ 0.1.14
    ├── TaskUtilities @ 0.0.1
    ├── Receivers @ 0.0.1
    └── StabilizedVehicle @ 0.5.25

└── MultiWiiSerialProtocol @ 0.0.17
    ├── TaskUtilities @ 0.0.1
    └── StreamBuf @ 0.0.4

└── Blackbox @ 0.0.25
    ├── TaskUtilities @ 0.0.1
    └── StreamBuf @ 0.0.4

└── Protoflight @ 0.0.1
    ├── Sensors @ 0.0.6
    ├── Filter @ 0.0.1
    ├── PIDF @ 0.4.2
    ├── MotorMixers @ 0.0.7
    ├── StabilizedVehicle @ 0.5.25
    ├── Receivers @ 0.0.1
    ├── MultiWiiSerialProtocol @ 0.0.16
    ├── Blackbox @ 0.0.25
    └── Backchannel @ 0.1.14


└── IMU @ 0.9.8 - superseded
    └── VectorQuaternionMatrix @ 0.4.5

└── TaskBase @ 0.0.13 - superseded

└── Receiver @ 0.5.14 - superseded
    └── TaskBase @ 0.0.13

```

## Notes

To add a library to the Arduino Library manager, make a [pull request here](https://github.com/arduino/library-registry)

`pio pkg list` - checks has been published
To add a library to platformio, type `pio pkg publish`, once logged into platformio account

## Library dependencies before changing to use Library-Sensors

```text
lib_deps =
    martinbudden/Filters@^0.9.4
    martinbudden/PIDF@^0.4.1
    martinbudden/VectorQuaternionMatrix@^0.4.5
    martinbudden/SensorFusion@^0.2.8
    martinbudden/IMU@^0.9.8
    martinbudden/TaskBase@^0.0.10
    martinbudden/StabilizedVehicle@^0.5.18
    martinbudden/Receiver@^0.5.11
    martinbudden/Backchannel@^0.1.9
    martinbudden/StreamBuf@^0.0.2
    martinbudden/MultiWiiSerialProtocol@^0.0.13
    martinbudden/Blackbox@^0.0.22
    martinbudden/MotorMixers@^0.0.3
```
