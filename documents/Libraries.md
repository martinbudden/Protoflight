# Libraries used by Protoflight

| Library                | On github | tag    | release | platformio | Arduino |
| ---------------------- | --------- | ------ | ------- | ---------- | ------- |
| Filters                | yes       | 0.9.4  | 0.9.4   | yes        | no      |
| PIDF                   | yes       | 0.4.1  | 0.4.1   | yes        | no      |
| VectorQuaternionMatrix | yes       | 0.4.5  | 0.4.5   | yes        | no      |
| MotorMixers            | yes       | 0.0.3  | 0.0.3   | yes        | no      |
| SensorFusion           | yes       | 0.2.8  | 0.2.8   | yes        | no      |
| StreamBuf              | yes       | 0.0.2  | 0.0.2   | yes        | no      |
| IMU                    | yes       | 0.9.8  | 0.9.8   | yes        | no      |
| TaskBase               | yes       | 0.0.10 | 0.0.10  | yes        | no      |
| StabilizedVehicle      | yes       | 0.5.17 | 0.5.17  | yes        | no      |
| Receiver               | yes       | 0.5.11 | 0.5.11  | yes        | no      |
| Backchannel            | yes       | 0.1.8  | 0.1.8   | yes        | no      |
| MultiWiiSerialProtocol | yes       | 0.0.12 | 0.0.12  | yes        | no      |
| Blackbox               | yes       | 0.0.21 | 0.0.21  | yes        | no      |
| FlashKLV               | yes       | 0.0.3  | 0.0.3   | yes        | no      |

## Dependencies

```text
Libraries
├── Filters @ 0.9.4
├── PIDF @ 0.4.1
├── TaskBase @ 0.0.10
├── VectorQuaternionMatrix @ 0.4.5
├── FlashKLV @ 0.0.3
│
├── SensorFusion @ 0.2.8
│   └── VectorQuaternionMatrix @ 0.4.5
│
├── IMU @ 0.9.8
│   └── VectorQuaternionMatrix @ 0.4.5
│
├── MotorMixers @ 0.0.3
│   ├── VectorQuaternionMatrix @ 0.4.5
│   ├── Filters @ 0.9.4
│   └── PIDF @ 0.4.1
│
├── StabilizedVehicle @ 0.5.17
│   ├── TaskBase @ 0.0.9
│   ├── VectorQuaternionMatrix @ 0.4.5
│   ├── IMU @ 0.9.8
│   │   └── VectorQuaternionMatrix @ 0.4.5
│   └── SensorFusion @ 0.2.8
│       └── VectorQuaternionMatrix @ 0.4.5
│
├── Receiver @ 0.5.11
│   └── TaskBase @ 0.0.10
│
└── Backchannel @ 0.1.8
    ├── TaskBase @ 0.0.7
    ├── Receiver @ 0.5.10
    └── StabilizedVehicle @ 0.5.15


└── Protoflight @ 0.0.1
    ├── IMU @ 0.9.7
    ├── Filters @ 0.9.4
    ├── PIDF @ 0.4.1
    ├── MotorMixers @ 0.0.2
    ├── StabilizedVehicle @ 0.5.16
    ├── Receiver @ 0.5.10
    └── Backchannel @ 0.1.8

└── MultiWiiSerialProtocol @ 0.0.12
    ├── TaskBase @ 0.0.9
    └── StreamBuf @ 0.0.2

└── Blackbox @ 0.0.21
    ├── TaskBase @ 0.0.9
    └── StreamBuf @ 0.0.2
```

## Notes

To add a library to the Arduino Library manager, make a [pull request here](https://github.com/arduino/library-registry)

`pio pkg list` - checks has been published
To add a library to platformio, type `pio pkg publish`, once logged into platformio account
