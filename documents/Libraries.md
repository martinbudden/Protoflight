# Libraries used by Protoflight

| Library                | On github | tag    | release | platformio | Arduino |
| ---------------------- | --------- | ------ | ------- | ---------- | ------- |
| Filters                | yes       | 0.9.4  | 0.9.4   | yes        | no      |
| PIDF                   | yes       | 0.4.1  | 0.4.1   | yes        | no      |
| VectorQuaternionMatrix | yes       | 0.4.4  | 0.4.4   | yes        | no      |
| MotorMixers            | yes       | 0.0.2  | 0.0.2   | yes        | no      |
| SensorFusion           | yes       | 0.2.7  | 0.2.7   | yes        | no      |
| StreamBuf              | yes       | 0.0.2  | 0.0.2   | yes        | no      |
| IMU                    | yes       | 0.9.7  | 0.9.7   | yes        | no      |
| TaskBase               | yes       | 0.0.7  | 0.0.7   | yes        | no      |
| StabilizedVehicle      | yes       | 0.5.16 | 0.5.16  | yes        | no      |
| Receiver               | yes       | 0.5.10 | 0.5.10  | yes        | no      |
| Backchannel            | yes       | 0.1.8  | 0.1.8   | yes        | no      |
| MultiWiiSerialProtocol | yes       | 0.0.10 | 0.0.10  | yes        | no      |
| Blackbox               | yes       | 0.0.19 | 0.0.19  | yes        | no      |
| FlashKLV               | yes       | 0.0.3  | 0.0.3   | yes        | no      |

## Dependencies

```text
Libraries
├── Filters @ 0.9.4
├── PIDF @ 0.4.1
├── TaskBase @ 0.0.7
├── VectorQuaternionMatrix @ 0.4.4
├── FlashKLV @ 0.0.3
│
├── SensorFusion @ 0.2.7
│   └── VectorQuaternionMatrix @ 0.4.4
│
├── IMU @ 0.9.7
│   └── VectorQuaternionMatrix @ 0.4.3
│
├── MotorMixers @ 0.0.2
│   ├── VectorQuaternionMatrix @ 0.4.4
│   ├── Filters @ 0.9.4
│   └── PIDF @ 0.4.1
│
├── StabilizedVehicle @ 0.5.16
│   ├── TaskBase @ 0.0.7
│   ├── VectorQuaternionMatrix @ 0.4.3
│   ├── IMU @ 0.9.7
│   │   └── VectorQuaternionMatrix @ 0.4.3
│   └── SensorFusion @ 0.2.5
│       └── VectorQuaternionMatrix @ 0.4.3
│
├── Receiver @ 0.5.10
│   └── TaskBase @ 0.0.7
│
└── Backchannel @ 0.1.8
    ├── TaskBase @ 0.0.7
    ├── Receiver @ 0.5.10
    └── StabilizedVehicle @ 0.5.15


└── Protoflight @ 0.0.1
    ├── IMU @ 0.9.7
    ├── Filters @ 0.9.3
    ├── PIDF @ 0.3.4
    ├── MotorMixers @ 0.0.1
    ├── StabilizedVehicle @ 0.5.15
    ├── Receiver @ 0.5.10
    └── Backchannel @ 0.1.8

└── MultiWiiSerialProtocol @ 0.0.10
    ├── TaskBase @ 0.0.7
    └── StreamBuf @ 0.0.2

└── Blackbox @ 0.0.19
    ├── TaskBase @ 0.0.7
    └── StreamBuf @ 0.0.2
```

## Notes

To add a library to the Arduino Library manager, make a [pull request here](https://github.com/arduino/library-registry)

`pio pkg list` - checks has been published
To add a library to platformio, type `pio pkg publish`, once logged into platformio account
