# Libraries used by Protoflight

| Library                | On github | tag    | release | platformio | Arduino |
| ---------------------- | --------- | ------ | ------- | ---------- | ------- |
| Filters                | yes       | 0.9.2  | 0.9.2   | yes        | no      |
| PIDF                   | yes       | 0.3.3  | 0.3.3   | yes        | no      |
| VectorQuaternionMatrix | yes       | 0.4.2  | 0.4.2   | yes        | no      |
| SensorFusion           | yes       | 0.2.4  | 0.2.4   | yes        | no      |
| StreamBuf              | yes       | 0.0.1  | 0.0.1   | yes        | no      |
| IMU                    | yes       | 0.9.5  | 0.9.5   | yes        | no      |
| TaskBase               | yes       | 0.0.7  | 0.0.7   | yes        | no      |
| StabilizedVehicle      | yes       | 0.5.7  | 0.5.7   | yes        | no      |
| Receiver               | yes       | 0.5.6  | 0.5.6   | yes        | no      |
| Backchannel            | yes       | 0.1.5  | 0.1.5   | yes        | no      |
| MultiWiiSerialProtocol | yes       | 0.0.8  | 0.0.8   | yes        | no      |
| Blackbox               | yes       | 0.0.13 | 0.0.13  | yes        | no      |
| FlashKLV               | yes       | 0.0.3  | 0.0.3   | yes        | no      |

## Dependencies

```text
Libraries
├── Filters @ 0.9.2
├── PIDF @ 0.3.3
├── TaskBase @ 0.0.7
├── VectorQuaternionMatrix @ 0.4.2
├── FlashKLV @ 0.0.3
│
├── IMU @ 0.9.5
│   └── VectorQuaternionMatrix @ 0.4.2
│
├── SensorFusion @ 0.2.4
│   └── VectorQuaternionMatrix @ 0.4.2
│
├── StabilizedVehicle @ 0.5.7
│   ├── TaskBase @ 0.0.7
│   ├── VectorQuaternionMatrix @ 0.4.2
│   ├── IMU @ 0.9.5
│   │   └── VectorQuaternionMatrix @ 0.4.2
│   └── SensorFusion @ 0.2.4
│       └── VectorQuaternionMatrix @ 0.4.2
│
├── Receiver @ 0.5.6
│   └── TaskBase @ 0.0.7
│
└── Backchannel @ 0.1.5
    ├── TaskBase @ 0.0.7
    ├── Receiver @ 0.5.6
    └── StabilizedVehicle @ 0.5.7


└── Protoflight @ 0.0.1
    ├── IMU @ 0.9.5
    ├── Filters @ 0.9.2
    ├── PIDF @ 0.3.3
    ├── StabilizedVehicle @ 0.5.6
    ├── Receiver @ 0.5.6
    └── Backchannel @ 0.1.4

└── MultiWiiSerialProtocol @ 0.0.8
    ├── TaskBase @ 0.0.7
    └── StreamBuf @ 0.0.1

└── Blackbox @ 0.0.13
    ├── TaskBase @ 0.0.7
    └── StreamBuf @ 0.0.1
```

## Notes

To add a library to the Arduino Library manager, make a pull request [here](https://github.com/arduino/library-registry)

`pio pkg list` - checks has been published
To add a library to platformio, type `pio pkg publish`, once logged into platformio account
