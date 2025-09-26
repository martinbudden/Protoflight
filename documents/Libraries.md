# Libraries used by Protoflight

| Library                | On github | tag    | release | platformio | Arduino |
| ---------------------- | --------- | ------ | ------- | ---------- | ------- |
| Filters                | yes       | 0.9.2  | 0.9.2   | yes        | no      |
| PIDF                   | yes       | 0.3.4  | 0.3.4   | yes        | no      |
| VectorQuaternionMatrix | yes       | 0.4.3  | 0.4.3   | yes        | no      |
| SensorFusion           | yes       | 0.2.5  | 0.2.5   | yes        | no      |
| StreamBuf              | yes       | 0.0.1  | 0.0.1   | yes        | no      |
| IMU                    | yes       | 0.9.7  | 0.9.7   | yes        | no      |
| TaskBase               | yes       | 0.0.7  | 0.0.7   | yes        | no      |
| StabilizedVehicle      | yes       | 0.5.10 | 0.5.10  | yes        | no      |
| Receiver               | yes       | 0.5.7  | 0.5.7   | yes        | no      |
| Backchannel            | yes       | 0.1.6  | 0.1.6   | yes        | no      |
| MultiWiiSerialProtocol | yes       | 0.0.8  | 0.0.8   | yes        | no      |
| Blackbox               | yes       | 0.0.14 | 0.0.14  | yes        | no      |
| FlashKLV               | yes       | 0.0.3  | 0.0.3   | yes        | no      |

## Dependencies

```text
Libraries
├── Filters @ 0.9.2
├── PIDF @ 0.3.4
├── TaskBase @ 0.0.7
├── VectorQuaternionMatrix @ 0.4.3
├── FlashKLV @ 0.0.3
│
├── IMU @ 0.9.7
│   └── VectorQuaternionMatrix @ 0.4.3
│
├── SensorFusion @ 0.2.5
│   └── VectorQuaternionMatrix @ 0.4.3
│
├── StabilizedVehicle @ 0.5.10
│   ├── TaskBase @ 0.0.7
│   ├── VectorQuaternionMatrix @ 0.4.3
│   ├── IMU @ 0.9.7
│   │   └── VectorQuaternionMatrix @ 0.4.3
│   └── SensorFusion @ 0.2.5
│       └── VectorQuaternionMatrix @ 0.4.3
│
├── Receiver @ 0.5.7
│   └── TaskBase @ 0.0.7
│
└── Backchannel @ 0.1.6
    ├── TaskBase @ 0.0.7
    ├── Receiver @ 0.5.7
    └── StabilizedVehicle @ 0.5.9


└── Protoflight @ 0.0.1
    ├── IMU @ 0.9.7
    ├── Filters @ 0.9.2
    ├── PIDF @ 0.3.4
    ├── StabilizedVehicle @ 0.5.10
    ├── Receiver @ 0.5.7
    └── Backchannel @ 0.1.6

└── MultiWiiSerialProtocol @ 0.0.8
    ├── TaskBase @ 0.0.7
    └── StreamBuf @ 0.0.1

└── Blackbox @ 0.0.14
    ├── TaskBase @ 0.0.7
    └── StreamBuf @ 0.0.1
```

## Notes

To add a library to the Arduino Library manager, make a pull request [here](https://github.com/arduino/library-registry)

`pio pkg list` - checks has been published
To add a library to platformio, type `pio pkg publish`, once logged into platformio account
