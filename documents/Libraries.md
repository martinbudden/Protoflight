# Libraries used by Protoflight

| Library                | On github | release | platformio | Arduino |
| ---------------------- | --------- | ------- | ---------- | ------- |
| Filters                | yes       | 0.9.4   | yes        | no      |
| PIDF                   | yes       | 0.4.1   | yes        | no      |
| VectorQuaternionMatrix | yes       | 0.4.6   | yes        | no      |
| MotorMixers            | yes       | 0.0.5   | yes        | no      |
| SensorFusion           | yes       | 0.2.10  | yes        | no      |
| StreamBuf              | yes       | 0.0.4   | yes        | no      |
| Sensors                | yes       | 0.0.4   | yes        | no      |
| TaskBase               | yes       | 0.0.12  | yes        | no      |
| StabilizedVehicle      | yes       | 0.5.23  | yes        | no      |
| Receiver               | yes       | 0.5.13  | yes        | no      |
| Backchannel            | yes       | 0.1.12  | yes        | no      |
| MultiWiiSerialProtocol | yes       | 0.0.14  | yes        | no      |
| Blackbox               | yes       | 0.0.23  | yes        | no      |
| FlashKLV               | yes       | 0.0.3   | yes        | no      |
| IMU                    | yes       | 0.9.8 - superseded   | yes        | no      |

## Dependencies

```text
Libraries
├── Filters @ 0.9.4
├── PIDF @ 0.4.1
├── TaskBase @ 0.0.12
├── VectorQuaternionMatrix @ 0.4.6
├── FlashKLV @ 0.0.3
│
├── SensorFusion @ 0.2.10
│   └── VectorQuaternionMatrix @ 0.4.6
│
├── Sensors @ 0.0.4
│   └── VectorQuaternionMatrix @ 0.4.6
│
├── MotorMixers @ 0.0.5
│   ├── VectorQuaternionMatrix @ 0.4.6
│   ├── Filters @ 0.9.4
│   └── PIDF @ 0.4.1
│
├── StabilizedVehicle @ 0.5.23
│   ├── TaskBase @ 0.0.12
│   ├── VectorQuaternionMatrix @ 0.4.6
│   ├── Sensors @ 0.0.4
│   │   └── VectorQuaternionMatrix @ 0.4.6
│   └── SensorFusion @ 0.2.10
│       └── VectorQuaternionMatrix @ 0.4.6
│
├── Receiver @ 0.5.13
│   └── TaskBase @ 0.0.12
│
└── Backchannel @ 0.1.12
    ├── TaskBase @ 0.0.12
    ├── Receiver @ 0.5.13
    └── StabilizedVehicle @ 0.5.22

└── MultiWiiSerialProtocol @ 0.0.15
    ├── TaskBase @ 0.0.12
    └── StreamBuf @ 0.0.3

└── Blackbox @ 0.0.23
    ├── TaskBase @ 0.0.12
    └── StreamBuf @ 0.0.3

└── Protoflight @ 0.0.1
    ├── Sensors @ 0.0.4
    ├── Filters @ 0.9.4
    ├── PIDF @ 0.4.1
    ├── MotorMixers @ 0.0.2
    ├── StabilizedVehicle @ 0.5.21
    ├── Receiver @ 0.5.12
    ├── MultiWiiSerialProtocol @ 0.0.15
    ├── Blackbox @ 0.0.23
    └── Backchannel @ 0.1.11


└── IMU @ 0.9.8 - superseded
    └── VectorQuaternionMatrix @ 0.4.5

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
