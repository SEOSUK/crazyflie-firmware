## Status
- 🧪 Currently on the `flyingpen_temp` branch for **code cleanup and structural refactoring**.
- 🔍 Comprehensive **validation and stress testing** of the hybrid **Complementary (Mahony) + Kalman Filter**–based tilted state estimation is in progress.
- ⏳ Further improvements are required for acceleration bias handling and slow attitude recovery.

## History
- Forked from upstream Bitcraze crazyflie-firmware on 2025-12-08
- FlyingPen branch under active development
- Milestone: Tilted state estimation achieved using a hybrid Complementary (Mahony) and Kalman Filter structure (commit `386d056`, 2026-01-10)
- Current phase: Code cleanup, refactoring, and systematic validation on `flyingpen_temp`



# Crazyflie Firmware  [![CI](https://github.com/bitcraze/crazyflie-firmware/workflows/CI/badge.svg)](https://github.com/bitcraze/crazyflie-firmware/actions?query=workflow%3ACI)

This project contains the source code for the firmware used in the Crazyflie range of platforms, including the Crazyflie 2.x and the Roadrunner.

### Crazyflie 1.0 support

The 2017.06 release was the last release with Crazyflie 1.0 support. If you want
to play with the Crazyflie 1.0 and modify the code, please clone this repo and
branch off from the 2017.06 tag.

## Building and Flashing
See the [building and flashing instructions](https://github.com/bitcraze/crazyflie-firmware/blob/master/docs/building-and-flashing/build.md) in the github docs folder.


## Official Documentation

Check out the [Bitcraze crazyflie-firmware documentation](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/) on our website.

## Generated documentation

The easiest way to generate the API documentation is to use the [toolbelt](https://github.com/bitcraze/toolbelt)

```tb build-docs```

and to view it in a web page

```tb docs```

## Contribute
Go to the [contribute page](https://www.bitcraze.io/contribute/) on our website to learn more.

### Test code for contribution

To run the tests please have a look at the [unit test documentation](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/development/unit_testing/).

## License

The code is licensed under LGPL-3.0
