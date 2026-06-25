- # AGENTS.md — NUEDC H Smart Car

  ## Project identity

  This repository contains bare-metal firmware for the NUEDC H-problem smart car.

  Current real hardware target:

  - MCU: **STM32F103C8T6**
  - Toolchain: **Keil MDK / uVision**
  - Compiler: **ARM Compiler 6.24 (ARMCLANG)**
  - Firmware library: **STM32F10x Standard Peripheral Library V3.5.0**
  - Architecture: **bare-metal, no RTOS**
  - Do **not** migrate to HAL, LL, STM32CubeMX, CMSIS-only, FreeRTOS, CMake, Makefile, or another MCU.

  Important: although some reference documents may mention STM32G474 or TI/MSPM0, the current working car firmware in this repository is for **STM32F103C8T6**.

  ## Hard constraints for agents

  - Do not rewrite the whole project.
  - Do not replace the existing task state machines.
  - Prefer small, reviewable patches.
  - Preserve task1/task2/task3 behavior unless the user explicitly asks to change it.
  - Do not introduce dynamic allocation.
  - Do not add large libraries.
  - Do not use language/library features that may break ARM Compiler 6.24 / ARMCLANG in this bare-metal SPL project.
  - Do not assume command-line builds work. The project is built through Keil.
  - Do not claim the project builds unless a Keil build log is provided.
  - If adding new source files, update both:
    - `.gitignore` allow-list
    - Keil project file `Project.uvprojx`, if the file must be compiled

  ## Build and project files

  - Keil project: `Project.uvprojx`
  - Objects: `Objects/` not tracked
  - Listings: `Listings/` not tracked
  - Vendor library: `Library/`, read-only unless explicitly requested

  This repository uses an allow-list `.gitignore` pattern. Files are ignored by default. New tracked code must be explicitly unignored.

  Current tracked source areas:

  ```gitignore
  !Project.uvprojx
  !Start/**
  !Library/**
  !Hardware/**
  !System/**
  !Control/**
  !App/**
  !User/main.c
  !User/stm32f10x_conf.h
  !User/stm32f10x_it.c
  !User/stm32f10x_it.h
  !.gitignore
  ```

  ## Directory ownership

  | Directory   | Purpose                                                      |
  | ----------- | ------------------------------------------------------------ |
  | `User/`     | Main firmware entry point, global state, task state machines, local keys, main loop |
  | `Hardware/` | Low-level drivers: Motor, Encoder, OLED, MPU6050, Grayscale, PWM, Serial, Key |
  | `Control/`  | Generic PID code                                             |
  | `App/`      | Application-level glue: protocol parser, motor control glue, line tracking, state interface |
  | `System/`   | Delay and timer setup                                        |
  | `Library/`  | STM32F10x StdPeriphLib vendor code; avoid editing            |

  ## Main architecture

  The firmware is tightly coupled through `volatile` global variables. Many globals are declared in `User/main.c` and accessed through `extern` declarations from `App/app_protocol.c` and other app modules.

  Main loop responsibilities:

  1. `App_Protocol_Process()`
  2. `Main_KeyProcess()`
  3. MPU yaw update scheduling
  4. OLED refresh scheduling
  5. serial plot/status output scheduling

  TIM1 1ms interrupt responsibilities:

  1. key debounce tick
  2. prompt/beep/LED timing
  3. millisecond counters
  4. 100Hz control loop through `Control_Run10ms()`

  Control loop priority:

  1. task2/task3 special states
  2. straight yaw-hold mode
  3. standalone arc mode
  4. line tracing mode
  5. standby/BT timeout handling
  6. `App_Control_ApplyMotorOutput()`

  ## Current task goals

  The project is for the NUEDC H-problem automatic driving car.

  Required task paths:

  ```text
  task1: A -> B
  task2: A -> B -> C -> D -> A
  task3: A -> C -> B -> D -> A
  task4: repeat task3 path for multiple laps
  ```

  Current control strategy:

  ```text
  Straight: MPU yaw closed loop
  Arc entry: MPU yaw hard entry + grayscale line search
  Half-circle arc: 8-channel grayscale line tracing
  Arc exit: minimum arc distance + line-lost confirmation + yaw window
  Angle correction: MPU yaw alignment with timeout / wheel-diff protection
  task3 diagonal: geometric target yaw
  ```

  Do not replace this with a camera-based solution, map-based navigation, DMP fusion, full attitude solver, or a completely new state machine unless explicitly requested.

  ## Important hardware mapping

  ### Motor driver TB6612

  ```text
  PWMA = PA0
  PWMB = PA1
  AIN1 = PB12
  AIN2 = PB13
  BIN1 = PB14
  BIN2 = PB15
  LEFT_MOTOR_DIR_SIGN  = -1
  RIGHT_MOTOR_DIR_SIGN = +1
  ```

  ### Encoders

  ```text
  Left encoder E1:  PA6 / PA7 -> TIM3_CH1 / TIM3_CH2
  Right encoder E2: PB6 / PB7 -> TIM4_CH1 / TIM4_CH2
  LEFT_ENCODER_SIGN  = -1
  RIGHT_ENCODER_SIGN = +1
  ```

  ### 8-channel grayscale sensor through CD4051

  ```text
  AD0 = PA8
  AD1 = PB3
  AD2 = PB4
  OUT = PB0
  ```

  ### OLED and MPU6050 shared software I2C

  ```text
  PB8 = SCL
  PB9 = SDA
  ```

  ### MPU6050

  ```text
  Module: GY-521
  AD0 connected to GND
  WHO_AM_I may return 0x68 or 0x70
  Board flat, chip text upward, pin header toward car front
  +X points to car rear
  +Y points to car right
  Yaw uses GyroZ
  Counter-clockwise is positive
  g_mpuYawSign = 1 by default
  ```

  ### Prompt IO

  ```text
  PB1 = BEEP, low-level active
  PB5 = LED_EXT, high-level active
  Do not use PC13 for prompt output
  ```

  ### Physical keys

  ```text
  K1 / SW1 -> PB10
  K2 / SW2 -> PB11
  K3 / SW3 -> PA11
  K4 / SW4 -> PA12
  ```

  Current intended key behavior:

  | Key  | Intended behavior                                            |
  | ---- | ------------------------------------------------------------ |
  | K1   | Cycle local mode: Standby -> MPU Debug -> Standby         |
  | K2   | Select task: task1 -> task2 -> task3 -> task4                |
  | K3   | Start selected task in standby; zero yaw in MPU debug      |
  | K4   | In command-wait state: enter MPU debug and perform one-key MPU calibration flow; otherwise unlock/stop/return safe |

  K4 must not directly make the car turn 90 degrees.

  ## MPU yaw model

  This project does not use a full attitude solver.

  Do not introduce DMP, quaternion fusion, Mahony, Madgwick, accelerometer yaw correction, or magnetometer logic unless explicitly requested.

  Current yaw model:

  ```text
  GyroZ_raw
  -> GyroZ_dps
  -> optional 1D Kalman filter
  -> static bias tracking
  -> subtract bias
  -> multiply gyroZScale
  -> multiply yawSign
  -> deadband
  -> integrate yaw
  ```

  Formula:

  ```text
  gyroFiltered = KF(gyroRaw)
  gyroCorrected = (gyroFiltered - biasZ) * gyroZScale * yawSign
  yaw = yaw + gyroCorrected * dt
  ```

  Default MPU parameters should remain:

  ```c
  volatile float g_gyroZScale = 1.0f;
  volatile uint8_t g_gyroZKalmanEnable = 1U;
  volatile float g_gyroZKalmanQ = 0.02f;
  volatile float g_gyroZKalmanR = 1.5f;
  volatile float g_gyroZKalmanP = 1.0f;
  volatile float g_gyroZKalmanX = 0.0f;
  volatile uint8_t g_staticBiasTrackEnable = 1U;
  volatile float g_staticBiasAlpha = 0.999f;
  volatile float g_gyroZDeadbandDps = 0.03f;
  ```

  Recommended manual MPU debug flow:

  ```text
  [key,taskStop,down]
  [key,mpuDebug,down]
  [slider,gyroZScale,1.000]
  [slider,gyroZKalmanEnable,1]
  [slider,gyroZKalmanQ,0.02]
  [slider,gyroZKalmanR,1.5]
  [slider,gyroZDeadband,0.03]
  [slider,staticBiasTrack,1]
  [slider,staticBiasAlpha,0.999]

  Place car still for 3~5 seconds.

  [key,mpuCalib,down]
  [key,yawZero,down]

  [slider,staticBiasTrack,0]
  [key,yawZero,down]
  ```

  When testing manual rotations or gyro scale, turn off static bias tracking:

  ```text
  [slider,staticBiasTrack,0]
  ```

  After testing or before normal runs, static bias tracking may be re-enabled:

  ```text
  [slider,staticBiasTrack,1]
  ```

  ## K4 MPU calibration flow

  K4 should be treated as a local shortcut for MPU debug and calibration.

  When K4 is pressed in command-wait state, the intended flow is:

  1. force PWM zero
  2. stop motors
  3. reset PID if needed
  4. enter local MPU debug mode
  5. set `g_plotMode = 2`
  6. restore recommended MPU parameters if the current code supports that
  7. wait briefly for physical key-release vibration to settle
  8. enable `g_staticBiasTrackEnable = 1`
  9. calibrate GyroZ bias
  10. reset yaw
  11. disable `g_staticBiasTrackEnable = 0` for manual rotation test
  12. remain in MPU debug plot mode

  This makes K4 closer to the known-good web debug flow.

  Do not make K4 start a task or drive the motors.

  ## Serial protocol

  Packet format:

  ```text
  [type,field1,field2,...]\r\n
  ```

  Supported categories:

  | Packet type    | Purpose                                                      |
  | -------------- | ------------------------------------------------------------ |
  | `key` / `k`    | task selection/start/stop, emergency, unlock, MPU debug/calib/yaw zero |
  | `slider` / `s` | parameter tuning                                             |
  | `plot` / `p`   | MCU-to-web plotting output                                   |
  | `pid`          | optional web PID debug path, only when compiled/enabled      |

  Important key commands to preserve:

  ```text
  [key,task1,down]
  [key,task2,down]
  [key,task3,down]
  [key,task4,down]
  [key,start,down]
  [key,taskStop,down]
  [key,emergency,down]
  [key,unlock,down]
  [key,mpuDebug,down]
  [key,mpuCalib,down]
  [key,yawZero,down]
  [key,presetFast,down]
  ```

  Removed legacy encoder debug commands:

  ```text
  [key,encDebug,down] / [key,encoderDebug,down] / [key,encDbg,down] are ignored.
  [key,encClear,down] / [key,encoderClear,down] / [key,clearEnc,down] are ignored.
  ```

  Important slider groups to preserve:

  ```text
  RP
  straightSpeed
  yawKp
  yawKd
  gyroZScale
  gyroZKalmanEnable
  gyroZKalmanQ
  gyroZKalmanR
  staticBiasTrack
  staticBiasAlpha
  gyroZDeadband
  task2SearchPulse
  arcMinPulse
  arcFoundMs
  arcLostMs
  alignWaitMs
  entrySpeed
  entryYawKp
  entryYawKd
  entryTurnLimit
  task2BcEntryYaw
  task2DaEntryYaw
  task3AcYaw
  task3BdYaw
  ```

  Lightweight decimal parsing may only support ordinary decimals such as:

  ```text
  1
  1.053
  -20.5
  .5
  0.999
  ```

  Do not rely on scientific notation unless the current code explicitly supports it.

  ## Bluetooth remote driving is disabled

  The current project goal is to remove Bluetooth remote driving while keeping Bluetooth parameter tuning and task control.

  Allowed motion sources:

  1. physical K3 starts a selected task
  2. `[key,start,down]` starts a selected task
  3. explicitly requested standalone test functions

  Do not re-enable these remote-driving features:

  ```text
  [key,Bluetooth,down] entering manual drive
  [joystick,...] controlling forward/turn speed
  forward/backward/left/right remote driving
  speedUp/speedDown remote speed control
  legacy BT_proto.c remote-control behavior
  ```

  `[joystick,...]` packets should be ignored or safely acknowledged according to the current protocol policy, but must not move the car.

  ## Safety rules

  - `g_safetyLocked` blocks motion.
  - Emergency stop must force PWM zero and stop motion.
  - Unlock must not automatically start motion.
  - On startup, PWM must be zero and the car must not move.
  - Do not add any behavior where receiving parameter packets makes the car move.
  - Do not allow Bluetooth remote packets to drive the car.
  - Before any calibration that assumes stillness, stop motors and force PWM zero.

  ## Plot modes

  `g_plotMode` values:

  | Value | Meaning                    |
  | ----- | -------------------------- |
  | 0     | basic telemetry            |
  | 1     | deprecated / removed       |
  | 2     | MPU debug                  |
  | 3     | straight/yaw debug         |
  | 4     | arc/task debug             |
  | 5     | web PID tuning, if enabled |

  For `plotMode = 2`, preserve these meanings:

  ```text
  CH1: mode code
  CH2: yaw * 10
  CH3: final gyroZ * 10
  CH4: raw gyroZ * 10
  CH5: bias * 10
  ```

  Additional channels may exist; do not break the first five channel meanings.

  ## Current debugging sequence

  Recommended bring-up sequence:

  ```text
  1. Confirm Keil compile result.
  2. Flash firmware.
  3. Put car on a stand; confirm wheels do not move on power-up.
  4. Test basic serial protocol responses.
  5. Test MPU debug and calibration.
  6. Test task1.
  7. Tune task2 B->C entry and arc.
  8. Run full task2.
  9. Tune task3.
  10. Add or improve task4 only after task3 is stable.
  ```

  Basic serial test:

  ```text
  [key,unlock,down]
  [slider,RP,55]
  [slider,straightSpeed,30]
  [slider,yawKp,1.8]
  [slider,gyroZScale,1.000]
  ```

  MPU debug test:

  ```text
  [key,mpuDebug,down]
  [key,mpuCalib,down]
  [key,yawZero,down]
  ```

  Task1 test:

  ```text
  [key,task1,down]
  [key,start,down]
  ```

  Task2 test:

  ```text
  [key,presetFast,down]
  [key,task2,down]
  [key,start,down]
  ```

  Task3 test:

  ```text
  [key,task3,down]
  [key,start,down]
  ```

  For task3 default startup, the car is placed at A facing A->B, yaw zeroed, and the program uses `task3AcYaw = 39` degrees by default. Do not assume the car starts physically facing A->C unless the user explicitly changes `task3AcYaw` to 0.

  ## Coding style

  - Use existing naming style.
  - Public functions generally use `Module_FunctionName()`.
  - Keep internal helpers `static`.
  - Prefer explicit fixed-width integer types where appropriate.
  - Keep memory usage low.
  - Do not use `malloc` / `free`.
  - Avoid expensive floating-point formatting in frequent telemetry if code size is tight.
  - Use `Serial_Printf()` for UART output.
  - Use `OLED_Printf()` for OLED output.
  - Avoid direct `printf()` unless the current file already intentionally uses it.
  - Header guards follow the existing project style.

  ## Keil Lite code-size awareness

  The free Keil MDK Lite limit may cause linker failure around 32KB code size.

  If code size becomes a problem:

  - prefer small helper functions
  - avoid pulling in heavy libc functions
  - avoid unnecessary `printf` float formatting
  - keep debug features behind compile-time flags
  - do not add large tables or general-purpose parsers
  - preserve existing code-size reduction measures

  Known intentional reductions may include:

  - legacy `BT_proto.c` disabled or reduced to empty `BT_Process()`
  - Bluetooth remote drive disabled
  - long status replies compressed
  - old standalone debug paths optionally compiled out

  ## When modifying code

  Before editing:

  1. identify the exact function and file
  2. keep the state machine structure
  3. preserve task1/task2/task3
  4. check whether a change affects safety or startup motion
  5. check whether a new global needs `extern` declaration
  6. check whether a new source file must be added to `.gitignore` and `Project.uvprojx`

  After editing, report:

  1. files changed
  2. key logic changed
  3. behavior before vs after
  4. manual test steps
  5. whether task1/task2/task3 are affected
  6. whether Keil project file or `.gitignore` needs update

  Do not claim hardware behavior is verified unless the user provides test results.
  Do not claim the Keil project builds unless a real Keil build log is available.
