# README

## Design Goals
- Enable 'FastBoot' Architecture
- Optimize CANBus usage using default parameters
- Should be lightweight to implement on top of existing code


## Implementation
- Base it off of CommandScheduler, Subsystems, and Epilogue
- Each Subsystem has a list of devices
  - A subsystem must be initialized with its devices
  - Device configuration needs to be split from subsystem constructor/init
- Each loop
- Hardware manager should use a separate thread for specific actions
  - Fastboot
  - Device configuration


## Health Status

### Subsystem Health
- All subsystems should have a health status
- The logic for this should generally be custom per subsystem (?)
  - Too complex to implement/maintain
  - Custom logic should just be appended if needed
- The logic should be based on the hardware of the subsystem
  - CTRE devices make this easy to implement using signals

- A subsystem is healthy if all primary devices are polled without issues
- A subsystem is degraded if its secondary devices cannot be polled
- A subsystem is unhealthy if any primary device cannot be polled

### Device health
- CANcoder:
  - MagnetHealth
- Talon
  - MotorOutputStatus (Response)

- Ctre bus optimization
  - Signals by device priority, device type
  - Add custom signals
### Subsystem health
- Check all primary devices
- Check all secondary/backup devices

### CTRE Signals


## Fast boot
- Speed up boot process by performing initial checks to decide what actions to check
  - Don't reconfigure devices if robot has been on for some time (?)
    - Can cause coder confusion on testing
      - Implement some hash check on critical config files to see if things should be reconfigured
  - Don't load/construct auto routines if booting while match is running
    - Check if FMS is connected and current match state
      - TODO: Investigate what state the robot ends up in with Chezy Arena
  - Poll hardware for status and decide which subsystems to enable
    - SwerveDrive should always be configured
    - Remaining subsystems should be incrementally enabled using a thread
    - Need to reconfigure bindings as things come up
      - Consider flexible commands that can work with partially enabled subsystems

### Init process
- Poll all devices for response
  - CANBus devices should be easy
  - Poll only Swerve devices first
- Initialize Swerve, regardless of hardware state
  - Alert driver on errors
- Default initialization
  - Normal init
- Staggered initialization of all subsystems
  - Use a thread to bring up subsystems
  - Check if autos need to be built
  - Check if devices need to be configured

## Runtime
- Query all hardware device signals
- Check health
- Update heath report

## Things to look into to make this easier
- Create subsystem interface to check for member objects that are valid (motors, sensors)
- Create wrapper classes for common devices that automatically register (?)
- Interface vs annotation.
  -
- Splitting hardware vs categorizing them by subsystem
  - Need to categorize by subsystem to roll up subsystem health, but init only needs all devices
    - Subsystem constructors should not init devices (?)
  - Centralize hardware init prior to subsystem init(?)
- Move all device init into a separate class
  - Annotate device priority
