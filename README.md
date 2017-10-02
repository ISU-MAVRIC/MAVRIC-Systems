# MAVRIC-Electrical
The repository for the E-Team. Includes designs and documentation.

# Team Structure
Team Lead: James Talbert (@Talonj123)

Members:
 * Morgan Foley (Circuit Design/E-CAD)
 * Jefferson O'Brien (Control Systems)
 * Jacob Raymer (Motors And Power)
 * Alexander Vande Loo (Programming)

# Folder Structure:

```
|> Electrical
|  |> Design Documents // The system diagrams, datasheets, etc.
|  `> Implmentation    // KiCad deigns, simulation data, etc.
|
|> Firmware
|  |> Design Documents // The system diagrams, datasheets, etc.
|  `> Implmentation    // The code
|     `> [Catkin Worksapce]
|
`> Base Station
   |> ?                // We still need to decide more about this
```

# Milestones [Proposed]
|                     | Milestone 1  | Milestone 2 | Milestone 3 |
|:--------------------|:------------:|:-----------:|:-----------:|
| Name                | Drive System | Refinement  | Arm Control |
| Due Date            |    Nov. 12   |   Nov. 27   |   Dec. 03   |

## Milestone 1:  [Drive System](https://github.com/m2i/MAVRIC-Electrical/milestone/2)
Description: Control of the drive system. Includes basic forward, backward, and turn maneuvers as well as heading and position feedback to the controller.

Notes: Phoenix will need to have at least serial comms, but we should finalize a communications stack before we get too far on this. We can simulate a Rocket system by hooking everything up on a local network. If we go with a network-bridge solution like the Rocket, we will most likely use sockets. Sockets will give us the flexibility to open up multiple 'channels' to and from Phoenix.
### Tasks
 * Establish basic connectivity with the rover
   * Sockets over a Rocket antenna bridge.
   * Requires a master Control Board and Drive Control Board
 * Define control parameters
   * Open Loop: (`Output Power`, `Left/Right Output Ratio`)
   * Open Loop Direct: (`Left Output Power`, `Right Output Power`)
   * Closed Loop: (`Velocity`, `Turn Radius`)
   * Closed Loop Direct: (`Left Velocity`, `Right Velocity`)
 * Implement Control Nodes
 * Implement sensing
   * GPS
   * Compass

### Deliverables
 * Video of Phoenix Driving in the Howe atrium
 * Description of control systems and sensors in place, including safety systems
   * Communication setup
   * Control Parameters chosen
   * Hartbeat signal
   * Positioning?
   * Heading?

## Milestone 2: [System Refinement](https://github.com/m2i/MAVRIC-Electrical/milestone/3)
Description: After we have a drivable rover, we will refine the internals
to enhance the extensibility and maintainability of the system. This will
allow us to integrate new components easier.

Notes: 
### Tasks
 * Refine communications protocol
   * Extensible packet structure with different commands
 * Refine Firmware Systems
   * Modularize the components
   * Structure the IPC namespaces
 * Refine the Electrical systems
   * Check wiring
   * Evaluate the need for fuses and other HW saftety systems

### Deliverables
 * System architecture documents
   * ROS architecture
   * Electrical architecture
 * Electrical Changes summary
   * A summary of the changes made to the electrical system
   * Includes any changes evaluated, but not made

## Milestone 3: [Arm Control](https://github.com/m2i/MAVRIC-Electrical/milestone/4)
Description: Add the ability to control a robotic arm, as specified by the Mechanical team.
The arm has:
 * 2-axis Shoulder Joint (DC Motors)
 * Elbow Joint (DC Motors)
 * 3-axis Wrist (Servos?)
 * Claw (?)

Notes: 
### Tasks
 * Develop logic-level hardware drivers
 * Develop power systems (power supply and Motor Drivers)
 * Develop control systems
 * Define Communication protocol for the base station

### Deliverables
 * Updated System architecture documents
   * ROS architecture
   * Electrical architecture
 * Packet structure writeup
 * Video of an operator manipulating each joint
