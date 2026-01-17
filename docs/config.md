# Config

This document explains how to create a configuration for an arm
with modular parameters.

## Example

```yaml
joints:
  - abc:
    position:
      x: 1
      y: 2
      z: 3
    angle:
      x: 1
      y: 2
      z: 3
    maxAngle:
      x: 1
      y: 2
      z: 3

gripper:
  state: Close # Close | Open
```

## Parameters

### joints

The `joints` parameter allows you to define all the properties
of the joints of a given robotic arm, from the base to the end
of the arm. A collection is expected, giving a unique name to
each of the joints that make up the arm.

```
joints: collection
```

#### position

This parameter allows you to define the absolute starting position
in x, y and z of the joint.

```
x: float
y: float
z: float
```

#### angle

This parameter allows you to define the initial tilt angle in x, y
and z of the joint.

```
x: float (degree)
y: float (degree)
z: float (degree)
```

#### maxAngle

This parameter allows you to define the max tilt angle in x, y and
z of the joint.

```
x: float (degree)
y: float (degree)
z: float (degree)
```

### gripper

The `gripper` parameter allows you to define all the properties of a
gripper on a given arm.

```
clamp: structure
```

#### state

The `state` parameter is used to define the starting state of the
gripper of a given arm: open or closed.

```
state: string (Close | Open)
```
