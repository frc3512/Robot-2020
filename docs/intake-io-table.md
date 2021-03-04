The following table shows the relevant inputs and the desired allowable
actuations for each input combination.

| Inputs     |               |                    |                    | Outputs        |                  |                   |                 |                   |
|------------|---------------|--------------------|--------------------|----------------|------------------|-------------------|-----------------|-------------------|
| flywheelOn | flywheelReady | upperSensorBlocked | lowerSensorBlocked | allowTeleopArm | armAutoDirection | allowTeleopFunnel | funnelAutoSpeed | conveyorAutoSpeed |
|      false |         false |              false |              false |            Yes |              N/A |               Yes |             N/A |               0.0 |
|      false |         false |              false |               true |            Yes |              N/A |                No |             0.4 |               0.7 |
|      false |         false |               true |              false |            Yes |              N/A |               Yes |             N/A |               0.0 |
|      false |         false |               true |               true |            Yes |              N/A |                No |             0.0 |               0.0 |
|       true |         false |              false |              false |             No |           Intake |                No |             0.0 |               0.0 |
|       true |         false |              false |               true |             No |           Intake |                No |             0.4 |               0.0 |
|       true |         false |               true |              false |             No |           Intake |                No |             0.0 |               0.0 |
|       true |         false |               true |               true |             No |           Intake |                No |             0.0 |               0.0 |
|       true |          true |              false |              false |             No |           Intake |                No |             0.4 |              0.85 |
|       true |          true |              false |               true |             No |           Intake |                No |             0.4 |              0.85 |
|       true |          true |               true |              false |             No |           Intake |                No |             0.4 |              0.85 |
|       true |          true |               true |               true |             No |           Intake |                No |             0.4 |              0.85 |

Assign the output booleans based on the input conditions that make it true.
```
allowTeleopArm = !flywheelOn
allowTeleopFunnel = !flywheelOn && ((!upper && !lower) || (upper && !lower))
                  = !flywheelOn && !lower
```

Arm logic:
```
# if !allowTeleopArm:
if flywheelOn:
    armAutoDirection = Intake
else:
    # Do teleop arm
```

Funnel logic:
```
# if !allowTeleopFunnel:
# if !(!flywheelOn && !lower):
# if !!flywheelOn || !!lower:
if flywheelOn || lower:
    # if (!flywheelReady && !upper && lower) || (flywheelReady):
    if flywheelReady || (!upper && lower):
        funnelAutoSpeed = 0.4
    else:
        funnelAutoSpeed = 0.0
else:
    # Do teleop funnel
```

Conveyor logic:
```
if flywheelReady:
    conveyorAutoSpeed = 0.85
# elif !flywheelOn && !flywheelReady && !upper && lower:
elif !flywheelOn && !upper && lower:
    conveyorAutoSpeed = 0.7
else:
    conveyorAutoSpeed = 0.0
```
