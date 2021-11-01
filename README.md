# Hybrid A*-based Push Planner

It's a 2D push planner made in a hurry and messy. So, it is very slow.

`hybrid a-star algorithm`

## Demo

The current timeout is 10 secs.

```sh
python debug.py
```

- Thin green line: object start
- Bold green line: object goal
- Light purple: robot (pusher)
- Black dots: point obstacles

![example](Peek%202021-11-01%2010-54.gif)

### Demo Key Settings

- Number key `1`: Obstacle editing mode
  - `left click`: Set obstacle on the current mouse position
  - `space`: Remove all obstacles
- Number key `2`: Object start (X, Y, Orientation) editing mode
  - 1st `left click`: Set XY
  - 2nd `left click`: Set direction
- Number key `3`: Object goal (X, Y, Orientation) editing mode
  - 1st `left click`: Set XY
  - 2nd `left click`: Set direction
- Number key `4`: Path computing mode
  - You can toggle target orientation option by pressing number key `4`.
    - **goal_XY**(`Mode: 4 (ig: True)` on your terminal) or
    - **goal_XYTheta**(`Mode: 4 (ig: False)` on your terminal)
  - `left click`: **Compute path**
- Number key `5`: Pause and show belows:
  - Grid (gray line)
  - Successors of the start(red line)
  - Mouse position (blue circle)
  - Node position for the mouse position (yellow circle)
  - The closest obstacle from the mouse position(red circle)
- Key `` ` `` (backtick): Relase pause and remove grid
