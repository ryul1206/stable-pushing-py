# Hybrid A*-based Push Planner

It's a 2D push planner made in a hurry and messy. So, it is very slow.

`hybrid a-star algorithm`

가장 최근의 작업물은 `socialrobot_reasoner-devel-pusher_node.zip` 파일에 덤프되어 있다. 알고리즘은 동일한 Hybrid A*이고, 이웃 샘플링할 때 cost 계산만 조금 다르다. 예를 들어, `sqrt` 같은 것을 적게 사용하는 좀더 단순하고 빠른 방법을 사용하면 계산속도 뿐만 아니라, 정답 수렴도 더 적은 node 확장으로 가능했다.

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
