# Adding obstacles to the dwa_test world

Copy the block below, paste it into `worlds/dwa_test.sdf` right before `</world>`,
then just change `<pose>` and `name`.

## Wall (box)

`<box><size>thickness length height</size></box>`:

```xml
<model name='wall_3'>
  <static>true</static>
  <pose>X Y Z 0 0 0</pose>
  <link name='link'>
    <collision name='collision'>
      <geometry><box><size>0.2 4.5 1</size></box></geometry>
    </collision>
    <visual name='visual'>
      <geometry><box><size>0.2 4.5 1</size></box></geometry>
      <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material>
    </visual>
  </link>
</model>
```

## Cylinder

```xml
<model name='cylinder_3'>
  <static>true</static>
  <pose>X Y Z 0 0 0</pose>
  <link name='link'>
    <collision name='collision'>
      <geometry><cylinder><radius>0.3</radius><length>1</length></cylinder></geometry>
    </collision>
    <visual name='visual'>
      <geometry><cylinder><radius>0.3</radius><length>1</length></cylinder></geometry>
      <material><ambient>0.8 0.2 0.2 1</ambient><diffuse>0.8 0.2 0.2 1</diffuse></material>
    </visual>
  </link>
</model>
```

## Key points

- `pose`'s Z is usually set to `<length>/2` (half the height) so the base sits on the ground.
- `name` must be unique per model; duplicates get ignored or cause gz errors.
- For a wall, the three numbers in `<box><size>` are, in order, "thickness (perpendicular to
  the wall face), length (along the wall), height"; for a cylinder, use `radius`/`length`
  the same way (`length` is the cylinder's height, not its radius).

## Spawning obstacles live, without restarting the simulation

If the sim is already running and you don't want to edit a file and restart, you can use
`gz service` to spawn an obstacle live without restarting the whole simulation (swap the
world name for the one you're actually running, e.g. `dwa_test`):

```bash
gz service -s /world/dwa_test/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'sdf: "<sdf version=\"1.9\"><model name=\"runtime_cyl\"><static>true</static><pose>6 0 0.5 0 0 0</pose><link name=\"link\"><collision name=\"c\"><geometry><cylinder><radius>0.3</radius><length>1</length></cylinder></geometry></collision><visual name=\"v\"><geometry><cylinder><radius>0.3</radius><length>1</length></cylinder></geometry></visual></link></model></sdf>"'
```
