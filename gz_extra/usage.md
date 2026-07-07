# 在 dwa_test world 裡加障礙物

複製下面的 block、貼在 `worlds/dwa_test.sdf` 裡 `</world>` 之前，改 `<pose>` 跟
`name` 就好。

## 牆（長方體）

`<box><size>厚度 長度 高度</size></box>`：

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

## 圓柱

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

## 要點

- `pose` 的 Z 通常設成 `<length>/2`（高度的一半），讓底部貼地。
- `name` 每個 model 要唯一，重複會被 gz 忽略或報錯。
- 牆的 `<box><size>` 三個數字依序是「厚度（垂直牆面方向）、長度（沿牆延伸方向）、
  高度」；圓柱同理用 `radius`/`length`（`length` 是圓柱的高度，不是半徑）。

## 不重開模擬、即時生成障礙物

如果 sim 已經在跑、不想改檔重開，可以用 `gz service` 即時生成一個障礙物，不用重啟
整個模擬（world 名稱要換成你實際跑的那個，例如 `dwa_test`）：

```bash
gz service -s /world/dwa_test/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'sdf: "<sdf version=\"1.9\"><model name=\"runtime_cyl\"><static>true</static><pose>6 0 0.5 0 0 0</pose><link name=\"link\"><collision name=\"c\"><geometry><cylinder><radius>0.3</radius><length>1</length></cylinder></geometry></collision><visual name=\"v\"><geometry><cylinder><radius>0.3</radius><length>1</length></cylinder></geometry></visual></link></model></sdf>"'
```
