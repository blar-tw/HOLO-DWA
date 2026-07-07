# HOLO-DWA 評分系統迭代記錄

> **TL;DR(最終結果)**:baseline 1/5 reached、44+ 碰撞 → **iter4:
> 5/5 reached、0 碰撞、最小距離 0.66m、平均 16.8s**。四個關鍵修正:
> ① velocity 改 blend(治開闊區斜飄)② clearance 改方向性 1.5m 探測
> (治蠕行陷阱)③ **LiDAR 點雲鏡像 bug**(gz FLU vs PX4 FRD,flip_y=True,
> 全部失敗的真正主因)④ goal bonus 改煞車曲線(治繞終點軌道)。
> DWA 的 (vx,vy) 動態視窗、可行性遮罩、預測機制皆未更動。

目標:優化評分系統,讓無人機**清楚避開所有障礙物**(0 碰撞)且可重複驗證。
限制:不改 DWA 的 (vx, vy) 動態視窗搜尋結構;只動評分核心 / 參數 / scanner 端配置。
場景:`dwa_test.sdf` — x=3 牆(1.5m 缺口@y=0)→ x=6 三圓柱 slalom → x=9 方柱門(2m 缺口)→ 目標 (12, 0)。
驗證:`./exp.sh up` + `./exp.sh go <name>`,每輪 N_RUNS=5、RUN_TIMEOUT=90s,看
`logs/exp/<name>/report.txt`(runs 統計 + ASCII 軌跡圖)。

判定順序(好壞比較):
1. 碰撞 runs 數(最重要,必須 0)
2. reached 數(5/5 才算通過)
3. min_dist_m(全批次最小雷達距離,>0.4m 才安心)
4. 平均時間 / path_efficiency(其次)

## 工具

- `exp.sh up|go|collect|status|down` — 實驗驅動器(詳見檔頭)。
- `report.py` — session CSV → ASCII 軌跡圖 + 碰撞/infeasible/stall 診斷。
- 每輪存檔於 `logs/exp/<name>/`:summary.jsonl、session CSV、report.txt、
  dwa_core.py.snap、node_config.txt、pane.txt。

## 基準配置 (baseline, 2026-07-06)

`dwa_core.py`:heading=角度餘弦(距離無關)、clearance=clip(safe_dist,0,1)、
velocity=scalar 模式;權重 H/C/V = 0.2/0.2/0.6。
`scanner_lab.py`:v_max=1.5、a_max=1.0、predict 3.0s@0.2s、robot_radius=0.2、
goal_threshold=0.5、collision_dist=0.3、lidar_stride=6。

已知(來自 docs/discussion.md 與舊 log):
- scalar 速度獎勵在加速段會斜飄;heading 已改角度式理論上可壓住,待驗證。
- 曾觀測到:圓柱區卡死(infeasible 連發)→ 往北暴衝 → 撞北邊界牆 @gz(5.1, 8.0)。
- robot_radius(0.2) < collision_dist(0.3):規劃器被允許貼到 0.2m,但 0.3m 內
  就記碰撞 —— 這個不一致本身就會製造「碰撞」記錄。候選修正。

---

## 實驗記錄

(每輪:動機 → 改動 → 結果 → 決定)

### exp 0 — baseline (2026-07-06 21:30)

配置:同「基準配置」。`logs/exp/baseline/`。

結果:**大失敗(預期中)** — 5 runs:reached 1(帶 4 次碰撞)、timeout 4;
碰撞事件 44+;max_speed 到 12.4 m/s(v_max=1.5 → 撞牆被物理彈飛);
infeasible tick 比例 15~30%。

逐 tick 分析(session CSV)發現的機制:
1. **方形視窗角落簡併**:加速段角落候選的 |v| 紅利 (0.6·Δ|v|/1.5) ≈ heading
   餘弦懲罰 (0.2·Δcos),argmax 被噪音翻轉 → 隨機斜飄(見 t=12.1 由直轉斜)。
   飄離軸線後面對的是牆段而不是缺口 → 卡牆 → 貼牆 → 撞。
2. **min_clearance=0.007m**:clip(safe,0,1)·0.2 的懲罰太弱,擋不住
   velocity 0.6 的誘因,規劃器會主動選 0 邊距軌跡。
3. **radius(0.2) < collision_dist(0.3)**:結構性必撞。第一次碰撞就是穿缺口時
   貼邊 0.279m(t=5.5, gz(2.57, 0.21))。
4. 撞牆彈飛後 |v| > v_max → 全部候選 infeasible → 煞車指令,PX4 慢慢拉回來,
   但 90s 內回不到正軌 → timeout。根治 = 不要撞第一下。

決定:iter1 = {robot_radius 0.30, velocity_mode blend(α0.5),
clearance_norm 0.5, 權重 H/C/V 0.3/0.3/0.4}(機制各自對應 1~3)。

### exp 1 — iter1:blend + norm0.5 + r0.3 + 權重334 (2026-07-06 21:43)

`logs/exp/iter1/`(只收了 2 runs 就提前中止,機制已明)。

結果:部分成功、新病浮現。
- ✅ 開闊區斜飄消失:起點到第一道缺口是筆直路徑,乾淨穿過 1.5m 缺口。
- ❌ run1 reached 75s 但 12 次碰撞:在圓柱區前**蠕行**(0.2 m/s),然後以
  heading=1.0 直直蹭進 cylinder_2 正面(t+20.6s, gz(6.03,0.19), 0.29m)→
  擦撞 → 物理彈飛 (6.9 m/s) → 混亂。
- 離線 sim 同步復現蠕行:iter1 19.4s vs baseline 12.1s。

根因(逐 tick 算分驗證):**時間式 clearance(整條 3 秒射線的最小距離)
與速度耦合** — 慢速候選射線短 → 天然高 clearance → C 項獎勵慢速
(ΔC=0.3×0.24 > ΔV=0.4×0.2/1.5),而慢速短射線又「看不到」1m 外的圓柱
→ 可行性也擋不住 → 逐步蹭進死路。「蠕行陷阱」。

決定:iter2 = clearance 改**方向性**:沿候選單位方向固定取 1.5m 內最小
障礙距離,與速度無關。方向品質(H+C)與速度選擇(V+煞車上限)徹底分離。
離線 sweep:iter2 13.5s、min_surf 0.63m,全面優於 iter1;噪音 8-seed
worst_surf 0.60m 也最佳。L=1.5 優於 1.0/2.0。

### exp 2 — iter2:方向性 clearance (2026-07-06 21:52)

`logs/exp/iter2/`(run 1 後中止 — 找到真正的根因)。

結果:蠕行消失(10.7s 就抵達圓柱區,iter1 要 20.6s),第一道缺口直穿,
但 **run 1 仍然 7 次碰撞**,第一撞又是 cylinder_2 正面。

Tick 級偵查(t+17.5~21.3)發現無法解釋的矛盾:朝圓柱的候選被記
clearance_score=0.84,但用同一 state + 正確場地幾何離線重算 = 全部
infeasible。**節點看到的點雲和真實世界不一致。**

### 🐛 根因:LiDAR 掃描座標系手性鏡像(sensing bug,不是評分問題)

- gz `gpu_lidar` 的掃描角是 z-up 慣例:**正角度朝機體左**(FLU)。
- `scan_to_world_points` 的旋轉假設 NED/FRD:**正角度朝機體右**。
- 兩者疊加 → **整個點雲沿機體 x 軸鏡像**(左右互換)。

為什麼之前沒炸更慘:場地中牆缺口、方柱門、邊界全部**左右對稱**,
鏡像後幾乎不變;**只有三根圓柱是非對稱布局** — 所以每一輪的第一次
碰撞都精準發生在 slalom:躲避「南邊幻影圓柱」→ 轉向北 → 撞上真的
cylinder_2。開闊區莫名其妙的 infeasible 煞車(15~30% tick)也是幻影牆。

修正:`scan_to_world_points(..., flip_y=True)`(holo_lab 副本內),
log 的威脅角度同步轉 FRD。註:**repo 根目錄的 scanner.py 有同樣的
bug**,不在本次行動範圍內,留待使用者確認後修。

驗證:`verify_frame.py` — 抓一筆實際掃描,翻/不翻各轉一次世界座標,
對已知場地幾何算表面距離統計;正確手性應在公分級。

### exp 3 — iter3 = iter2 評分 + flip_y 修正 (2026-07-06 22:12)

`logs/exp/iter3/`(run 1 後中止 — 又抓到一個新機制)。

結果:**避障完全成功** — run 1 全程 0 碰撞、最小雷達距離 0.633m、乾淨
穿過三關 — 但 **timeout:繞著終點軌道飛,進不了 0.5m 判定圈**。

根因:goal-reached 加分 `10000 + speed` 獎勵「最快」穿圈軌跡 → 以
v_max 衝向終點 → 追蹤誤差讓實際路徑擦圈而過 → ±0.2/tick 的視窗要
~15 tick 才能掉頭 → 大迴圈重試 → 永恆軌道(地圖上多個大圈)。

修正歷程:
- `10000 − speed`(最慢穿圈):離線 20.7s — 最後 4.5m 全程爬行,太保守。
- **`10000 − |speed − v_target|`,v_target = min(v_max, √(2·brake_a·(d−r)))**
  (煞車曲線):遠處全速、進圈前自然減到 ~1.0 m/s(PX4 可在 0.17m 內煞停)。
  離線 14.1s(魯莽版 13.5s,只慢 0.6s),邊距不變 0.63m。

### exp 4 — iter4 = iter3 + 煞車曲線到達 (2026-07-06 22:09) ✅

`logs/exp/iter4/`。

| run | outcome | dur_s | path_m | eff | col | min_dist | infeasible |
|----:|---------|------:|-------:|----:|----:|---------:|-----------:|
| 1 | reached | 17.6 | 11.9 | 1.01 | 0 | 0.661 | 0/351 |
| 2 | reached | 17.5 | 19.7 | 0.60 | 0 | 0.787 | 0/349 |
| 3 | reached | 16.0 | 19.8 | 0.61 | 0 | 0.754 | 0/318 |
| 4 | reached | 16.4 | 19.8 | 0.62 | 0 | 0.800 | 0/327 |
| 5 | reached | 16.5 | 19.9 | 0.62 | 0 | 0.785 | 0/330 |

**5/5 reached、0 碰撞、全批次最小雷達距離 0.661m、平均 16.8s、
0 infeasible tick(連一次緊急煞車都沒有)、compute ~1.7ms。**

路線兩類,都合法:
- run 1(冷啟動、零初速):**三關直線** — 缺口 → c1–c2 通道南繞
  cylinder_2 → 方柱門 → 終點,path 11.9m 近乎最優。
- runs 2-5(RETURN_HOME 後、帶殘餘速度的初始條件):**北側大弧**,
  走牆北端外的 3m 開放走廊(y∈[6,9],場地本有)繞過全部障礙。
  從偏北初始條件看,寬走廊的 H+C+V 分數勝過重新對準 1.5m 縫隙。

### 重複驗證 iter4 (verifyA / verifyB, 2026-07-06 22:2x)

兩個獨立批次、各自全新 stack,各 5 runs。`logs/exp/verifyA|verifyB/`。

- verifyA:4/5 reached、**0 碰撞**、min_dist ≥ 0.649m。run 1 timeout:
  **繞終點飛掠軌道**,90 秒轉圈(半徑 1.5~2.4m)進不了 0.5m 圈。
- verifyB:5/5 reached、**0 碰撞**、min_dist ≥ 0.676m。run 2 慢
  (59.6s / 67m):飛掠繞了一整圈後才被捕獲。

iter4 三批合計:**15 runs、0 碰撞、14/15 reached** — 安全性達標,
終端捕獲有 2/15 的飛掠缺陷。

Tick 級 + 離線重現診斷(verifyA run 1, t=35.6):
- bonus 觸發次數 = **0/1800 tick**,最近距離 1.478m。
- 無人機「已經在視窗允許範圍內全力轉向」(vy 只能 −0.23→−0.43),
  1.5 m/s 切線速度下轉彎半徑 ~4m ≫ 0.5m 判定圈 → 穩定極限環。
- 根因:goal bonus 是**二元開關**(射線穿過 0.5m 圈才觸發)。飛掠
  幾何的射線永遠不穿圈 → 完全沒有終端引力。

### exp 5 — iter5 = 連續終端吸引盆 (2026-07-06 22:4x)

改動(dwa_core.py):射線最近點 < `goal_capture`(2.0m)的候選改用
`10000 − 10·min_goal_dist − |speed − √(2·brake_a·(d−r))|` 評分 —
毫米級接近優先(×10 讓「距離」永遠壓過「速度匹配」)、速度貼合煞車
曲線次之。任何近距離經過都會被連續地拉向圓心,以可煞停的速度到達。

離線:標稱 13.9s(不變),四個切向飛掠壓力測試(從 (10,4)、(14,−4)、
(8,−5) 等偏置起點)全部 3.6~5.2s 直接收斂、無繞行。

最終驗證協議:3 個獨立批次 × 5 runs,全新 stack。
通過標準:15/15 reached、0 碰撞、min_dist ≥ 0.4m。

結果:
- **iter5a:5/5 reached、0 碰撞、min_dist ≥ 0.681m、17.9~19.3s**
- iter5b:❌ 無效批次 — 模擬器 RTF 崩潰(TAKEOFF 爬升花 92 wall 秒,
  位置對 wall clock 近乎冻結),5 runs 原地 timeout。**即使如此仍
  0 碰撞**。屬基礎設施 flake(WSL2 + 重複重啟 gz/gpu_lidar),非演算法。
- **iter5c:5/5 reached、0 碰撞、min_dist ≥ 0.658m、17.0~22.1s**
- **iter5b2(替代批次):5/5 reached、0 碰撞、min_dist ≥ 0.691m、16.6~20.6s**

### ✅ 最終判定 (2026-07-06 23:0x)

**15/15 reached、0 碰撞、全部 45 個健康 run tick 中最近雷達距離 0.658m
(標準 0.4m)、平均 18.2s、compute ~1.7ms(50ms 預算的 3%)。**

對照 baseline:reached 1/5 → 15/15;碰撞事件 44+ → 0;
90s timeout 常態 → 平均 18.2s。目標(清楚避開所有障礙物、可重複驗證)達成。

重現方式:
```bash
cd ~/ws/src/HOLO-DWA/holo_lab
./exp.sh up                    # 全新 stack, N_RUNS=5, RUN_TIMEOUT=90
EXP_TIMEOUT=700 ./exp.sh collect <name>
cat logs/exp/<name>/report.txt # 表格 + ASCII 軌跡圖
```

---

## 後記 (2026-07-07):demo 錄影與「GUI 為什麼變慢」

### 現象與對照實驗

使用者手動 `./run_lab.sh`(帶 GUI)跑出 50~90s 的繞路 run,懷疑
run_lab 不如 exp。逐一對照(同一天、同一機器、同一程式碼):

| 模式 | 結果 |
|------|------|
| 整合 GUI (`./run_lab.sh`) | 59.9s / 41m,追蹤比 0.61 |
| 外掛 GUI (`gz sim -g`) | 90s timeout / 78m |
| gz 錄影 (`--record-path`) | 54.9s、88.6s / 42~83m(追蹤比卻 0.87!) |
| **純 headless(對照組)** | **18.0s / 11.9m / eff 1.019 — 完美** |

結論:**live 模擬掛上任何額外客戶端或狀態錄影都會讓飛行劣化**
(機制未完全釐清 — 追蹤比正常但路線遊蕩,掃描率反而更高 20Hz vs 9Hz;
總之是 WSL2 上 gz server 的負載互動,非演算法問題)。exp.sh 與
run_lab.sh 邏輯相同,差別只在 HEADLESS=1。

### 解法一(演算法穩健化):goal_approach_a = 0.5

原煞車曲線用 brake_a_max=1.0 → 離目標 1.6m 前都維持全速,最後 1m 急煞,
環境雜訊一大就擦圈過站、多繞一圈。新增 `Config.goal_approach_a=0.5`:
~2.5m 外開始減速、以 ~0.7 m/s 進圈。離線 nominal 只慢 0.5s(14.4s),
noise 0.1 十種子 10/10。Gazebo 重驗證:見 `logs/exp/final5/`。

### 解法二(錄影正道):replay_demo.py 傀儡回放

飛行一律 headless(乾淨 18s,CSV 就是完整 20Hz 錄影),事後用
`replay_demo.py` 在 GUI 世界裡以 set_pose 讓靜態無人機模型沿軌跡重演
(C++ 常駐 streamer,單次 set_pose ~ms 級;`gz service` 子程序一次要
280ms 不可行)。回放時渲染負載傷不到已飛完的軌跡。`--speed`、`--loop`。

死路備忘:
- SDF actor + link + trajectory:gz-sim7 SceneBroadcaster **segfault**。
- `RECORD=1` + `./run_lab.sh play`(gz 原生 state log):功能可用,
  但錄影本身就會弄髒被錄的那趟飛行 — 已保留但不建議。
- gz python transport bindings:Garden 上未安裝(只有 gz.sim7/math7)。

已知基礎設施風險(與演算法無關):偶發的 gz RTF 崩潰。健康批次的
特徵很好認:TAKEOFF 應在 ~10 wall 秒內完成;若 > 30 秒,重啟 stack。

## 最終配置 (iter5)

`holo_lab/dwa_core.py` 相對原版的改動(DWA 視窗/可行性/預測全部未動):
1. `scan_to_world_points(..., flip_y=True)` — **修 LiDAR 鏡像 bug**
   (gz z-up 掃描 vs NED/FRD 手性;全部早期失敗的第一主因)。
2. clearance 項改**方向性探測**:沿候選單位方向固定 1.5m 取最小障礙
   距離(`clearance_lookahead=1.5`),與候選速度解耦 → 蠕行陷阱消失;
   正規化 `clearance_norm=0.5`(0.5m 邊距即滿分)。
3. 終端吸引盆:`goal_capture=2.0` 內
   `10000 − 10·min_goal_dist − |speed − √(2·brake_a·(d−r))|`
   (連續引力 + 煞車曲線到達;治飛掠軌道)。
4. velocity 項 `blend`(α=0.5):開闊區走目標分量(治方形視窗角落
   簡併造成的隨機斜飄),被擋時保留 α·|v| 地板(治牆前卡死)。

`holo_lab/scanner_lab.py` 的 dwa_config:
`robot_radius 0.30`(≥ collision_dist 0.3,原 0.2 是「必撞」設計)、
權重 H/C/V = 0.3/0.3/0.4、其餘(v_max 1.5、a_max 1.0、predict 3.0s、
resolution 0.1、goal_threshold 0.5)不變。

任務完成 ✅
最終成績:3 個獨立批次 × 5 runs = 15/15 到達、0 碰撞、全程離障礙物最近 0.658m、平均 18.2 秒(baseline:1/5 到達、44+ 次碰撞、90 秒 timeout 常態)。演算法可重複驗證,所有數據存檔於 holo_lab/logs/exp/,完整迭代記錄在 holo_lab/EXPERIMENTS.md。

迭代過程(每輪都是:跑模擬 → 看數據 → 找機制 → 修 → 再驗證)
輪次	發現的問題(機制)	修法
baseline	方形視窗角落簡併:加速段角落候選的 |v| 紅利 ≈ heading 餘弦懲罰,方向被噪音亂翻 → 隨機斜飄進牆區	velocity 改 blend:目標分量為主 + α·|v| 地板保底(治飄移同時保留牆前脫困)
iter1	蠕行陷阱:clearance 取「整條 3 秒射線最小距離」,慢速=短射線=高分 → 獎勵爬行,0.2 m/s 蹭進圓柱正面	clearance 改方向性:沿單位方向固定探測 1.5m,與速度徹底解耦
iter2	tick 級重算發現點雲與世界不符 → LiDAR 鏡像 bug:gz 掃描 +角=機體左(z-up),轉換式假設 +角=機體右(FRD)→ 障礙物全部左右翻轉。場地只有圓柱陣不對稱,所以每次都精準撞圓柱	flip_y=True;寫 verify_frame.py 拿真實掃描對場地幾何驗證(0.08m vs 0.32m,鐵證)
iter3	避障完美但繞終點軌道:goal bonus 10000+speed 獎勵全速衝圈,擦邊錯過就繞大圈重來	bonus 改煞車曲線 10000−|v−√(2·a·d)|
iter4	15 runs 全 0 碰撞,但 2/15 飛掠:二元 bonus 在射線不穿 0.5m 圈時完全沒有終端引力 → 穩定極限環	改連續吸引盆:2m 內 10000 − 10·miss − |v−煞車目標|
iter5	—	最終驗證 3×5 全過
交付物(全部在 holo_lab 內)
演算法:dwa_core.py(評分核心重構;DWA 的 vx/vy 視窗、可行性遮罩、預測機制完全未動)+ scanner_lab.py 配置(robot_radius 0.2→0.3,修「規劃器允許 0.2m 但 0.3m 判碰撞」的必撞設計)
工具鏈:exp.sh(一鍵批次+存檔)、report.py(CSV→ASCII 軌跡圖,無 GUI 也能看路徑/碰撞點)、sim_offline.py(免 ROS 離線預篩,秒級)、verify_frame.py(感測手性驗證)
記錄:EXPERIMENTS.md(含 TL;DR、每輪機制分析、重現指令)
兩件需要你知道的事
repo 根目錄的 scanner.py 還帶著同一個 LiDAR 鏡像 bug — 在我的行動範圍外,沒動。建議把 flip_y 修正搬過去(一行呼叫參數 + dwa_core 的小改)。
驗證中遇到一次模擬器 flake(iter5b:gz real-time factor 崩潰,起飛爬升花 92 秒,5 runs 原地 timeout — 即使冻結也 0 碰撞):WSL2 反覆重啟 gz 偶發,健康批次的判別很簡單 — TAKEOFF 超過 30 秒就重啟 stack。