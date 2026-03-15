# ==========================================
# SLAM型配送 本番用ロジック（32x32 自動生成・PNG画像出力・信号拡張版）
# ==========================================
import os
import random
import heapq
import time
import copy
from itertools import permutations
from PIL import Image
from google.colab import drive

# Google Driveのマウント
drive.mount('/content/drive')

# ===== ディレクトリ設定 =====
BASE_DIR = "/content/drive/MyDrive"
PNG_DIR = f"{BASE_DIR}/delivery_PNG"
OUT_DIR = f"{BASE_DIR}/delivery_view"
os.makedirs(OUT_DIR, exist_ok=True)

# ===== 基本設定 =====
H, W = 32, 32
DIRS = {0: (-1, 0), 1: (0, -1), 2: (1, 0), 3: (0, 1)} # 0:北, 1:西, 2:南, 3:東

# ===== PNGマッピング =====
PNG_MAP = {
    "◆": "road.png", "＃": "zebra.png", "・": "sidewalk.png", "■": "building.png",
    "受": "delivery_center.png", "Ａ": "point_A.png", "Ｂ": "point_B.png", "Ｃ": "point_C.png",
    "ｚ": "blurry_person_zebra.png", "Ｚ": "person_zebra.png",
    "ｓ": "blurry_person_sidewalk.png", "Ｓ": "person_sidewalk.png",
    "ｎ": "nothing.png"
}

def manhattan(a, b): return abs(a[0] - b[0]) + abs(a[1] - b[1])

# ==========================================
# マップ生成・検証ロジック
# ==========================================
def generate_map():
    while True:
        grid = [["■" for _ in range(W)] for _ in range(H)]
        signal_dict = {} # 信号の位置と種類を管理
        
        # 1. 貫通道路の生成 (幅4)
        r0 = random.randint(4, 8)
        r1 = random.randint(20, 24)
        c0 = random.randint(4, 8)
        c1 = random.randint(20, 24)
        rows = [r0, r1]
        cols = [c0, c1]

        for r in rows:
            for i in range(4):
                for c in range(W): grid[r+i][c] = "◆"
        for c in cols:
            for r in range(H):
                for i in range(4): grid[r][c+i] = "◆"

        # 2. 歩道 (幅2)
        for r in rows:
            for c in range(W):
                if grid[r-1][c] == "■": grid[r-1][c] = grid[r-2][c] = "・"
                if r+4 < H and grid[r+4][c] == "■": grid[r+4][c] = grid[r+5][c] = "・"
        for c in cols:
            for r in range(H):
                if grid[r][c-1] == "■": grid[r][c-1] = grid[r][c-2] = "・"
                if c+4 < W and grid[r][c+4] == "■": grid[r][c+4] = grid[r][c+5] = "・"

        # 3. 横断歩道と信号の設置
        for r in rows:
            for c in cols:
                # 横断歩道
                for i in range(4):
                    for j in range(2):
                        grid[r+i][c-4+j] = "＃" # 左
                        grid[r+i][c+4+j] = "＃" # 右
                        grid[r-4+j][c+i] = "＃" # 上
                        grid[r+4+j][c+i] = "＃" # 下

                # 信号 ①
                sig1_pos = [(r-1, c-3), (r+4, c-4), (r-1, c+5), (r+4, c+4)] # 座標微調整
                for pr, pc in sig1_pos:
                    if 0<=pr<H and 0<=pc<W: 
                        grid[pr][pc] = "①"
                        signal_dict[(pr, pc)] = 1
                
                # 信号 ②
                sig2_pos = [(r-4, c-1), (r-3, c+4), (r+5, c-1), (r+6, c+4)] # 座標微調整
                for pr, pc in sig2_pos:
                    if 0<=pr<H and 0<=pc<W: 
                        grid[pr][pc] = "②"
                        signal_dict[(pr, pc)] = 2

        # 4. 配送所・目的地の配置 (歩道に接する建物を探す)
        b_cands = []
        for r in range(H):
            for c in range(W):
                if grid[r][c] == "■":
                    if any(0<=r+d[0]<H and 0<=c+d[1]<W and grid[r+d[0]][c+d[1]] == "・" for d in DIRS.values()):
                        b_cands.append((r, c))
        
        if len(b_cands) < 4: continue

        random.shuffle(b_cands)
        points = {"受": b_cands[0], "Ａ": b_cands[1], "Ｂ": b_cands[2], "Ｃ": b_cands[3]}
        for mark, pos in points.items(): grid[pos[0]][pos[1]] = mark

        # 5. A*で到達可能か検証
        if all(astar(points["受"], points[p], grid) for p in ["Ａ", "Ｂ", "Ｃ"]):
            return grid, points, signal_dict

def astar(start, goal, map_):
    pq = [(0, start)]
    cost = {start: 0}
    prev = {}
    while pq:
        _, cur = heapq.heappop(pq)
        if cur == goal: break
        for d in DIRS.values():
            nr, nc = cur[0] + d[0], cur[1] + d[1]
            if not (0 <= nr < H and 0 <= nc < W): continue
            # 進入不可オブジェクト判定 (s,zはA*では未検知のため通れるとみなす)
            if map_[nr][nc] in ["■", "◆", "Ｓ", "Ｚ"]: continue
            if map_[nr][nc] in ["Ａ", "Ｂ", "Ｃ", "受"] and (nr, nc) != goal: continue
            
            ncst = cost[cur] + 1
            if (nr, nc) not in cost or ncst < cost[(nr, nc)]:
                cost[(nr, nc)] = ncst
                heapq.heappush(pq, (ncst + manhattan((nr, nc), goal), (nr, nc)))
                prev[(nr, nc)] = cur
                
    if start == goal: return [start]
    if goal not in prev: return []
    path = [goal]
    while path[-1] != start: path.append(prev[path[-1]])
    return path[::-1]

def place_people(grid, points):
    cells = [(r, c) for r in range(H) for c in range(W) if grid[r][c] in ["・", "＃"]]
    h_max = max(0, (len(cells) // 4) - 4)
    if h_max <= 0: return 0

    h_total = random.randint(1, h_max)
    banned = set(points.values())
    random.shuffle(cells)
    
    placed = 0
    for r, c in cells:
        if any(manhattan((r, c), b) <= 2 for b in banned): continue
        grid[r][c] = "ｓ" if grid[r][c] == "・" else "ｚ"
        banned.add((r, c))
        placed += 1
        if placed >= h_total: break
        
    return placed

# ==========================================
# PNG・信号ロジック
# ==========================================
def get_signal_state(step, sig_type):
    # ①と②は25ステップのオフセット
    offset = 0 if sig_type == 1 else 25
    t = (step + offset) % 50
    if t < 20: return "blue"
    if t < 30: return "yellow"
    return "red"

def render_7x7(grid, agent_pos, agent_dir, step):
    canvas = Image.new('RGBA', (700, 700), (0, 0, 0, 0))
    ar, ac = agent_pos

    for j in range(-3, 4):
        for i in range(-3, 4):
            r, c = ar + j, ac + i
            
            # マップ外判定
            if not (0 <= r < H and 0 <= c < W):
                ch = "ｎ"
            else:
                ch = grid[r][c]

            # 画像パスの決定
            if ch == "①":
                img_name = f"signal_1_{get_signal_state(step, 1)}.png"
            elif ch == "②":
                img_name = f"signal_2_{get_signal_state(step, 2)}.png"
            else:
                img_name = PNG_MAP.get(ch, "nothing.png")

            img_path = os.path.join(PNG_DIR, img_name)
            
            try:
                img = Image.open(img_path).convert("RGBA")
                px, py = (i + 3) * 100, (j + 3) * 100
                canvas.paste(img, (px, py), img)
            except:
                pass # 画像が欠損している場合はスキップ
            
            # エージェントの描画 (中心)
            if j == 0 and i == 0:
                agent_pngs = {0: "robot_N.png", 1: "robot_W.png", 2: "robot_S.png", 3: "robot_E.png"}
                try:
                    agent_img = Image.open(os.path.join(PNG_DIR, agent_pngs[agent_dir])).convert("RGBA")
                    canvas.paste(agent_img, (px, py), agent_img)
                except:
                    pass

    out_path = os.path.join(OUT_DIR, f"step_{step:03d}.png")
    canvas.save(out_path)

# ==========================================
# メイン実行ブロック
# ==========================================
print("🗺️ マップを自動生成しています...")
grid, points, signals = generate_map()
h_count = place_people(grid, points)
print(f"👥 人の配置数: {h_count}人")

# 配送順序決定 (TSP)
ITEMS = {"ダンベル": 20.0, "机": 5.0, "PC部品": 1.0, "ポケモンカード": 0.0001}
WEIGHTS = {p: random.choice(list(ITEMS.values())) for p in ["Ａ", "Ｂ", "Ｃ"]}
DIST = {}
PTS = ["受", "Ａ", "Ｂ", "Ｃ"]
for a in PTS:
    for b in PTS:
        if a == b: continue
        DIST[(a, b)] = len(astar(points[a], points[b], grid)) - 1

def total_cost(order):
    remain = sum(WEIGHTS[x] for x in order)
    cost, cur = 0, "受"
    for nxt in order:
        cost += DIST[(cur, nxt)] * remain
        remain -= WEIGHTS[nxt]
        cur = nxt
    return cost

best_order = min(permutations(["Ａ", "Ｂ", "Ｃ"]), key=total_cost)
delivery_queue = list(best_order) + ["受"]
print("📦 配送順序:", " → ".join(delivery_queue))

# シミュレーションループ
agent = points["受"]
agent_dir = 0
mode = "ROTATE"
crossing = False
goal = points[delivery_queue[0]]

print("\n🚀 シミュレーション開始...")
for step in range(150): # テスト用に制限
    
    # 7x7のPNG出力
    render_7x7(grid, agent, agent_dir, step)

    if agent == goal:
        print(f"✅ STEP {step}: 到達 {delivery_queue[0]}")
        delivery_queue.pop(0)
        if not delivery_queue:
            print("🎉 全ての配送が完了しました！")
            break
        goal = points[delivery_queue[0]]
        continue

    path = astar(agent, goal, grid)
    if len(path) < 2:
        print(f"❌ STEP {step}: 経路が閉塞しました")
        break

    nr, nc = path[1]
    dr, dc = nr - agent[0], nc - agent[1]
    next_dir = [k for k, v in DIRS.items() if v == (dr, dc)][0]

    if mode == "ROTATE":
        if agent_dir != next_dir:
            agent_dir = next_dir
            mode = "MOVE"
            continue
        mode = "MOVE"

    if mode == "MOVE":
        next_cell = grid[nr][nc]
        
        # 人の回避 (1マス先確認)
        if next_cell in ["ｓ", "ｚ"]:
            grid[nr][nc] = "Ｓ" if next_cell == "ｓ" else "Ｚ"
            print(f"👀 STEP {step}: 人を発見 → 停止してマップ更新")
            continue

        # 信号活用ルールの処理
        if next_cell == "＃" and not crossing:
            # 近隣の信号を探す (マンハッタン距離4以内)
            nearest_sig_type = None
            min_dist = 999
            for (sr, sc), s_type in signals.items():
                d = manhattan(agent, (sr, sc))
                if d <= 4 and d < min_dist:
                    min_dist = d
                    nearest_sig_type = s_type
            
            if nearest_sig_type:
                sig_color = get_signal_state(step, nearest_sig_type)
                # 歩道上で、赤または黄なら停止
                if sig_color in ["red", "yellow"]:
                    # ※プリント出力を絞る場合は以下の行をコメントアウトしてください
                    # print(f"🚦 STEP {step}: 信号({sig_color})のため停止中") 
                    continue
            crossing = True # 青なら横断開始

        if crossing and grid[agent[0]][agent[1]] == "＃" and next_cell != "＃":
            crossing = False

        agent = (nr, nc)
        mode = "ROTATE"

print(f"📸 実行終了。{OUT_DIR} に画像が出力されました。")
