# ==========================================
# delivery_view PNG → GIF
# ==========================================

from google.colab import drive
drive.mount('/content/drive')

import os
from PIL import Image

# フォルダ
VIEW_DIR = "/content/drive/MyDrive/delivery_view"

# PNG取得（名前順）
files = sorted([f for f in os.listdir(VIEW_DIR) if f.endswith(".png")])

frames = []

for f in files:
    path = os.path.join(VIEW_DIR, f)
    img = Image.open(path)
    frames.append(img)

# GIF保存
gif_path = os.path.join(VIEW_DIR, "delivery_animation.gif")

frames[0].save(
    gif_path,
    save_all=True,
    append_images=frames[1:],
    duration=200,   # 1コマ0.2秒
    loop=0
)

print("GIF作成完了")
print(gif_path)