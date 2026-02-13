#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
For each mu directory under BASE_ROOT:
  - read isosurf_*_rank*.json from muX/params
  - merge per timestep -> isosurf_merged_{ts}.json
  - convert to PLY -> isosurf_merged_{ts}.ply
  - write all outputs to muX/params_merge

Usage:
  python merge_params_to_ply_all_mu.py \
      --base-root /home/tomoya/Work/Data/tomoya/CUBE
"""

import os, re, glob, json, argparse
from collections import defaultdict
from typing import List, Dict
import numpy as np
from plyfile import PlyData, PlyElement


# ----------------------
# データの不整合を補正する関数 (新規追加)
# ----------------------
def stretch_attribute(source_arr, target_len, name="attr"):
    """
    source_arr を target_len の長さになるように引き伸ばす（または縮める）。
    少ない場合は 'np.repeat' を使ってクラスタ状（ブロック状）に複製する。
    """
    source_arr = np.array(source_arr).flatten() # 一度1次元にならす
    current_len = source_arr.size
    
    if current_len == target_len:
        return source_arr
    
    # 全くデータがない場合はゼロ埋め
    if current_len == 0:
        # print(f"[Fix] {name} is empty. Padding with zeros.")
        return np.zeros(target_len, dtype=source_arr.dtype)

    # データが多い場合は切り捨て
    if current_len > target_len:
        # print(f"[Fix] {name} too long ({current_len} > {target_len}). Trimming.")
        return source_arr[:target_len]

    # データが少ない場合: クラスタ状に引き伸ばす
    # 例: current=24, target=6000 -> 1要素あたり250回繰り返す
    # print(f"[Fix] {name} mismatch ({current_len} vs {target_len}). Stretching clusters.")
    
    repeat_factor = int(np.ceil(target_len / float(current_len)))
    # repeatを使うと [A, B] -> [A, A, ..., B, B, ...] となる（これが要望のクラスタ分け）
    stretched = np.repeat(source_arr, repeat_factor)
    
    return stretched[:target_len]


# ----------------------
# メインロジック
# ----------------------

def parse_time_step_from_filename(filename: str):
    m = re.search(r'isosurf_(\d{6})_rank\d{3}\.json', filename)
    return m.group(1) if m else None

def load_json(filepath: str):
    with open(filepath, "r") as f:
        return json.load(f)

def merge_json_group(files: List[str]) -> Dict:
    merged = {"time_step": None, "coords": [], "colors": [], "normals": []}
    for path in sorted(files):
        data = load_json(path)
        if merged["time_step"] is None:
            merged["time_step"] = data.get("time_step")
        merged["coords"].extend(data.get("coords", []))
        merged["colors"].extend(data.get("colors", []))
        merged["normals"].extend(data.get("normals", []))
    return merged

def merge_all(input_dir: str, output_dir: str):
    os.makedirs(output_dir, exist_ok=True)
    files = glob.glob(os.path.join(input_dir, "isosurf_*_rank*.json"))
    timestep_map = defaultdict(list)

    for f in files:
        ts = parse_time_step_from_filename(os.path.basename(f))
        if not ts:
            continue
        out_path = os.path.join(output_dir, f"isosurf_merged_{ts}.json")
        if os.path.exists(out_path):
            continue
        timestep_map[ts].append(f)

    for ts, group in timestep_map.items():
        merged = merge_json_group(group)
        out_path = os.path.join(output_dir, f"isosurf_merged_{ts}.json")
        with open(out_path, "w") as out:
            json.dump(merged, out, indent=2)
        print(f"[✓] Wrote merged file timestep {ts}: {out_path}")

def store_ply_from_json(json_path: str, ply_path: str, skip_if_exists: bool = True):
    if skip_if_exists and os.path.exists(ply_path):
        print(f"[→] Skip (already exists): {ply_path}")
        return 0

    with open(json_path, "r") as f:
        data = json.load(f)

    # 1. Coords をマスターとする
    raw_coords = np.array(data.get("coords", []), dtype=np.float32).flatten()
    
    # 座標がXYZの3の倍数でない場合、端数を切り捨てる安全策
    if raw_coords.size % 3 != 0:
        raw_coords = raw_coords[: -(raw_coords.size % 3)]
    
    n_floats = raw_coords.size
    n_verts = n_floats // 3
    
    if n_verts == 0:
        # 空データの生成
        vertex_element = PlyElement.describe(np.empty(0, dtype=[('x','f4')]), 'vertex')
        PlyData([vertex_element], text=True).write(ply_path)
        return 0

    # 2. Colors と Normals を Coords の長さに強制的に合わせる
    raw_colors = data.get("colors", [])
    raw_normals = data.get("normals", [])

    # Coordsと同じ要素数(n_floats)になるように引き伸ばす
    # colorsが24個だけ→n_floats(6213個)になるようにブロックリピートされる
    coords  = raw_coords.reshape(-1, 3)
    colors  = stretch_attribute(raw_colors, n_floats, "colors").reshape(-1, 3)
    normals = stretch_attribute(raw_normals, n_floats, "normals").reshape(-1, 3)

    # 3. 色の正規化 (0-1 -> 0-255)
    if colors.size > 0 and colors.max() <= 1.0:
        colors = (colors * 255).clip(0, 255)
    colors = colors.astype(np.uint8)

    # 4. PLY要素の作成
    dtype = [
        ('x','f4'),('y','f4'),('z','f4'),
        ('nx','f4'),('ny','f4'),('nz','f4'),
        ('red','u1'),('green','u1'),('blue','u1')
    ]
    
    attributes = np.concatenate([coords, normals, colors], axis=1)
    
    # タプルに変換して構造化配列に入れる
    elements = np.empty(n_verts, dtype=dtype)
    elements[:] = list(map(tuple, attributes))

    vertex_element = PlyElement.describe(elements, 'vertex')
    ply_data = PlyData([vertex_element], text=True)
    ply_data.write(ply_path)
    
    print(f"[✓] PLY written: {ply_path} (verts={n_verts})")
    return n_verts


# ----------------------
# muごとに回す部分
# ----------------------

def process_one_mu(mu_dir: str, params_name="params", out_name="params_merge"):
    input_dir = os.path.join(mu_dir, params_name)
    output_dir = os.path.join(mu_dir, out_name)

    if not os.path.isdir(input_dir):
        print(f"[WARN] params not found, skip: {input_dir}")

    os.makedirs(output_dir, exist_ok=True)

    print("")
    print("====================================================")
    print(f"[MU] {os.path.basename(mu_dir)}")
    print("====================================================")

    # 1) merge json
    merge_all(input_dir, output_dir)

    # 2) merged json -> ply
    merged_jsons = sorted(glob.glob(os.path.join(output_dir, "isosurf_merged_*.json")))
    if not merged_jsons:
        print(f"[WARN] no merged json found in {output_dir}")
        return

    max_coord_num = 0
    max_ts_ply = None

    for jp in merged_jsons:
        ply_path = os.path.splitext(jp)[0] + ".ply"
        try:
            n = store_ply_from_json(jp, ply_path)
            if n > max_coord_num:
                max_coord_num = n
                max_ts_ply = ply_path
        except Exception as e:
            import traceback
            traceback.print_exc()
            print(f"[×] Failed ply convert {jp}: {e}")

    print(f"[i] max_coord_timestep ply: {max_ts_ply} (n={max_coord_num})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base-root", required=True,
                    help="mu* が並ぶルート。例: /home/tomoya/Work/Data/tomoya/CUBE")
    ap.add_argument("--mu-manual", default="",
                    help="空なら全mu。例: 0.0 を指定すると mu0.0 のみ。")
    args = ap.parse_args()

    base_root = args.base_root

    if not os.path.isdir(base_root):
        raise RuntimeError(f"base-root not found: {base_root}")

    # mu dirs listup
    mu_dirs = []
    if args.mu_manual:
        cand = os.path.join(base_root, f"mu{args.mu_manual}")
        if not os.path.isdir(cand):
            raise RuntimeError(f"manual mu dir not found: {cand}")
        mu_dirs = [cand]
    else:
        mu_dirs = [d for d in glob.glob(os.path.join(base_root, "mu*")) if os.path.isdir(d)]

    if not mu_dirs:
        raise RuntimeError(f"no mu dirs under {base_root}")

    print("[i] detected mu dirs:")
    for d in mu_dirs:
        print("  -", d)

    for mu_dir in mu_dirs:
        process_one_mu(mu_dir)

    print("")
    print("[OK] all done.")


if __name__ == "__main__":
    main()