#!/usr/bin/env bash
set -euo pipefail

# ========== 設定 ==========
# MU_MANUAL="${MU_MANUAL:-}"   # 空なら自動で mu* 全部。指定すればそのmuだけ。
MU_MANUAL=2.5

# 時間パラメータ（ひとまずそのまま）
T0=0
T1=4000
SAMPLE_TS=0
TIME_STEP=40
HALF_STEP=$(( TIME_STEP / 2 ))

# Python
PYTHON_BIN="${PYTHON_BIN:-python3}"

# スクリプト
POSTPROC_SCRIPT="/home/tomoya/Work/GitHub/OralAirFlowVis/rhoPimpleFoam_InSituVis/for_4dgs/merge_points_simple.py"
CONV_SCRIPT="/home/tomoya/Work/GitHub/OralAirFlowVis/rhoPimpleFoam_InSituVis/for_4dgs/new_convert_to_visgaussian.py"

# mu ルート
MU_ROOT="/data2/tomoya/OralAirFlow"

# 中間/最終
RAW_BASE="/data1/Data/VisGaussian/OralAirFlow/raw"
PROC_BASE="/data2/tomoya/VisGaussian/OralAirFlow/processed"
# ==========================

# 事前チェック
[ -f "$POSTPROC_SCRIPT" ] || { echo "[ERR] POSTPROC_SCRIPT not found: $POSTPROC_SCRIPT" >&2; exit 1; }
[ -f "$CONV_SCRIPT" ]     || { echo "[ERR] CONV_SCRIPT not found: $CONV_SCRIPT" >&2; exit 1; }
command -v "$PYTHON_BIN" >/dev/null 2>&1 || { echo "[ERR] python not found" >&2; exit 1; }
[ -d "$MU_ROOT" ]         || { echo "[ERR] MU_ROOT not found: $MU_ROOT" >&2; exit 1; }

# ==========================
# mu ディレクトリ列挙
# ==========================
MU_DIRS=()

if [ -n "$MU_MANUAL" ]; then
  cand="${MU_ROOT}/mu${MU_MANUAL}"
  if [ ! -d "$cand" ]; then
    echo "[ERR] manual mu dir not found: $cand" >&2
    exit 1
  fi
  MU_DIRS+=( "$cand" )
else
  for d in "$MU_ROOT"/mu*; do
    [ -d "$d" ] || continue
    MU_DIRS+=( "$d" )
  done
fi

if [ ${#MU_DIRS[@]} -eq 0 ]; then
  echo "[ERR] No mu dirs found under $MU_ROOT (expect mu*)" >&2
  exit 1
fi

echo "[i] detected mu dirs:"
printf '  - %s\n' "${MU_DIRS[@]}"

# ==========================
# mu ごとの外側ループ
# ==========================
for mu_dir in "${MU_DIRS[@]}"; do
  mu_base="$(basename "$mu_dir")"  # mu0.0
  MU="${mu_base#mu}"              # 0.0

  echo ""
  echo "======================================================="
  echo "[MU] $MU"
  echo "[MU_DIR] $mu_dir"
  echo "======================================================="

  # ==========================
  # (A) mu単位のPOSTPROCを先に1回だけ実行
  #     params -> params_merge (merged json + ply)
  # ==========================
  echo "[POSTPROC MU] run for MU=$MU"
  "$PYTHON_BIN" "$POSTPROC_SCRIPT" \
    --base-root "$MU_ROOT" \
    --mu-manual "$MU"

  # params_merge の存在チェック
  PARAMS_MERGE_DIR="${mu_dir}/params_merge"
  if [ ! -d "$PARAMS_MERGE_DIR" ]; then
    echo "[ERR] params_merge not found after postproc: $PARAMS_MERGE_DIR" >&2
    continue
  fi

  # ==========================
  # view_id 列挙（mu_dir 直下のサブディレクトリ）
  # └ viewpoints/ を持つものだけ
  # ==========================
  VIEW_DIRS=()
  for vd in "$mu_dir"/*; do
    [ -d "$vd" ] || continue
    [ -d "$vd/viewpoints" ] || continue   # viewpoints がある view だけ対象
    VIEW_DIRS+=( "$vd" )
  done

  if [ ${#VIEW_DIRS[@]} -eq 0 ]; then
    echo "[WARN] No valid view dirs under $mu_dir (need viewpoints/) -> skip MU=$MU" >&2
    continue
  fi

  echo "[i] detected views for MU=$MU:"
  for vd in "${VIEW_DIRS[@]}"; do
    echo "  - $(basename "$vd")"
  done

  # ==========================
  # view ごとの内側ループ
  # ==========================
  for view_dir in "${VIEW_DIRS[@]}"; do
    view_id="$(basename "$view_dir")"   # 1201 とか 142 とか
    NAME="mu${MU}_${view_id}"
    RAW_DIR="${RAW_BASE}/${NAME}"

    echo ""
    echo "[VIEW] $view_id"
    echo "  - view_dir = $view_dir"
    echo "  - NAME     = $NAME"
    echo "  - RAW_DIR  = $RAW_DIR"

    # ==========================
    # (B) あなた指定の raw 生成ルールでコピー
    # ==========================
    mkdir -p "$RAW_DIR/points" "$RAW_DIR/params" "$RAW_DIR/depth"

    # (1) mu_dir/params_merge/* -> RAW_DIR/points
    echo "  [COPY-1] params_merge -> points/"
    cp -a "${PARAMS_MERGE_DIR}/." "${RAW_DIR}/points/"

    # (2) view_dir/viewpoints/* -> RAW_DIR/params
    VIEWPOINTS_DIR="${view_dir}/viewpoints"
    if [ ! -d "$VIEWPOINTS_DIR" ]; then
      echo "  [ERR] viewpoints not found: $VIEWPOINTS_DIR" >&2
      exit 1
    fi
    echo "  [COPY-2] viewpoints -> params/"
    cp -a "${VIEWPOINTS_DIR}/." "${RAW_DIR}/params/"

    # (3)(4) Process0000 の bmp を振り分け
    PROC0_DIR="${view_dir}/Process0000"
    if [ ! -d "$PROC0_DIR" ]; then
      echo "  [ERR] Process0000 not found: $PROC0_DIR" >&2
      exit 1
    fi

    echo "  [COPY-3] Process0000/*.bmp (exclude depth) -> RAW root"
    shopt -s nullglob
    for f in "${PROC0_DIR}"/*.bmp; do
      base="$(basename "$f")"
      if [[ "$base" == *_depth_.bmp ]]; then
        continue
      fi
      cp -a "$f" "${RAW_DIR}/"
    done
    shopt -u nullglob

    echo "  [COPY-4] Process0000/*_depth_.bmp -> depth/"
    shopt -s nullglob
    for f in "${PROC0_DIR}"/*_depth_.bmp; do
      cp -a "$f" "${RAW_DIR}/depth/"
    done
    shopt -u nullglob

    # ==========================
    # (C) CONV（MultipleView 生成）
    #   src-root は RAW_DIR
    # ==========================
    SRC_ROOT="$RAW_DIR"

    ## ---- (1) 通常モード ----
    OUT_DIR_NORMAL="${PROC_BASE}/${NAME}_${T0}to${T1}"
    echo "  [NORMAL] OUT_DIR = $OUT_DIR_NORMAL"

    "$PYTHON_BIN" "$CONV_SCRIPT" \
      --src-root "$SRC_ROOT" \
      --out-dir  "$OUT_DIR_NORMAL" \
      --t0 "$T0" --t1 "$T1" \
      --sample-ts "$SAMPLE_TS" \
      --mu "$MU" \
      --time-step "$TIME_STEP"

    # ---- (2) eval_mode ----
    OUT_DIR_EVAL="${PROC_BASE}/${NAME}_sp${HALF_STEP}_${T0}to${T1}"
    echo "  [EVAL] OUT_DIR = $OUT_DIR_EVAL"

    "$PYTHON_BIN" "$CONV_SCRIPT" \
      --src-root "$SRC_ROOT" \
      --out-dir  "$OUT_DIR_EVAL" \
      --t0 "$T0" --t1 "$T1" \
      --sample-ts "$SAMPLE_TS" \
      --mu "$MU" \
      --time-step "$TIME_STEP" \
      --eval_mode

    echo "  [OK] done view=$view_id (MU=$MU)"
  done
done

echo ""
echo "[OK] all mu + views done."
