#!/usr/bin/env bash
set -euo pipefail

# ========== 設定 ==========
# 対象とする MU (2.5のみ)
MU_TARGET="2.5"

# 時間パラメータ
T0=0
T1=4000
SAMPLE_TS=0
TIME_STEP=40
HALF_STEP=$(( TIME_STEP / 2 ))

# Python / Script path
PYTHON_BIN="${PYTHON_BIN:-python3}"
CONV_SCRIPT="/home/tomoya/Work/GitHub/OralAirFlowVis/rhoPimpleFoam_InSituVis/for_4dgs/new_convert_to_visgaussian.py"

# ディレクトリ設定
# 元データ（ここを走査します）
RAW_BASE="/data1/Data/VisGaussian/OralAirFlow/raw"
# 出力先
PROC_BASE="/data2/tomoya/VisGaussian/OralAirFlow/processed"

# ==========================
# 事前チェック
[ -f "$CONV_SCRIPT" ] || { echo "[ERR] CONV_SCRIPT not found: $CONV_SCRIPT" >&2; exit 1; }
[ -d "$RAW_BASE" ]    || { echo "[ERR] RAW_BASE not found: $RAW_BASE" >&2; exit 1; }

echo "[INFO] Scanning RAW_BASE: $RAW_BASE"
echo "[INFO] Target pattern: mu${MU_TARGET}_*"

# ==========================
# RAWディレクトリの検索
# ==========================
# mu2.5_xxxx という名前のディレクトリを直接リストアップ
shopt -s nullglob
TARGET_DIRS=( "${RAW_BASE}/mu${MU_TARGET}_"* )
shopt -u nullglob

if [ ${#TARGET_DIRS[@]} -eq 0 ]; then
  echo "[ERR] No directories found in $RAW_BASE matching mu${MU_TARGET}_*" >&2
  exit 1
fi

# ==========================
# 処理ループ
# ==========================
for raw_dir in "${TARGET_DIRS[@]}"; do
  # raw_dir = .../raw/mu2.5_1201
  NAME="$(basename "$raw_dir")"  # mu2.5_1201

  echo ""
  echo "======================================================="
  echo "[TARGET] $NAME"
  echo "  - Src: $raw_dir"
  echo "======================================================="

  # SRC_ROOT は raw_dir そのもの
  SRC_ROOT="$raw_dir"

  # ==========================
  # (C) CONV (processed 生成)
  # ==========================

  ## ---- (1) 通常モード ----
  OUT_DIR_NORMAL="${PROC_BASE}/${NAME}_${T0}to${T1}"
  echo "  [NORMAL] Output: $OUT_DIR_NORMAL"

  "$PYTHON_BIN" "$CONV_SCRIPT" \
    --src-root "$SRC_ROOT" \
    --out-dir  "$OUT_DIR_NORMAL" \
    --t0 "$T0" --t1 "$T1" \
    --sample-ts "$SAMPLE_TS" \
    --mu "$MU_TARGET" \
    --time-step "$TIME_STEP"

  ## ---- (2) eval_mode ----
  OUT_DIR_EVAL="${PROC_BASE}/${NAME}_sp${HALF_STEP}_${T0}to${T1}"
  echo "  [EVAL]   Output: $OUT_DIR_EVAL"

  "$PYTHON_BIN" "$CONV_SCRIPT" \
    --src-root "$SRC_ROOT" \
    --out-dir  "$OUT_DIR_EVAL" \
    --t0 "$T0" --t1 "$T1" \
    --sample-ts "$SAMPLE_TS" \
    --mu "$MU_TARGET" \
    --time-step "$TIME_STEP" \
    --eval_mode

  echo "  [OK] done $NAME"
done

echo ""
echo "[OK] Processed generation finished for mu${MU_TARGET}."