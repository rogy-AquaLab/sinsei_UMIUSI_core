#!/bin/bash
# 前カメラのホワイトバランスを候補ごとに数秒ずつ撮り、色の数字と 1 枚の並べ画像を出す。control を止めてから（カメラを取り合う）。
#   awb_probe.sh                    既定の候補（auto、手のゲイン 4 つ（2.7,1.45 が control の既定）、cloudy）
#   awb_probe.sh 2.5,1.2 3.5,1.0    手のゲイン（赤,青）だけ試す
# 選んだら: AWB=<モード> control.sh か GAINS=<赤,青> control.sh
# 目安: 赤の平均が 0 近くなら赤い風船は黒くなる（10/17 の自動は赤 0.5・青 248 のコマが大半）。白い船体が白く見えるもの
OUT=/tmp/awb_probe/$(date +%H%M%S)
mkdir -p "$OUT"
FOCUS="af-mode=manual lens-position=${LENS:-1.0}"
if [ $# -gt 0 ]; then CANDS=("$@"); else CANDS=(auto 2.7,1.45 2.3,1.6 3.0,1.3 3.5,1.15 cloudy); fi
for c in "${CANDS[@]}"; do
  case "$c" in
    *,*) OPT="awb-enable=false colour-gains=<$c>" ;;
    *) OPT="awb-mode=$c" ;;
  esac
  echo "== $c ($OPT)"
  mkdir -p "$OUT/$c"
  # 最初の 2 秒は自動の露出・色が落ち着くまで。最後のコマを使う
  timeout -s INT 5 gst-launch-1.0 -e libcamerasrc $FOCUS $OPT ! video/x-raw,width=1280,height=720 ! \
    videoscale ! video/x-raw,width=320,height=240 ! videoconvert ! jpegenc ! \
    multifilesink location="$OUT/$c/%03d.jpg" > "$OUT/$c.log" 2>&1
  ls "$OUT/$c" | tail -1 | grep -q jpg || { echo "   撮れない（$OUT/$c.log）"; tail -3 "$OUT/$c.log"; }
done
python3 - "$OUT" "${CANDS[@]}" <<'PY'
import sys, glob, cv2, numpy as np
out, cands = sys.argv[1], sys.argv[2:]
tiles = []
for c in cands:
    fs = sorted(glob.glob(f'{out}/{c}/*.jpg'))
    if not fs:
        continue
    im = cv2.imread(fs[-1])
    b, g, r = im.reshape(-1, 3).mean(0)
    dead = (im[:, :, 2] < 10).mean()
    sat = (im[:, :, 0] > 250).mean()
    print(f'{c:>10}  赤 {r:5.1f}  緑 {g:5.1f}  青 {b:5.1f}  赤ほぼ0の割合 {dead:.2f}  青が飽和 {sat:.2f}')
    cv2.putText(im, c, (5, 20), 0, 0.6, (255, 255, 255), 2)
    tiles.append(im)
if tiles:
    while len(tiles) % 4:
        tiles.append(np.zeros_like(tiles[0]))
    sheet = np.vstack([np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles), 4)])
    cv2.imwrite(f'{out}/sheet.jpg', sheet)
    print(f'並べ画像: {out}/sheet.jpg')
PY
