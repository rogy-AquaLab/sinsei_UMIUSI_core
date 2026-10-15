# 実機（Pi）で ROS と libcamera を使えるようにする。ssh の非対話シェル（~/.bashrc が読まれない）からでも同じにするため、
# 道具はみなこれを source する。
#   UMIUSI_WS      ワークスペース（既定 ~/ros2-ws）
#   UMIUSI_OVERLAY 上に重ねる試験用のワークスペース（任意。例: 未マージの core を別に build したもの）
#   UMIUSI_ORTLIB  onnxruntime を pip --target で入れた場所（既定 ~/ortlib、無ければ使わない）
source /opt/ros/jazzy/setup.bash
# DDS を Pi の中に閉じる。ネットワークの口が増減しても（有線のアドレスが変わる、AP を立てる）ノード間の通信が壊れない。
# UI は rosbridge（websocket）経由なので影響しない。全ノードで揃えること（~/.bashrc にも同じ行を入れる）
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# libcamera を /usr/local に入れている（Pi の ~/.bashrc と同じ）
export LIBCAMERA_IPA_PROXY_PATH=/usr/local/libexec/libcamera
export LIBCAMERA_IPA_CONFIG_PATH=/usr/local/share/libcamera/ipa
export LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:/usr/local/lib:$LD_LIBRARY_PATH
export GST_PLUGIN_PATH=/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0:$GST_PLUGIN_PATH
UMIUSI_WS="${UMIUSI_WS:-$HOME/ros2-ws}"
source "$UMIUSI_WS/install/local_setup.bash"
if [ -n "$UMIUSI_OVERLAY" ]; then source "$UMIUSI_OVERLAY/install/local_setup.bash"; fi
if [ -d "${UMIUSI_ORTLIB:-$HOME/ortlib}" ]; then export PYTHONPATH="${UMIUSI_ORTLIB:-$HOME/ortlib}:$PYTHONPATH"; fi
