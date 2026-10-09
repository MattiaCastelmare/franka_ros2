#!/usr/bin/env bash
# Installs the RTMW hand back-end (hand_detector:=rtmw) deps in the robot
# container. Run it from the host: ./install_rtmw_deps.sh [container]
#
# - --no-deps everywhere: numpy, opencv, protobuf (mediapipe) are not touched.
# - onnxruntime -> onnxruntime-gpu, same version 1.23.2 (same module, CPU
#   provider included): the two packages cannot coexist.
# - TensorRT 10 (cu12) for the TensorRT execution provider (RTMW in FP16).
# - CUDA 12 runtime libs for onnxruntime-gpu next to torch's CUDA 13 ones
#   (different folders); cuDNN 9 is reused from torch.
# Tested on franka_labs009 (RTX 5090, torch cu130): RTMW on CUDA ~17 ms/frame.
set -euo pipefail

CONTAINER="${1:-franka_labs009}"

docker exec -i "$CONTAINER" bash -s <<'EOF'
set -euo pipefail
PIP="python3 -m pip"

if $PIP show -q onnxruntime 2>/dev/null; then
    $PIP uninstall -y onnxruntime
fi

$PIP install --user --no-cache-dir --no-deps \
    onnxruntime-gpu==1.23.2 \
    rtmlib==0.0.16 \
    nvidia-cublas-cu12==12.9.2.10 \
    nvidia-cuda-runtime-cu12==12.9.79 \
    nvidia-cufft-cu12==11.4.1.4 \
    nvidia-curand-cu12==10.3.10.19 \
    tensorrt-cu12==10.16.1.11 \
    tensorrt-cu12-libs==10.16.1.11 \
    tensorrt-cu12-bindings==10.16.1.11

# Check (same import order as RtmwHandDetector) + download the models once.
python3 - <<'PY'
import ctypes
ctypes.CDLL('libcuda.so.1').cuInit(0)
import torch  # noqa: F401
import mediapipe, numpy as np, onnxruntime as ort
from rtmlib import PoseTracker, Wholebody

m = PoseTracker(Wholebody, det_frequency=10, mode='lightweight',
                tracking=False, backend='onnxruntime', device='cuda')
m(np.zeros((480, 640, 3), np.uint8))
prov = m.det_model.session.get_providers()
print('onnxruntime', ort.__version__, prov, '| mediapipe', mediapipe.__version__,
      '| numpy', np.__version__)
assert prov[0] == 'CUDAExecutionProvider', 'RTMW is not on the GPU'
import tensorrt  # TensorRT FP16 for RTMW (rtmw_tensorrt: true)
assert 'TensorrtExecutionProvider' in ort.get_available_providers()
print('RTMW OK | tensorrt', tensorrt.__version__)
PY
EOF
