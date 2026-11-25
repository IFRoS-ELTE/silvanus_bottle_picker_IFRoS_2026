import os
import sys

import numpy as np
import torch
import zmq

# Grasps returned per request, best score first. Reply: TOP_K x 17 float32 per grasp
# (row-major 4x4 matrix, then the score); empty when there is none.
TOP_K = 10

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)

from contact_graspnet_pytorch.contact_grasp_estimator import GraspEstimator
from contact_graspnet_pytorch import config_utils
from contact_graspnet_pytorch.checkpoints import CheckpointIO

NUMPY_CHECKPOINT_GLOBALS = [(np.core.multiarray.scalar, 'numpy.core.multiarray.scalar'), np.dtype, np.dtypes.Float64DType, np.dtypes.Int64DType]

def main():
    print("Loading Contact-GraspNet (PyTorch)...")
    ckpt_dir = os.path.join(BASE_DIR, 'checkpoints', 'contact_graspnet')
    if not os.path.exists(os.path.join(ckpt_dir, 'checkpoints', 'model.pt')):
        print(f"ERROR: model.pt not found under {ckpt_dir}")
        sys.exit(1)

    # 3 passes need more than the 8 GB of the laptop GPU
    forward_passes = 1
    global_config = config_utils.load_config(ckpt_dir, batch_size=forward_passes)
    estimator = GraspEstimator(global_config)
    # model.pt stores a few numpy scalars next to the weights; allow only those through
    # torch's weights-only unpickler instead of disabling it
    with torch.serialization.safe_globals(NUMPY_CHECKPOINT_GLOBALS):
        CheckpointIO(checkpoint_dir=os.path.join(ckpt_dir, 'checkpoints'), model=estimator.model).load('model.pt')
    # The port never leaves training mode on its own (dropout would stay active)
    estimator.model.eval()
    print(f"Model loaded on {estimator.device}")

    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://127.0.0.1:5555")
    print("Server ready on 5555", flush=True)

    while True:
        message = socket.recv()
        try:
            # frombuffer gives a read-only view, the estimator needs a writable array
            pc = np.frombuffer(message, dtype=np.float32).reshape(-1, 3).copy()
            print(f"Received {pc.shape[0]} points. Predicting...", flush=True)

            with torch.no_grad():
                pred_grasps, scores, contact_pts, _ = estimator.predict_scene_grasps(
                    pc, pc_segments={}, local_regions=False,
                    filter_grasps=False, forward_passes=forward_passes)

            g, s = pred_grasps.get(-1), scores.get(-1)
            if g is not None and len(s) > 0:
                order = np.argsort(s)[::-1][:TOP_K]
                reply = np.concatenate([np.append(np.asarray(g[i], dtype=np.float32).ravel(), np.float32(s[i]))
                                        for i in order]).astype(np.float32)
                socket.send(reply.tobytes())
                print(f"Success. {len(s)} grasps, sent {len(order)}, best score: {s[order[0]]:.3f}", flush=True)
            else:
                socket.send(b"")
                print("No grasps found.", flush=True)
        except Exception as e:
            import traceback; traceback.print_exc()
            socket.send(b"")

if __name__ == '__main__':
    main()
