import zmq

# Grasps returned per request, best score first. Reply: TOP_K x 17 float32 per grasp
# (row-major 4x4 matrix, then the score); empty when there is none.
TOP_K = 10
import numpy as np
import tensorflow as tf
tf.compat.v1.disable_eager_execution()
import sys
import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)
# NVlabs Contact-GraspNet relies on its subdirectories being directly in the Python path
sys.path.append(os.path.join(BASE_DIR, 'contact_graspnet'))
sys.path.append(os.path.join(BASE_DIR, 'utils'))
sys.path.append(os.path.join(BASE_DIR, 'pointnet2'))
sys.path.append(os.path.join(BASE_DIR, 'pointnet2', 'utils'))

from contact_graspnet.contact_grasp_estimator import GraspEstimator
from contact_graspnet import config_utils

def main():
    print("Loading Contact-GraspNet (TensorFlow)...")
    checkpoint_dir = os.path.join(BASE_DIR, 'checkpoints', 'scene_test_2048_bs3_hor_sigma_001')
    if not os.path.exists(checkpoint_dir):
        print(f"ERROR: Checkpoint directory not found at {checkpoint_dir}")
        sys.exit(1)

    forward_passes = 3
    global_config = config_utils.load_config(checkpoint_dir, batch_size=forward_passes)
    global_config['MODEL']['model'] = 'contact_graspnet.contact_graspnet'
    estimator = GraspEstimator(global_config)
    estimator.build_network()

    saver = tf.compat.v1.train.Saver(save_relative_paths=True)
    config = tf.compat.v1.ConfigProto(allow_soft_placement=True)
    config.gpu_options.allow_growth = True
    sess = tf.compat.v1.Session(config=config)                   # tf.compat.v1.Session on TF2
    estimator.load_weights(sess, saver, checkpoint_dir, mode='test')

    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://127.0.0.1:5555")
    print("Server ready on 5555", flush=True)

    while True:
        message = socket.recv()
        try:
            pc = np.frombuffer(message, dtype=np.float32).reshape(-1, 3)
            print(f"Received {pc.shape[0]} points. Predicting...", flush=True)

            pred_grasps, scores, contact_pts, _ = estimator.predict_scene_grasps(
                sess, pc, pc_segments={}, local_regions=False,
                filter_grasps=False, forward_passes=forward_passes)

            g, s = pred_grasps.get(-1), scores.get(-1)
            if g is not None and len(s) > 0:
                order = np.argsort(s)[::-1][:TOP_K]
                reply = np.concatenate([np.append(g[i].astype(np.float32).ravel(), np.float32(s[i]))
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
    # Force TF to limit GPU memory growth so it plays nice with RViz/other nodes
    physical_devices = tf.config.list_physical_devices('GPU')
    if len(physical_devices) > 0:
        tf.config.experimental.set_memory_growth(physical_devices[0], True)
        
    main()