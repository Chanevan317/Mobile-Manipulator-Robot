"""Serve a trained LeRobot policy over a local socket, for eval_sim.py (step 5). Runs in lerobot_env.

    conda activate lerobot_env
    python train_pipeline/eval/policy_server.py --checkpoint train_pipeline/outputs/<job>/checkpoints/last/pretrained_model
    ... --n_action_steps 30      execute 30 of each predicted chunk (1 s at 30 fps) before asking again (default: as trained)
    ... --ensemble 0.01          ACT temporal ensembling instead (asks every frame)

eval_sim.py starts it by itself when given --checkpoint; run it by hand only to debug.
Protocol (one client, then it exits): each message is a 4-byte big-endian length + a pickle.
    request  {'state': [7 floats], 'images': {key: (H, W, rgb bytes)}, 'task': str, 'reset': bool}
    reply    {'action': [7 floats]}
Plain Python types only: Isaac's env has NumPy 1, LeRobot's NumPy 2, and pickled arrays don't cross between them.
"""
import argparse, pickle, socket, struct

import numpy as np
import torch


def recv(conn):
    head = _exact(conn, 4)
    return None if head is None else pickle.loads(_exact(conn, struct.unpack('>I', head)[0]))


def _exact(conn, n):
    buf = b''
    while len(buf) < n:
        part = conn.recv(n - len(buf))
        if not part:
            return None
        buf += part
    return buf


def send(conn, obj):
    data = pickle.dumps(obj, protocol=4)
    conn.sendall(struct.pack('>I', len(data)) + data)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--checkpoint', required=True, help='a .../pretrained_model folder')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=9988)
    ap.add_argument('--n_action_steps', type=int, default=None)
    ap.add_argument('--ensemble', type=float, default=None, help='ACT temporal_ensemble_coeff (e.g. 0.01)')
    args = ap.parse_args()

    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.processor import PolicyProcessorPipeline
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    kw = {}
    if args.ensemble is not None:
        kw = dict(temporal_ensemble_coeff=args.ensemble, n_action_steps=1)
    elif args.n_action_steps:
        kw = dict(n_action_steps=args.n_action_steps)
    policy = ACTPolicy.from_pretrained(args.checkpoint, **kw) if kw else ACTPolicy.from_pretrained(args.checkpoint)
    policy.eval().to(device)
    pre = PolicyProcessorPipeline.from_pretrained(args.checkpoint, config_filename='policy_preprocessor.json')
    post = PolicyProcessorPipeline.from_pretrained(args.checkpoint, config_filename='policy_postprocessor.json')
    cfg = policy.config
    print(f'policy: ACT chunk {cfg.chunk_size}, executes {cfg.n_action_steps}, ensemble {cfg.temporal_ensemble_coeff}; '
          f'inputs {list(cfg.input_features)}', flush=True)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port)); srv.listen(1)
    print(f'READY {args.host}:{args.port}', flush=True)
    conn, _ = srv.accept()
    n = 0
    with torch.inference_mode():
        while True:
            req = recv(conn)
            if req is None:
                break
            if req.get('reset'):
                policy.reset()
            batch = {'observation.state': torch.tensor(req['state'], dtype=torch.float32).unsqueeze(0).to(device)}
            for key, (h, w, raw) in req['images'].items():
                im = np.frombuffer(raw, np.uint8).reshape(h, w, 3)
                batch[key] = torch.from_numpy(im.copy()).permute(2, 0, 1).float().div(255.0).unsqueeze(0).to(device)
            batch['task'] = [req.get('task', '')]
            act = policy.select_action(pre(batch))
            act = post({'action': act}) if not isinstance(act, dict) else post(act)
            act = act['action'] if isinstance(act, dict) else act
            send(conn, {'action': [float(x) for x in act.squeeze(0).float().cpu()]})
            n += 1
    print(f'client left after {n} requests', flush=True)
    conn.close(); srv.close()


if __name__ == '__main__':
    main()
