"""lerobot-train with two speed-ups LeRobot doesn't expose (used by train_act.sh; same arguments as lerobot-train).

    FAST_COMPILE=1   torch.compile the ACT model's forward (default on; 0 = off)
    FAST_BF16=1      with --policy.use_amp=true: autocast in bfloat16 instead of torch's default float16 (default on)

Measured 2026-11-20 on the RTX 5060 Ti, ACT as train_act.sh sets it up (2 cameras 640 x 400 / 640 x 360, chunk 100,
batch 16), one training step:
    float32 (LeRobot's default)    3.5 step/s     float32 + compile    4.2
    bf16 autocast                  4.5            bf16 + compile       8.2 step/s, 4.0 GB instead of 5.9
The GPU is the limit (100 % busy, the data loader keeps up); compile mostly removes the many small kernels of the
ResNet + transformer. Checkpoints are unchanged: only the forward method is compiled, so the saved weights have
their normal names and load in policy_server.py / lerobot as usual.
"""
import os

import torch

if os.environ.get('FAST_BF16', '1') == '1':
    torch.set_autocast_dtype('cuda', torch.bfloat16)      # lerobot-train reads this when use_amp is on

if os.environ.get('FAST_COMPILE', '1') == '1':
    import torch._dynamo
    # the last batch of an epoch is smaller (drop_last=False): compile it as a second fixed shape instead of
    # switching the whole model to slower shape-generic kernels
    torch._dynamo.config.automatic_dynamic_shapes = False
    from lerobot.policies.act import modeling_act as _M
    _init = _M.ACTPolicy.__init__

    def _compiled_init(self, *a, **k):
        _init(self, *a, **k)
        self.model.forward = torch.compile(self.model.forward)   # instance attribute: state_dict keys unchanged
    _M.ACTPolicy.__init__ = _compiled_init

from lerobot.scripts.lerobot_train import main  # noqa: E402

if __name__ == '__main__':
    main()
