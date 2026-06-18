"""
convert_to_jit.py  —  将训练检查点 (.pt) 转换为 TorchScript JIT (.jit)

用法:
    python convert_to_jit.py --model_path <path/to/model_XXXX.pt> --task x1_dh_stand
    python convert_to_jit.py --model_path E:/Project/robot/navigation/agibot_x1_train/策略模型/model_5999.pt --task x1_dh_stand

输出文件默认保存在模型文件同目录下，名称为 <原文件名>.jit
可用 --output 参数指定输出路径。
"""

import argparse
import copy
import os
import sys

import torch
import torch.nn as nn

from humanoid.envs.x1.x1_dh_stand_config import X1DHStandCfg, X1DHStandCfgPPO
from humanoid.algo.ppo import ActorCriticDH


def class_to_dict(obj) -> dict:
    if not hasattr(obj, "__dict__"):
        return obj
    result = {}
    for key in dir(obj):
        if key.startswith("_"):
            continue
        val = getattr(obj, key)
        if isinstance(val, list):
            result[key] = [class_to_dict(item) for item in val]
        else:
            result[key] = class_to_dict(val)
    return result


def _get_cfgs(task: str):
    if task == "x1_dh_stand":
        env_cfg = X1DHStandCfg()
        train_cfg = X1DHStandCfgPPO()
        return env_cfg, train_cfg
    raise ValueError(f"Unknown task: {task}. Supported: x1_dh_stand")


# ───────────────────────────────────────────────────────────
#  ExportedDH  (与 export_policy_dh.py 完全一致)
# ───────────────────────────────────────────────────────────
class ExportedDH(torch.nn.Module):
    def __init__(self, actor, long_history, state_estimator,
                 num_short_obs: int, in_channels: int, num_proprio_obs: int):
        super().__init__()
        self.actor          = copy.deepcopy(actor).cpu()
        self.long_history   = copy.deepcopy(long_history).cpu()
        self.state_estimator = copy.deepcopy(state_estimator).cpu()
        self.num_short_obs  = num_short_obs
        self.in_channels    = in_channels
        self.num_proprio_obs = num_proprio_obs

    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        short_history = observations[..., -self.num_short_obs:]
        es_vel = self.state_estimator(short_history)
        compressed_long_history = self.long_history(
            observations.view(-1, self.in_channels, self.num_proprio_obs)
        )
        actor_obs = torch.cat((short_history, es_vel, compressed_long_history), dim=-1)
        actions_mean = self.actor(actor_obs)
        return actions_mean

    def export(self, path: str):
        self.to("cpu")
        traced = torch.jit.script(self)
        traced.save(path)


def main():
    parser = argparse.ArgumentParser(description="将 .pt 检查点转换为 TorchScript JIT")
    parser.add_argument("--model_path", type=str, required=True,
                        help="训练检查点路径，例如 策略模型/model_5999.pt")
    parser.add_argument("--task", type=str, default="x1_dh_stand",
                        help="任务名称（用于读取 env/train 配置），默认 x1_dh_stand")
    parser.add_argument("--output", type=str, default=None,
                        help="输出 .jit 文件路径，默认与输入文件同目录")
    args = parser.parse_args()

    # ── 1. 加载任务配置 ──────────────────────────────────────
    env_cfg, train_cfg = _get_cfgs(args.task)
    train_cfg_dict = class_to_dict(train_cfg)
    policy_cfg     = train_cfg_dict["policy"]

    num_short_obs   = env_cfg.env.short_frame_stack * env_cfg.env.num_single_obs
    num_proprio_obs = env_cfg.env.num_single_obs
    in_channels     = policy_cfg["in_channels"]   # == frame_stack

    num_critic_obs = env_cfg.env.num_privileged_obs
    if env_cfg.terrain.measure_heights:
        num_critic_obs = env_cfg.env.c_frame_stack * (
            env_cfg.env.single_num_privileged_obs + env_cfg.terrain.num_height
        )

    print(f"[配置] task           = {args.task}")
    print(f"[配置] num_short_obs  = {num_short_obs}")
    print(f"[配置] num_proprio_obs= {num_proprio_obs}")
    print(f"[配置] in_channels    = {in_channels}")
    print(f"[配置] num_actions    = {env_cfg.env.num_actions}")

    # ── 2. 构建 ActorCriticDH ────────────────────────────────
    actor_critic_class = eval(train_cfg_dict["runner"]["policy_class_name"])
    actor_critic: ActorCriticDH = actor_critic_class(
        num_short_obs,
        num_proprio_obs,
        num_critic_obs,
        env_cfg.env.num_actions,
        **policy_cfg,
    )

    # ── 3. 加载权重 ──────────────────────────────────────────
    model_path = os.path.abspath(args.model_path)
    if not os.path.isfile(model_path):
        print(f"[错误] 模型文件不存在: {model_path}")
        sys.exit(1)

    print(f"\n正在加载检查点: {model_path}")
    loaded_dict = torch.load(model_path, map_location="cpu")

    if "model_state_dict" in loaded_dict:
        state_dict = loaded_dict["model_state_dict"]
    else:
        state_dict = loaded_dict

    actor_critic.load_state_dict(state_dict)
    actor_critic.eval()
    print("[成功] 权重加载完毕")

    # ── 4. 封装并导出 JIT ────────────────────────────────────
    exported = ExportedDH(
        actor_critic.actor,
        actor_critic.long_history,
        actor_critic.state_estimator,
        num_short_obs,
        in_channels,
        num_proprio_obs,
    )

    if args.output is None:
        base = os.path.splitext(model_path)[0]
        out_path = base + ".jit"
    else:
        out_path = os.path.abspath(args.output)

    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    exported.export(out_path)
    print(f"\n[完成] JIT 模型已保存至: {out_path}")

    # ── 5. 快速验证 ──────────────────────────────────────────
    num_obs = env_cfg.env.num_observations
    test_input = torch.zeros(1, num_obs)
    jit_model = torch.jit.load(out_path)
    jit_model.eval()
    with torch.no_grad():
        out = jit_model(test_input)
    print(f"[验证] 输入维度={num_obs}, 输出维度={out.shape[-1]}  ✓")


if __name__ == "__main__":
    main()
