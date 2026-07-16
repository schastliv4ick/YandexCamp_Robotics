"""
Веса передаются в Unity НЕ пересборкой билда, а через блок environment_parameters
в config.yaml — их читает RobotBrain.LoadRewardConfigFromEnvironmentParameters()

ПРИМЕРЫ ЗАПУСКА:

  # быстро проверить, что конфиги генерируются правильно (без обучения):
  python tune_rewards.py --dry-run --trials 3

  # реальный перебор 20 наборов, по 200к шагов каждый, 4 арены:
  python tune_rewards.py --base-config config.yaml \
      --env Build/GFSX_Simulator.exe --num-envs 4 \
      --trials 20 --steps-per-trial 200000

Итог: печатает топ наборов, сохраняет trials.csv и config_best.yaml
"""
import argparse
import csv
import os
import random
import re
import subprocess
import sys
import time
import copy

import yaml

SEARCH_SPACE = {
    "success_reward": (2.0, 10.0, False),
    "distance_reward_near": (0.05, 0.4, True),
    "distance_reward_far": (0.01, 0.15, True),
    "centering_reward_scale": (0.0, 0.1, False),
    "action_rate_penalty": (0.0, 0.05, False),
    "obstacle_penalty_scale": (0.0, 0.2, False),
    "ir_collision_penalty": (0.0, 0.1, False),
    "backward_penalty": (0.0,  0.05, False),
    # fall_penalty // оставим фиксированными из base-config
}

def sample_params(rng: random.Random) -> dict:
    """случайный набор весов из пространства поиска"""
    params = {}
    for name, (low, high, use_log) in SEARCH_SPACE.items():
        if use_log and low > 0:
            import math
            val = math.exp(rng.uniform(math.log(low), math.log(high)))
        else:
            val = rng.uniform(low, high)
        params[name] = round(val, 5)
    return params


def load_base_config(path: str, behavior: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    if "behaviors" not in cfg or behavior not in cfg["behaviors"]:
        sys.exit(f"[!] {path} has no behaviors.{behavior}. "
                 f"Check --behavior (сейчас '{behavior}').")
    return cfg

def write_trial_config(base_cfg: dict, behavior: str, params: dict,
                       steps: int, summary_freq: int, out_path: str) -> None:
    """base-config + короткий max_steps + блок environment_parameters с весами."""
    cfg = copy.deepcopy(base_cfg)
    b = cfg["behaviors"][behavior]
    b["max_steps"] = int(steps)
    b["summary_freq"] = int(min(summary_freq, max(1000, steps // 5)))
    b["keep_checkpoints"] = 1
    # блок env-параметров: константа = один float на ключ
    cfg["environment_parameters"] = {k: float(v) for k, v in params.items()}
    with open(out_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, sort_keys=False, allow_unicode=False)
        
        
def run_training(config_path: str, run_id: str, env: str, num_envs: int,
                 base_port: int, results_dir: str, timeout_s: int,
                 log_path: str) -> str:
    """Запускает mlagents-learn, пишет stdout в файл, возвращает хвост stdout."""
    cmd = ["mlagents-learn", config_path, f"--run-id={run_id}",
           "--force", f"--results-dir={results_dir}", f"--base-port={base_port}"]
    if env:
        cmd += [f"--env={env}", f"--num-envs={num_envs}", "--no-graphics"]

    print(f"    $ {' '.join(cmd)}")
    tail = []
    with open(log_path, "w", encoding="utf-8") as log:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, bufsize=1)
        start = time.time()
        try:
            for line in proc.stdout:
                log.write(line)
                tail.append(line)
                if len(tail) > 400:
                    tail.pop(0)
                if time.time() - start > timeout_s:
                    print(f"    [!] Таймаут {timeout_s}s — останавливаю прогон.")
                    proc.terminate()
                    break
            proc.wait(timeout=30)
        except KeyboardInterrupt:
            proc.terminate()
            raise
        finally:
            if proc.poll() is None:
                proc.kill()
    return "".join(tail)


_REWARD_RE = re.compile(r"Mean Reward:\s*([-+]?\d+\.?\d*)")

def parse_mean_reward(stdout_tail: str):
    """Last 'Mean Reward: X' from output mlagents-learn (fallback-метрика)."""
    vals = _REWARD_RE.findall(stdout_tail)
    return float(vals[-1]) if vals else None


def read_success_rate(results_dir: str, run_id: str, behavior: str,
                      last_k: int = 3):
    """Average of the last values of Custom/GrabSuccessRate from TensorBoard (if available)."""
    try:
        from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    except Exception:
        return None  # tensorboard not installed —  going to fallback

    run_dir = os.path.join(results_dir, run_id)
    if not os.path.isdir(run_dir):
        return None
    # search all tfevents under the run directory (ML-Agents puts them in behavior)
    event_files = []
    for root, _, files in os.walk(run_dir):
        for fn in files:
            if "tfevents" in fn:
                event_files.append(os.path.join(root, fn))
    values = []
    for ef in event_files:
        try:
            acc = EventAccumulator(ef)
            acc.Reload()
            if "Custom/GrabSuccessRate" in acc.Tags().get("scalars", []):
                values += [e.value for e in acc.Scalars("Custom/GrabSuccessRate")]
        except Exception:
            continue
    if not values:
        return None
    return sum(values[-last_k:]) / len(values[-last_k:])

def evaluate(params, base_cfg, args, trial_idx, work_dir):
    """Trains one weight set, returns (score, metric_name, detail)"""
    run_id = f"tune_{args.study}_{trial_idx:03d}"
    cfg_path = os.path.join(work_dir, f"{run_id}.yaml")
    log_path = os.path.join(work_dir, f"{run_id}.log")
    write_trial_config(base_cfg, args.behavior, params,
                       args.steps_per_trial, args.summary_freq, cfg_path)

    if args.dry_run:
        return None, "dry-run", cfg_path

    tail = run_training(cfg_path, run_id, args.env, args.num_envs,
                        args.base_port + trial_idx, args.results_dir,
                        args.timeout, log_path)

    sr = None
    if args.objective in ("auto", "success"):
        sr = read_success_rate(args.results_dir, run_id, args.behavior)
    if sr is not None:
        return sr, "GrabSuccessRate", f"{sr:.3f}"

    if args.objective == "success":
        print("    [!] GrabSuccessRate is not found — connect LogEpisodeEnd(). "
              "Using Mean Reward.")
    mr = parse_mean_reward(tail)
    if mr is None:
        print("    [!] Failed to read metric from trial — setting to -inf.")
        return float("-inf"), "none", "no data"
    return mr, "MeanReward", f"{mr:.3f}"

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base-config", default="config.yaml",
                    help="base config.yaml (P3, Step 5)")
    ap.add_argument("--behavior", default="GFSX_Brain",
                    help="Behavior Name (as in Behavior Parameters)")
    ap.add_argument("--env", default="",
                    help="path to simulator build (required for real tuning)")
    ap.add_argument("--num-envs", type=int, default=4)
    ap.add_argument("--trials", type=int, default=20)
    ap.add_argument("--steps-per-trial", type=int, default=200000,
                    help="budget of steps per trial (less = faster, coarser)")
    ap.add_argument("--summary-freq", type=int, default=10000)
    ap.add_argument("--timeout", type=int, default=3600,
                    help="limit of seconds per trial")
    ap.add_argument("--base-port", type=int, default=5006)
    ap.add_argument("--results-dir", default="results")
    ap.add_argument("--objective", choices=["auto", "success", "reward"], default="auto")
    ap.add_argument("--study", default="rw")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--out-dir", default="./tuning")
    ap.add_argument("--dry-run", action="store_true",
                    help="just generate configs and print them, without training")
    args = ap.parse_args()

    if not args.dry_run and not args.env:
        sys.exit("[!] For real tuning you need --env (simulator build). "
                 "Or run with --dry-run to check configs.")

    os.makedirs(args.out_dir, exist_ok=True)
    base_cfg = load_base_config(args.base_config, args.behavior)
    rng = random.Random(args.seed)

    # Optuna, if installed, — smarter than random search. No — random search.
    try:
        import optuna
        optuna.logging.set_verbosity(optuna.logging.WARNING)
        use_optuna = True
    except Exception:
        use_optuna = False
        print("[i] Optuna is not installed — using random search (this also works).")

    results = []  # (score, params, metric_name, detail)

    def record(idx, params):
        score, metric, detail = evaluate(params, base_cfg, args, idx, args.out_dir)
        tag = "" if score is None else f"  -> {metric}={detail}"
        print(f"[{idx + 1}/{args.trials}] {params}{tag}")
        if score is not None:
            results.append((score, params, metric, detail))
        return score

    if args.dry_run:
        for i in range(args.trials):
            record(i, sample_params(rng))
        print(f"\n[dry-run] Configs in {os.path.abspath(args.out_dir)}. Training not started.")
        return

    if use_optuna:
        study = optuna.create_study(direction="maximize",
                                    sampler=optuna.samplers.TPESampler(seed=args.seed))

        def objective(trial):
            params = {}
            for name, (low, high, use_log) in SEARCH_SPACE.items():
                params[name] = round(trial.suggest_float(name, low, high, log=use_log), 5)
            score = record(trial.number, params)
            return score if score is not None else float("-inf")

        study.optimize(objective, n_trials=args.trials)
    else:
        for i in range(args.trials):
            record(i, sample_params(rng))

    if not results:
        sys.exit("[!] No trial yielded a metric. Check logs in out-dir.")

    results.sort(key=lambda r: r[0], reverse=True)

    print("\n================= TOP =================")
    for rank, (score, params, metric, detail) in enumerate(results[:5], 1):
        print(f"{rank}. {metric}={detail}")
        for k, v in params.items():
            print(f"     {k:24}= {v}")

    # trials.csv
    csv_path = os.path.join(args.out_dir, "trials.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["score", "metric"] + list(SEARCH_SPACE.keys()))
        for score, params, metric, _ in results:
            w.writerow([score, metric] + [params[k] for k in SEARCH_SPACE])

    # config_best.yaml — full config with max_steps and env-params
    best_score, best_params, best_metric, _ = results[0]
    best_cfg = copy.deepcopy(base_cfg)
    best_cfg["environment_parameters"] = {k: float(v) for k, v in best_params.items()}
    best_path = os.path.join(args.out_dir, "config_best.yaml")
    with open(best_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(best_cfg, f, sort_keys=False, allow_unicode=False)

    print(f"\nBest result: {best_metric}={best_score:.3f}")
    print(f"Table of trials: {os.path.abspath(csv_path)}")
    print(f"Ready config:   {os.path.abspath(best_path)}")
    print("Starting real training:")
    print(f"  mlagents-learn {best_path} --run-id=gfsx_best "
          f"--env={args.env or 'Build/GFSX_Simulator.exe'} --num-envs={args.num_envs} --no-graphics")


if __name__ == "__main__":
    main()
