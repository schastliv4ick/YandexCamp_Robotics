"""
Анализ логов обучения GFS-X: пошаговая диагностика (П6, Шаг 3) +
кастомная метрика "в каких сценариях робот реально хватает мяч".

Использование:
    python analyze_logs.py --diagnostic diagnostic_log.csv --episodes episode_metrics.csv --out ./plots

Оба файла опциональны — скрипт построит только те графики, для которых
нашёл данные, и явно скажет, каких файлов не хватает.

Файлы, которые ожидает скрипт:
  - diagnostic_log.csv  — пошаговый лог одного локального тестового прогона
                          (пишет DiagnosticLogger.LogStep, П6 Шаг 1).
  - episode_metrics.csv — лог по эпизодам (пишет DiagnosticLogger.LogEpisodeEnd,
                          кастомная метрика; доступен только если
                          writeEpisodeCsvLocally=true и прогон был одиночным
                          локальным, не облачным с --num-envs>1).
"""

import argparse
import os
import sys

import pandas as pd
import matplotlib
matplotlib.use("Agg")  # без GUI — просто сохраняем PNG
import matplotlib.pyplot as plt


def analyze_step_log(path: str, out_dir: str) -> None:
    if not os.path.exists(path):
        print(f"[!] Пошаговый лог не найден: {path} — пропускаю графики П6 Шаг 3.")
        return

    df = pd.read_csv(path)
    print(f"[OK] Пошаговый лог: {path} — {len(df)} строк.")

    # --- 1. Динамическое торможение: gas относительно ballDist ---
    # Что ищем: по мере уменьшения ballDist значение gas должно плавно падать.
    # Если график плоский — модель не тормозит перед целью, собьёт мяч в реальности.
    seen = df[df["ballSeen"] == 1].copy()
    if len(seen) > 5:
        seen_sorted = seen.sort_values("ballDist")
        plt.figure(figsize=(7, 5))
        plt.scatter(seen_sorted["ballDist"], seen_sorted["gas"], s=8, alpha=0.4, label="сырые точки")
        # скользящее среднее по бинам дистанции — так тренд виднее, чем в облаке точек
        bins = pd.cut(seen_sorted["ballDist"], bins=15)
        binned = seen_sorted.groupby(bins, observed=True)["gas"].mean()
        bin_centers = [interval.mid for interval in binned.index]
        plt.plot(bin_centers, binned.values, color="red", linewidth=2, label="среднее по бинам")
        plt.xlabel("ballDist (0 = вплотную, 1 = далеко)")
        plt.ylabel("gas")
        plt.title("Динамическое торможение: gas(ballDist)")
        plt.legend()
        plt.grid(alpha=0.3)
        plt.tight_layout()
        plt.savefig(os.path.join(out_dir, "1_gas_vs_balldist.png"), dpi=120)
        plt.close()
        print("  -> 1_gas_vs_balldist.png")
    else:
        print("  [!] Слишком мало строк с ballSeen==1 для графика торможения.")

    # --- 2. Фаза Retry: isRetrying в моменты потери мяча вблизи ---
    plt.figure(figsize=(10, 4))
    plt.plot(df["time"], df["ballSeen"], label="ballSeen", drawstyle="steps-post", alpha=0.7)
    plt.plot(df["time"], df["isRetrying"], label="isRetrying", drawstyle="steps-post", alpha=0.7)
    plt.plot(df["time"], df["ballDist"].clip(0, 1), label="ballDist", alpha=0.5, linestyle="--")
    plt.xlabel("время, с")
    plt.title("Retry-фаза относительно видимости и дистанции мяча")
    plt.legend()
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "2_retry_trace.png"), dpi=120)
    plt.close()
    print("  -> 2_retry_trace.png")
    print("     NB: isRetrying в текущей интеграции — эвристика (не настоящая фаза")
    print("     состояния в RobotBrain), см. комментарий в коде OnActionReceived.")

    # --- 3. Мёртвая зона руления: гистограмма steering ---
    plt.figure(figsize=(7, 5))
    plt.hist(df["steering"], bins=40, color="steelblue", edgecolor="black", alpha=0.8)
    plt.axvline(-0.1, color="red", linestyle="--", label="±0.1 (примерная мёртвая зона PWM)")
    plt.axvline(0.1, color="red", linestyle="--")
    plt.xlabel("steering")
    plt.ylabel("количество шагов")
    plt.title("Распределение команд руления")
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "3_steering_histogram.png"), dpi=120)
    plt.close()
    frac_deadzone = ((df["steering"].abs() < 0.1) & (df["steering"].abs() > 0.01)).mean()
    print(f"  -> 3_steering_histogram.png (доля команд в районе мёртвой зоны: {frac_deadzone:.1%})")

    # --- 4. Траектория движения ---
    plt.figure(figsize=(6, 6))
    plt.plot(df["displacementX"], df["displacementZ"], alpha=0.7)
    plt.scatter([0], [0], color="green", label="старт", zorder=5)
    plt.xlabel("displacementX, м")
    plt.ylabel("displacementZ, м")
    plt.title("Траектория робота за прогон")
    plt.axis("equal")
    plt.legend()
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "4_trajectory.png"), dpi=120)
    plt.close()
    print("  -> 4_trajectory.png")


def analyze_episode_log(path: str, out_dir: str) -> None:
    if not os.path.exists(path):
        print(f"[!] Лог по эпизодам не найден: {path} — пропускаю анализ сценариев успеха.")
        return

    df = pd.read_csv(path)
    print(f"[OK] Лог по эпизодам: {path} — {len(df)} эпизодов.")

    total = len(df)
    success = (df["outcome"] == "success").sum()
    fall = (df["outcome"] == "fall").sum()
    timeout = (df["outcome"] == "timeout").sum()
    print(f"  Итог: success={success} ({success/total:.1%}), "
          f"fall={fall} ({fall/total:.1%}), timeout={timeout} ({timeout/total:.1%})")

    df["is_success"] = df["outcome"] == "success"

    # --- 5. Успех по сценариям: параметры DR, success vs остальное ---
    params = ["robotMass", "ballMassMultiplier", "actionLatency"]
    available = [p for p in params if p in df.columns]

    fig, axes = plt.subplots(1, len(available), figsize=(5 * len(available), 5))
    if len(available) == 1:
        axes = [axes]

    for ax, param in zip(axes, available):
        data_success = df.loc[df["is_success"], param]
        data_other = df.loc[~df["is_success"], param]
        ax.boxplot([data_success, data_other], tick_labels=["success", "fall/timeout"])
        ax.set_title(param)
        ax.grid(alpha=0.3)
        if len(data_success) > 0 and len(data_other) > 0:
            print(f"  {param}: среднее при успехе={data_success.mean():.3f}, "
                  f"при остальных исходах={data_other.mean():.3f}")

    fig.suptitle("В каких сценариях (DR-параметрах) робот реально хватает мяч")
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "5_success_by_scenario.png"), dpi=120)
    plt.close()
    print("  -> 5_success_by_scenario.png")

    # --- 6. Success rate по ходу обучения (скользящее окно) ---
    window = max(5, total // 20)
    df["success_rate_rolling"] = df["is_success"].rolling(window, min_periods=1).mean()
    plt.figure(figsize=(8, 5))
    plt.plot(df.index, df["success_rate_rolling"])
    plt.xlabel("номер эпизода")
    plt.ylabel(f"success rate (скользящее окно {window} эп.)")
    plt.title("Динамика успеха по ходу прогона")
    plt.ylim(0, 1)
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "6_success_rate_over_time.png"), dpi=120)
    plt.close()
    print("  -> 6_success_rate_over_time.png")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--diagnostic", default="diagnostic_log.csv", help="путь к diagnostic_log.csv")
    parser.add_argument("--episodes", default="episode_metrics.csv", help="путь к episode_metrics.csv")
    parser.add_argument("--out", default="./plots", help="папка для сохранения графиков")
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)

    analyze_step_log(args.diagnostic, args.out)
    print()
    analyze_episode_log(args.episodes, args.out)

    print(f"\nГотово. Графики сохранены в: {os.path.abspath(args.out)}")


if __name__ == "__main__":
    main()
