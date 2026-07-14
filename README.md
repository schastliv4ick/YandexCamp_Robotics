# GFSX-RL-Sim

Симуляция гусеничного робота GFS-X на Unity 6 + ML-Agents.

**Команда:** Rhymes & Codes

---

## Технологии
- Unity 6 (6000.0.79f1)
- ML-Agents 1.1.0
- Python 3.10.12 + PyTorch 2.2.2

---

## Структура проекта
```
GFSX-RL-Sim/
├── .vscode/               # Настройки редактора (VS Code)
├── Assets/                # Все ресурсы Unity
│   ├── Scripts/           # C# скрипты (TrackController, RobotBrain, сенсоры и т.д.)
│   ├── Scenes/            # Сцены (основная сцена симуляции)
│   ├── Prefabs/           # Префабы (робот, мяч, стены)
│   ├── Models/            # 3D-модели (Xiao-r GFS-X_1.fbx и др.)
│   ├── Materials/         # Материалы для объектов
│   ├── Textures/          # Текстуры (если есть)
│   └── ML-Agents/         # Импортированные примеры (опционально)
├── Packages/              # Управление пакетами Unity (manifest.json)
├── .gitignore             # Исключения для Git (Unity-шаблон)
├── README.md              # Краткое описание проекта
├── config.yaml            # Конфигурация обучения PPO (гиперпараметры)
└── [другие файлы гайдов]  # P1_Intro_And_Setup.md, P2_*, P3_*, YANDEX_CLOUD_GUIDE.md
```
---

## Быстрый старт

```bash
git clone https://github.com/ваш_логин/GFSX-RL-Sim.git