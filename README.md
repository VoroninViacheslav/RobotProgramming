# Управление перевёрнутым маятником в MuJoCo

**Автор:** Воронин Вячеслав 312440

## Цель проекта

Разработать и сравнить системы стабилизации для обратного маятника с одним звеном на тележке в среде симуляции MuJoCo.

## Задачи

- Моделирование динамики системы в MuJoCo
- Реализация алгоритмов управления для системы (LQR, PID)
- Сравнение устойчивости и эффективности разных подходов

## Технологии

- Python 3.8+
- MuJoCo 3.0+
- NumPy

## Структура проекта

```
RobotProgramming/
├── src/                   # Исходный код
│   ├── models/            # MuJoCo XML модели
│   ├── controllers/       # Алгоритмы управления
│   ├── simulators/        # Обёртки для симуляции
│   ├── utils/             # Вспомогательные функции
│   └── main.py            # Точка входа
├── tests/                 # Тесты
├── Dockerfile             # Контейнеризация
├── Makefile               # Команды для запуска
├── requirements.txt       # Зависимости
└── README.md              # Документация
```

## Запуск

### С использованием Makefile (рекомендуется)

Установка зависимостей:
```bash
make install
```

Запуск симуляции с визуализацией:
```bash
make run
```

Запуск симуляции без GUI:
```bash
make run-headless
```

Просмотр всех доступных команд:
```bash
make help
```

### Без Makefile

Установка зависимостей:
```bash
pip install -r requirements.txt
```

Запуск симуляции с визуализацией (улучшенные параметры по умолчанию):
```bash
python src/main.py
```

Запуск симуляции без GUI:
```bash
python src/main.py --headless
```

Настройка PID-параметров:
```bash
python src/main.py --kp-angle 100 --kd-angle 20 --ki-angle 1 --kp-pos 5 --kd-pos 1
```
Запуск системы с LQR-регулятором:
```bash
python src/main.py --controller lqr --duration 5 --initial-offset 0.1
```
