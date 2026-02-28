define warehouse_lidar ranger
(
  sensor (
    range [ 0.12 3.5 ]           # мин/макс дальность (м) — уменьшена для склада
    fov 360.0                     # угол обзора (градусы)
    samples 180                   # количество лучей (180 вместо 360 для производительности)
  )
  size [ 0.05 0.05 0.05 ]
  color "blue"
)

define warehouse_robot position
(
  # Размер робота (x y z) в метрах
  size [ 0.30 0.30 0.15 ]
  
  # Начальная позиция (будет переопределена при спавне)
  origin [ 0.0 0.0 0.0 0.0 ]
  
  # Форма — восьмиугольник, приближающий круг
  block (
    points 8
    point[0] [ -0.15  0.05 ]
    point[1] [ -0.10  0.13 ]
    point[2] [  0.10  0.13 ]
    point[3] [  0.15  0.05 ]
    point[4] [  0.15 -0.05 ]
    point[5] [  0.10 -0.13 ]
    point[6] [ -0.10 -0.13 ]
    point[7] [ -0.15 -0.05 ]
    z [ 0 0.15 ]
  )
  
  # Diff-drive модель
  drive "diff"
  
  # Сенсоры
  warehouse_lidar ( pose [ 0.0 0.0 -0.05 0.0 ] )
  
  # Навигационный флаг — робот может двигаться
  obstacle_return 1
  ranger_return 1.0
  blob_return 1
  
  # Цвет по умолчанию (переопределяется при спавне)
  color "red"
)