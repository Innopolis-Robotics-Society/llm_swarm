Заметка оператора (из scenario_no-llm/README.md архива scenarios_6_7_12.zip)

Goals роботам отправлялись со сдвигами +1 и -1 по X относительно координат задания (task). 
Для этого в файле iros_llm_swarm_bt/scripts/e3_scripted_driver.py строки 215-218 были заменены на:
```python
for robots, (x, y) in assignments:
    for i, r in enumerate(robots):
        ids.append(r)
        dx = -1.0 if i == 0 else 1.0
        pts.append(Point(x=float(x) + dx, y=float(y), z=0.0))
```
