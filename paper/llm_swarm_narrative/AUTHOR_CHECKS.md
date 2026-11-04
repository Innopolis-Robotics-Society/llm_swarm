# Сведения для завершения научного черновика

Площадка не зафиксирована. Ниже нужны проверки уже существующих материалов, а не новые эксперименты.

1. Реестр запусков: session IDs для 21 reported sessions, 18 included runs, 3 formation-mission exclusions (M4) и M3 before/after. Три post-guard cases подтверждены как B-M3 entries по session ID (`20260818_111548`, `113030`, `114445`); pre-guard cases — `20260815_153035`, `154541` (8/9, Chutes). Не складывать 5 before/after cases с 18 основной серии.
2. Конфигурации: commit/tag системы, model revision, quantization, serving versions, endpoint/provider, prompt/schema hashes, launch parameters и seeds — только если такие сведения сохранились. Не подставлять неизвестные defaults.
3. Carry evaluator: автор подтвердил dwell сначала в pickup, затем в drop-off как единственное условие success. Нужны численный dwell threshold и код переходов для воспроизводимости. Радиус 1.5 м взят из исходной рукописи. Дополнительные детали identity binding между двумя этапами не описывались как проверенный факт без кода.
4. Assignment counts: сверить 5/11 inspected plans с тремя model-9b и двумя B cases. Если decision journals не сохранились, описывать это как reported inspection, не как новый аудит.
5. Три main-series B-M3 runs и before/after: подтвердить software/guard version и сопоставление записей; прежние totals не позволяют установить их пересечение.
6. Makespan: подтвердить начало отсчёта от первого /tasks/state tick. Command timestamps не восстановлены; latency/overhead выводы остаются ограниченными.
7. Concurrency: если есть activity traces, описать реальную одновременную активность. Пока статья утверждает только deployment 20 robots и не заявляет scaling/concurrent coordination 20 agents.
8. Offline harness: raw model responses, 44 case definitions, validators, prompts и analysis scripts. Проверить pass@1 как aggregate call pass rate и pass@5 как долю cases с хотя бы одним passing output среди пяти attempts.
9. Formation case: проверить описание ширины, трактовку column/abreast, сообщение path failure и использованную footprint model. Количественная частота таких ошибок не заявляется.
10. Артефакты: выяснить, какие код/rosbags/journals реально доступны, и оформить versioned reviewer-accessible package или честное Data/Code Availability Statement. Редактура не подтверждает их наличие.
11. Авторы и declarations: утверждённый список, affiliations, corresponding-author email, funding, conflicts, contributions. Английское имя пользователя — Artem Tuzov. Не объявлять отсутствие финансирования или conflicts без сведений от авторов.
12. Библиография: новые близкие источники и изменённые записи проверены по первичным страницам; остальную унаследованную библиографию сверить перед подачей.

Рукопись явно признаёт текущие ограничения. Эти пункты не следует автоматически заполнять правдоподобными значениями или удалять ради более убедительного вида.
