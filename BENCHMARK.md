# Channel-3 Benchmark — map: Among Us (The Skeld)

**Локации:**
`upper_engine (-21.9, 9.9)` · `cafeteria (2.7, 10.1)` · `weapons (17.6, 10.7)` · `reactor (-28.4, 0.9)` · `security (-21.9, 0.8)` · `medbay (-9.5, 4.0)` · `admin (4.0, 0.0)` · `o2 (13.4, 3.4)` · `navigation (29.5, 1.9)` · `electrical (-8.7, -5.9)` · `shields (10.4, -3.7)` · `lower_engine (-21.9, -8.4)` · `storage (0.6, -10.9)` · `communications (17.6, -9.7)`

**Группы:**
`cyan [0-3] @ upper_engine` · `magenta [4-7] @ electrical` · `green [8-11] @ navigation` · `orange [12-15] @ storage` · `yellow [16-19] @ weapons`

**Зоны формаций:** `cafeteria r=5` · `storage r=3` · `reactor r=2` (admin/o2/security/shields — слишком узко)

---

| № | Test case | Prompt | Expected output |
|---|-----------|--------|-----------------|
| 1 | `mapf_basic_01` | send cyan to cafeteria | `mapf` — robot_ids == [0,1,2,3]; spread == True OR len(goals) == 1; goals near (2.7, 10.1) |
| 2 | `mapf_basic_02` | orange robots go home | `mapf` — robot_ids == [12,13,14,15]; goals near storage (0.6, -10.9) |
| 3 | `mapf_basic_03` | move all robots to cafeteria | `mapf` — robot_ids == list(range(20)); spread == True; goals near (2.7, 10.1) |
| 4 | `mapf_basic_04` | drive yellow to navigation | `mapf` — robot_ids == [16,17,18,19]; goals near (29.5, 1.9) |
| 5 | `mapf_basic_05` | green to reactor | `mapf` — robot_ids == [8,9,10,11]; goals near (-28.4, 0.9) |
| 6 | `mapf_spread_01` | place cyan at storage | `mapf` — robot_ids == [0,1,2,3]; spread == True OR len(goals) == 1; goals near (0.6, -10.9) |
| 7 | `mapf_spread_02` | drive magenta to cafeteria, locate these robots in cafeteria explicitly | `mapf` — robot_ids == [4,5,6,7]; spread == True OR (len(goals)==4 and all distinct); NOT: два идентичных последовательных mapf |
| 8 | `mapf_explicit_01` | place robot_12 at (1.0, -10.9), robot_13 at (0.0, -10.9), robot_14 at (1.0, -11.9) | `mapf` — robot_ids contains 12, 13, 14; spread == False; len(goals) == 3; all goals distinct |
| 9 | `mapf_explicit_02` | place cyan in a 2x2 grid at cafeteria | `mapf` — robot_ids == [0,1,2,3]; len(goals) == 4 and all goals distinct; all near (2.7, 10.1) |
| 10 | `parallel_01` | yellow to navigation, green to reactor | `parallel` — two mapf steps; yellow goals near (29.5, 1.9); green goals near (-28.4, 0.9) |
| 11 | `parallel_02` | cyan and magenta both go to cafeteria | `parallel` — steps cover [0,1,2,3] and [4,5,6,7]; all goals near (2.7, 10.1) |
| 12 | `parallel_03` | magenta to green home, green to magenta home | `parallel` — two mapf steps; magenta [4-7] goals near navigation (28.5, 1.9); green [8-11] goals near electrical (-8.7, -5.9) |
| 13 | `parallel_04` | send all five groups to their home positions | `parallel` — five mapf steps; cyan→(-21.9,9.9); magenta→(-8.7,-5.9); green→(28.5,1.9); orange→(0.6,-10.9); yellow→(17.6,10.7) |
| 14 | `sequence_01` | cyan go to cafeteria, then form a wedge | `sequence` — step[0]: mapf [0-3] to (2.7,10.1); step[1]: formation leader_ns='robot_0', wedge offsets, follower_ns 3 entries |
| 15 | `sequence_02` | move orange to cafeteria, then send them to storage | `sequence` — step[0]: mapf [12-15] to (2.7,10.1); step[1]: mapf [12-15] to (0.6,-10.9) |
| 16 | `sequence_03` | cyan and magenta to cafeteria, then cyan forms a line while magenta goes home | `sequence` — step[0]: parallel (обе группы в cafeteria); step[1]: parallel (formation cyan + mapf magenta home) |
| 17 | `sequence_04` | move green to cafeteria, after that to navigation, then stop | `sequence` — step[0]: mapf [8-11]→cafeteria; step[1]: mapf [8-11]→(29.5,1.9); step[2]: idle |
| 18 | `formation_create_01` | magenta form a line | `sequence` — step[0]: mapf followers [5,6,7] к staging (robot_4_pos + offsets); step[1]: formation leader_ns='robot_4', line offsets, robot_4 NOT in step[0].robot_ids |
| 19 | `formation_create_02` | orange make a triangle formation | `sequence` — step[0]: mapf followers (not 12) staging; step[1]: formation leader_ns='robot_12', follower_ns≠∅, offsets_x≠∅, offsets_y≠∅ |
| 20 | `formation_create_03` | cyan form a wedge | `sequence` — staging mapf then formation; leader_ns='robot_0'; offsets_x=[-1.0,-1.0,...], offsets_y=[0.6,-0.6,...] |
| 21 | `formation_create_04` | green form an abreast line | `sequence` — formation step: offsets_x all 0.0; offsets_y non-zero and varied; follower_ns 3 entries |
| 22 | `formation_create_05` | form a line with orange, leader should be robot_15 | `sequence` — formation step: leader_ns='robot_15'; follower_ns contains robot_12, robot_13, robot_14; robot_15 NOT in staging mapf |
| 23 | `formation_move_01` | send the magenta line to storage | `mapf` — robot_ids == [4] only; goals near (0.6, -10.9); [5,6,7] NOT in robot_ids |
| 24 | `formation_move_02` | move cyan wedge to reactor | `mapf` — robot_ids == [0] only; goals near (-28.4, 0.9) |
| 25 | `formation_move_03` | magenta line, head to cafeteria | `sequence` — step[0]: mapf [5,6,7] staging; step[1]: formation 'magenta_line'; step[2]: mapf [4] to (2.7,10.1); len==3 |
| 26 | `disband_01` | disband the magenta line | `disband` — formation_id == 'magenta_line'; NOT idle |
| 27 | `disband_02` | break the magenta formation and send all four to storage | `sequence` — step[0]: disband 'magenta_line'; step[1]: mapf [4,5,6,7] goals near (0.6,-10.9) |
| 28 | `disband_03` | cancel the cyan wedge, then split: robot_0 to cafeteria, rest to storage | `sequence` — step[0]: disband 'cyan_wedge'; robot_0 → (2.7,10.1); [1,2,3] → (0.6,-10.9) |
| 29 | `formation_complex_01` | orange make a triangle and move to cafeteria | `sequence` — step[0]: mapf followers staging; step[1]: formation leader_ns='robot_12'; step[2]: mapf [12] to (2.7,10.1) |
| 30 | `formation_complex_02` | yellow form a wedge at cafeteria | `sequence` — mapf [16-19] to cafeteria; staging followers; formation leader_ns='robot_16', wedge offsets |
| 31 | `formation_complex_03` | cyan go to cafeteria, form a wedge, then go to reactor | `sequence` — mapf [0-3]→cafeteria; formation wedge; mapf [0]→(-28.4,0.9) — leader only |
| 32 | `idle_01` | stop | `idle` — reason non-empty |
| 33 | `idle_02` | halt everything | `idle` — reason non-empty |
| 34 | `idle_03` | cancel all missions | `idle` — reason non-empty |
| 35 | `escalate_01` | orange make a triangle formation *(robot_12 pose missing)* | `idle` — reason starts with 'needs_help:'; reply asks about robot_12 |
| 36 | `escalate_02` | что делают роботы | `idle` — reason starts with 'reply_only:'; reply describes magenta_line STABLE |
| 37 | `escalate_03` | where is robot_4 | `idle` — reason starts with 'reply_only:'; reply includes position near (2.7, 10.1) |
| 38 | `escalate_04` | form a triangle | `idle` — reason starts with 'needs_help:' or 'clarify:'; reply asks which group |
| 39 | `russian_01` | оранжевые в кафетерий | `mapf` — robot_ids == [12,13,14,15]; goals near (2.7, 10.1); reply in Russian |
| 40 | `russian_02` | голубые в кафе, потом построиться в линию | `sequence` — step[0]: mapf cyan→cafeteria; step[1]: formation line leader_ns='robot_0'; reply in Russian |
| 41 | `russian_03` | стоп | `idle` — reply in Russian |
| 42 | `russian_04` | жёлтые и зелёные одновременно в кафетерий | `parallel` — [16-19] и [8-11] к (2.7,10.1); reply in Russian |
| 43 | `russian_05` | расформировать линию маджента | `disband` — formation_id == 'magenta_line'; reply in Russian |
| 44 | `edge_01` | send all robots home | `parallel` — 5 mapf шагов; cyan→(-21.9,9.9); magenta→(-8.7,-5.9); green→(28.5,1.9); orange→(0.6,-10.9); yellow→(17.6,10.7) |
| 45 | `edge_02` | orange make a triangle and move to cafeteria *(robot_12 pose missing)* | `idle` — reason starts with 'needs_help:'; reply mentions robot_12 unavailable |
| 46 | `edge_03` | put magenta right behind the cyan wedge leader | `mapf` — robot_ids == [4,5,6,7]; goals derived from robot_0 pos (2.7,10.1) + rear offset |
| 47 | `edge_04` | move the formation to storage | `mapf` — robot_ids == [4] only (magenta_line STABLE); goals near (0.6,-10.9) |
| 48 | `edge_05` | green and yellow go to cafeteria, then form separate wedges | `sequence` — step[0]: parallel (green+yellow→cafeteria); step[1]: parallel (два formation, разные formation_id) |
| 49 | `edge_06` | repeat the last command | `idle` — reason starts with 'needs_help:' or 'clarify:'; no prior context |