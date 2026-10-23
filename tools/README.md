# 测试工具

| 目录 | 内容 | 依赖 |
|---|---|---|
| `testdata/gen_packages.py` | 合成多帧任务包（雷达点云、两路相机图片、标定、位姿、逐点真值、运动目标真值框）与平台导入 CSV | Python 3 + numpy |
| `e2e/workbench-smoke.mjs` | 点云作业台浏览器端到端冒烟（模拟后端） | Node 22、Playwright、Chromium |

## 生成测试包

```bash
python3 tools/testdata/gen_packages.py --out ./out --packages 2 --frames 20 --prefix labeling/pc-test
```

输出：

```text
out/
├── import.csv                 # 平台“OSS 键清单”导入文件（每包一行：object_key,manifest_code,media_type）
├── classes.json               # 测试规范类别（名称、英文名、编号、颜色），用于在规范库建类别
└── PCTEST_0001/
    ├── package.json           # 任务包描述（pc-package-v1）
    ├── lidar/<帧ID>.pcd       # 雷达坐标，binary PCD（x y z intensity），按方位角扫描顺序
    ├── camera/<相机>/<帧ID>.png
    └── truth/                 # 真值（不上传也可以）：<帧ID>.label 每点类别编号（uint16），tracks.json 运动目标框
```

场景：自车沿道路行驶 8 m/s、10 Hz；地面、人行道、路沿、两侧建筑、树、杆、停放车辆；两辆行驶车辆（跟踪 ID 1、2）与一名行人（3）。单帧约 3.2 万点，20 帧约 67 万点。

上传时保持 `<prefix>/<包编号>/...` 的目录结构，`import.csv` 中的对象键与之对应。

## 端到端冒烟

见脚本文件头部说明。脚本只模拟后端接口，不需要数据库与对象存储；联调环境的完整链路按 [docs/11-阶段1测试手册](../docs/11-阶段1测试手册.md) 执行。
