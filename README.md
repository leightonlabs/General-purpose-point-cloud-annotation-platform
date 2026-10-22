# General-purpose-point-cloud-annotation-platform
A platform applicable to point cloud data, with customizable output rules and covering multiple annotation functions.

通用三维点云标注平台的设计、规范与计划。实现代码位于 [annotation-platform](https://github.com/leightonlabs/annotation-platform) 的独立点云模块：前端 `labeling-admin-web/src/workbench/editors/lidar/`，后端 `labeling-server` 的 `com.pmsm.zs.labeling.pointcloud` 包（不修改平台其它模块）。

- 文档入口：[docs/README.md](./docs/README.md)
- 一期范围：单帧点云的三维立体框（含跟踪 ID）与逐点语义分割，打通导入、派工、标注、提交校验、质检审核、导出全链路。
- 交付说明：[docs/08-一期交付说明.md](./docs/08-一期交付说明.md)
- 分阶段开发记录（进度、待定事项、待解答问题、下一阶段计划）：[docs/09-分阶段开发记录.md](./docs/09-分阶段开发记录.md)
