# General-purpose-point-cloud-annotation-platform
A platform applicable to point cloud data, with customizable output rules and covering multiple annotation functions.

通用三维点云标注平台的设计、规范与计划。实现代码位于 [annotation-platform](https://github.com/leightonlabs/annotation-platform)（点云编辑器族 `labeling-admin-web/src/workbench/editors/lidar/` 与 `labeling-server` 的点云校验、导出）。

- 文档入口：[docs/README.md](./docs/README.md)
- 一期范围：单帧点云的三维立体框（含跟踪 ID）与逐点语义分割，打通导入、派工、标注、提交校验、质检审核、导出全链路。
