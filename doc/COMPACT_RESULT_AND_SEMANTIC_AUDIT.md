# 交付地图体积与车辆残留诊断

原 `--export-global` 会额外拼接所有保留关键帧的点云，包含帧间重叠；保留关键帧副本又占用另一份空间。新增 `--global-voxel-size 0.10` 可在世界系中跨关键帧去重，每个 10 cm 体素保留最先遇到的一个真实观测点，保留该点全部原有字段，不生成质心。体素分辨率不等于定位精度，细小结构仍可能丢失；可选 0.05 m 保留更多细节，文件也更大。默认不改旧导出方式。

`--compact-result` 默认开启 0.10 m 全局去重，只保存 `global_clean.pcd`、位姿/坐标元数据及诊断，不保存清理关键帧副本、删除点云或不确定点云。原始地图、缓存和已有历史结果均不删除。这个选项减少的是新结果的重复存储，不会自动释放已有历史文件占用。目录仍是 `data/<map>_clean[_编号]/`。此结果缺少清理关键帧，不能直接供现有逐帧 build_static_navigation 使用；需要该导航构建流程时仅添加 `--global-voxel-size 0.10`，保留关键帧输出。

去重使用临时 SQLite，数据库页缓存设为 16 MiB，每次处理至多 50000 点，按块写最终 PCD。操作系统页缓存和 Python/NumPy 仍有额外开销，不能视为严格的总内存上限。临时数据库会消耗磁盘，成功或异常关闭时移除。不调用会加载整张地图的 PCL 压缩器。

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/clean_map_vehicles.py \
  --map data/street_vehicle_test4 \
  --bag "/media/lizhi/Seagate Backup Plus Drive/M2DGR数据集/street_06_ros2" \
  --config config/m2dgr_clean_alignment13_keyframes.yaml \
  --device cpu --rviz --compact-result --global-voxel-size 0.10
```

已有完整 binary global_clean.pcd 可单独去重，不重新运行 YOLO，不改变原结果：

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
nice -n 19 python3 scripts/compact_clean_map.py \
  --result data/street_vehicle_test4_clean \
  --output data/street_vehicle_test4_compact --voxel-size 0.10
```

输出必须是新目录。去重只是几何处理，不能把重复点减少的数量解释为删除了车辆，也不能解决原图中的车辆残留。

车辆识别/关联问题需检查真实图像与点云投影，不默认降低证据要求、不删除所有未知点、不做无证据的大范围簇扩展。新增逐帧/汇总统计：有正票的点、观察数不足、正票数不足、背景否决、地面保护阻挡。已有 coverage/baseline/confidence 阻挡统计仍保留。各阻挡统计会重叠，不能相加当成点总数；这些是诊断计数，不是识别精度。

```bash
python3 scripts/audit_clean_map.py --result data/street_vehicle_test4_clean
```

该命令也能读取旧结果，报告空间组成、投影/对象/正票覆盖、原有语义证据和位姿失败计数，并列出诊断叠加图。旧报告未保存的新字段显示 null，不会编造。views 按图像-关键帧对统计，不能当作独立图像数量。需要结合一张包含明显车辆的原始图像与 overlay 判断：分割未命中、投影错位、深度关联失败、证据门限、地面估计，还是相机未覆盖。

此前日志中大量内部点时间约 -0.1 秒，稠密轨迹 usable=0。这个时间参考/去畸变问题没有在本次改动中修复，关键帧备用关联也不能证明其正确。需要原始传感器字段/时间约定和真实投影验证，不能仅修改 usable 或靠减小导出体素来掩盖。

回归覆盖跨帧去重、负坐标、非有限点、所有字段与真实观测坐标保留、临时文件清理、已保存结果压缩与原文件保护，以及真实清理流程的精简输出和拒绝原因统计。没有真实 bag/模型/ROS2 环境，未据此声称车辆识别精度提高或实际压缩比例。
