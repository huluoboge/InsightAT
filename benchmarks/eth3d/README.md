# ETH3D 位姿基准

## 位姿集合快照

`data/wai_eth3d_pose_collection_v1.json` 是一次完整 `wai_eth3d` 批处理生成的位姿集合快照，包含 13 个场景的：

- `ground_truth`：从各场景 `scene_meta.json` 收集的真值位姿；
- `estimated`：InsightAT 增量 SfM 输出的估计位姿；
- `matching`：按图像名匹配后的数量统计。

该文件是位姿评估的 benchmark 输入，不包含 ETH3D 原始图像；原始图像仍保持只读并位于数据集目录之外。

同一份快照已经展开保存为 CSV：

- `data/wai_eth3d_ground_truth_poses_v1.csv`
- `data/wai_eth3d_estimated_poses_v1.csv`
- `data/wai_eth3d_pose_comparison_v1.csv`
- `data/wai_eth3d_pose_metrics_v1.csv`

## 运行位姿评估

从仓库根目录运行：

```bash
python3 benchmarks/eth3d/evaluate_wai_eth3d_poses.py \
  --poses benchmarks/eth3d/data/wai_eth3d_pose_collection_v1.json \
  --output-dir /tmp/wai_eth3d_evaluation
```

评估会对每个场景独立估计 Sim(3)：

```text
C_gt ≈ scale * R_align * C_est + t
```

默认会生成：

- `ground_truth_poses.csv`：展开后的真值位姿；
- `estimated_poses.csv`：展开后的估计位姿；
- `pose_comparison.csv`：逐图像匹配、对齐位置和误差；
- `pose_evaluation.csv`：按场景及全部场景汇总的指标；
- `pose_evaluation.json`：带元数据的汇总结果。

如果要复现仓库中的 CSV 快照，可以将输出文件名指定为上述 `v1` 文件名。

然后输出相机中心位置误差和旋转误差。不能把不同场景直接放在同一个坐标系中对齐。
