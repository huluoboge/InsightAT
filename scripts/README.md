# InsightAT Scripts

此目录包含InsightAT项目的辅助脚本和工具。

## 可视化工具

### visualize_vlad_retrieval.py

**功能**: VLAD检索结果可视化工具

生成HTML报告，展示每个查询图像的top-k检索结果，用于人工评估检索质量。

**用法**:
```bash
python scripts/visualize_vlad_retrieval.py \
    --vlad-dir ./vlad_cache \
    --images ./images.json \
    --output ./retrieval_report.html \
    --top-k 10
```

**详细文档**: [doc/tools/visualize_vlad_retrieval.md](../doc/tools/visualize_vlad_retrieval.md)

**特点**:
- 📊 加载预计算的VLAD向量（.isat_vlad格式）
- 🔍 对每个查询执行top-k检索
- 🎨 距离颜色编码（绿/橙/红）
- 📝 生成交互式HTML报告

### quick_start_vlad_visualization.sh

**功能**: VLAD可视化完整流程示例

从特征提取到可视化的一站式脚本，适合快速测试和演示。

**用法**:
1. 修改脚本中的 `IMAGE_LIST` 和 `IMAGE_DIR` 变量
2. 运行脚本:
```bash
./scripts/quick_start_vlad_visualization.sh
```

**包含步骤**:
1. 提取SIFT特征（检索流，降采样到1024）
2. 训练VLAD codebook（128聚类，PCA降维到512维）
3. 运行VLAD检索（生成缓存）
4. 生成HTML可视化报告

## DBoW训练工具

### dbow/

DBoW3词汇树训练相关的Python脚本和工具。

详见: [dbow/README.md](dbow/README.md)

## 未来计划

- **定量评估脚本**: 计算Recall@K、mAP等指标
- **消融实验工具**: 自动化参数扫描和结果对比
- **性能基准测试**: 与其他检索方法的对比
- **数据增强工具**: 图像预处理和数据扩充

## 贡献

添加新脚本时，请：
1. 添加清晰的文档注释
2. 遵循项目的命名规范
3. 在本README中添加说明
4. 如有需要，在 `doc/tools/` 中创建详细文档
