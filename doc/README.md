# InsightAT 文档目录

本目录包含 InsightAT 项目的所有文档，按功能模块组织。

## 📁 目录结构

```
doc/
├── README.md                    ← 你在这里
└── rotation/                    ← 旋转表示法标准化文档
    ├── ROTATION_README.md          (文档导航索引)
    ├── ROTATION_STANDARDS.md       (国际标准和理论)
    ├── CODE_ANALYSIS_ROTATION.md   (代码问题分析)
    ├── ROTATION_QUICK_REFERENCE.md (快速查询卡片)
    └── ROTATION_PROJECT_SUMMARY.md (项目进展总结)
```

## 🎯 快速导航

### 旋转表示法（Rotation Representation）

InsightAT 的摄影测量模块使用欧拉角和四元数表示相机姿态。这涉及复杂的坐标系转换和约定，已被完整标准化。

**入口**: [rotation/ROTATION_README.md](rotation/ROTATION_README.md)

**包含内容**：
- ✅ 国际标准的完整解释（AIAA, IEEE, ISPRS）
- ✅ 摄影测量 vs 导航体系的对比
- ✅ 现有代码的问题分析
- ✅ 改进方案和实现指南
- ✅ 快速参考卡片和常见错误

**适合读者**：
- 需要理解相机姿态表示的开发者
- 进行 DBPose 相关代码修改的人
- 与外部摄影测量软件对接的人

---

## 📚 文档贡献指南

如果你想添加或更新文档：

1. **确定分类** - 选择合适的子目录（如 `rotation/`）
2. **遵循格式** - 参考已有文档的结构和风格
3. **更新导航** - 在对应子目录的 README 中更新索引
4. **更新本文件** - 如果添加了新的子目录，更新这个总 README
5. **链接代码** - 在代码注释中链接相关文档

## 🔗 代码中的文档引用

代码中的注释应该链接到相应的文档。例如：

```cpp
/**
 * 相关文档：doc/rotation/ROTATION_STANDARDS.md
 */
struct DBPose {
    // ...
};
```

这样可以帮助开发者快速找到所需的文档。

---

## 📖 现有文档列表

### rotation/ 子目录（旋转表示法）

| 文件 | 用途 | 读者 | 时间 |
|------|------|------|------|
| [ROTATION_README.md](rotation/ROTATION_README.md) | 导航索引 | 所有人 | 5分钟 |
| [ROTATION_STANDARDS.md](rotation/ROTATION_STANDARDS.md) | 理论基础 | 开发者/审查者 | 30分钟 |
| [CODE_ANALYSIS_ROTATION.md](rotation/CODE_ANALYSIS_ROTATION.md) | 问题分析 | 代码修改者 | 20分钟 |
| [ROTATION_QUICK_REFERENCE.md](rotation/ROTATION_QUICK_REFERENCE.md) | 快速查询 | 所有人 | 5分钟 |
| [ROTATION_PROJECT_SUMMARY.md](rotation/ROTATION_PROJECT_SUMMARY.md) | 项目总结 | 项目管理 | 10分钟 |

---

## 💡 使用建议

### 新开发者上手
1. 读 [rotation/ROTATION_QUICK_REFERENCE.md](rotation/ROTATION_QUICK_REFERENCE.md)（5分钟）
2. 在编码时查阅 [rotation/ROTATION_README.md](rotation/ROTATION_README.md)

### 修改旋转相关代码
1. 查看 [rotation/CODE_ANALYSIS_ROTATION.md](rotation/CODE_ANALYSIS_ROTATION.md) 的改进方案
2. 参考 [rotation/ROTATION_STANDARDS.md](rotation/ROTATION_STANDARDS.md) 的具体约定
3. 检查 [rotation/ROTATION_QUICK_REFERENCE.md](rotation/ROTATION_QUICK_REFERENCE.md) 的常见错误

### 深入学习
遵循 [rotation/ROTATION_README.md](rotation/ROTATION_README.md) 中的学习路径

---

## 🚀 计划中的文档

以下主题的文档计划在未来添加：

- [ ] `algorithms/` - 算法文档（SfM, BA, 特征匹配等）
- [ ] `architecture/` - 系统架构文档
- [ ] `api/` - API 参考文档
- [ ] `tutorials/` - 新手教程

---

**最后更新**: 2026-02-08  
**位置**: `doc/README.md`
