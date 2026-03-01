# Git 仓库初始化指南

将 `cpp/` 目录作为独立的 Git 仓库发布。

## 🚀 快速设置

```bash
# 1. 进入 cpp 目录
cd cpp

# 2. 初始化 Git
git init

# 3. 添加所有文件
git add .

# 4. 首次提交
git commit -m "Initial commit: SuperPoint ONNX C++ v1.0.0"

# 5. 添加远程仓库 (GitHub/GitLab/etc)
git remote add origin https://github.com/yourusername/superpoint-onnx-cpp.git

# 6. 推送
git branch -M main
git push -u origin main
```

## 📝 建议的 Git 工作流

### 分支策略

```bash
main          # 稳定版本
├── develop   # 开发分支
├── feature/* # 新功能
└── hotfix/*  # 紧急修复
```

### 版本标签

```bash
# 创建版本标签
git tag -a v1.0.0 -m "Release version 1.0.0"
git push origin v1.0.0

# 列出所有标签
git tag -l
```

## 📦 发布清单

在发布到 GitHub 之前，确保：

- [x] README.md 完整且准确
- [x] LICENSE 文件存在
- [x] .gitignore 正确配置
- [x] 文档齐全 (docs/)
- [x] 示例图像包含
- [x] 编译脚本可执行
- [x] CHANGELOG.md 更新

## 🎯 GitHub 发布步骤

### 1. 创建仓库

在 GitHub 上创建新仓库:
- 名称: `superpoint-onnx-cpp`
- 描述: "High-performance SuperPoint keypoint detection in C++ with ONNX Runtime GPU"
- 公开/私有: 根据需求选择
- ❌ **不要**选择 "Initialize with README" (我们已经有了)

### 2. 推送代码

```bash
cd cpp
git remote add origin https://github.com/yourusername/superpoint-onnx-cpp.git
git branch -M main
git push -u origin main
git push --tags
```

### 3. 创建 Release

在 GitHub 上:
1. 进入仓库页面
2. 点击 "Releases" → "Create a new release"
3. 选择标签: `v1.0.0`
4. 标题: `SuperPoint ONNX C++ v1.0.0`
5. 描述:
   ```markdown
   ## 🎉 首次发布
   
   高性能 SuperPoint 关键点检测 C++ 实现
   
   ### ✨ 主要特性
   - GPU 加速 (CUDA 11.8)
   - 动态分辨率支持
   - 完整 NMS 实现
   - CPU/GPU 自动切换
   
   ### 📊 性能
   - GTX 1060: ~104ms @ 640×480 (9.5 FPS)
   - RTX 3060: ~35ms @ 640×480 (预估)
   
   ### 📖 文档
   - [快速开始](README.md)
   - [完整指南](docs/CPP_API_GUIDE.md)
   - [部署文档](docs/DEPLOYMENT.md)
   
   ### 💾 安装
   ```bash
   git clone https://github.com/yourusername/superpoint-onnx-cpp.git
   cd superpoint-onnx-cpp
   ./scripts/install_all.sh
   ```
   ```

6. 附件（可选）:
   - 预编译二进制
   - 示例结果图

### 4. 完善仓库

添加以下内容（在 GitHub Web 界面）:

**About (仓库简介)**:
- Description: "High-performance SuperPoint C++ with ONNX Runtime GPU"
- Website: (如果有)
- Topics: `computer-vision`, `keypoint-detection`, `onnx`, `cuda`, `cpp`, `superpoint`, `onnxruntime`

**README Badges** (可选):

在 README.md 顶部添加:
```markdown
![GitHub release](https://img.shields.io/github/v/release/yourusername/superpoint-onnx-cpp)
![License](https://img.shields.io/github/license/yourusername/superpoint-onnx-cpp)
![Stars](https://img.shields.io/github/stars/yourusername/superpoint-onnx-cpp)
```

## 📋 .gitattributes

创建 `.gitattributes` 文件：

```bash
cat > .gitattributes << 'EOF'
# Auto detect text files
* text=auto

# C++ files
*.cpp text
*.h text
*.hpp text
*.c text

# Shell scripts
*.sh text eol=lf

# CMake
*.cmake text
CMakeLists.txt text

# Documentation
*.md text
*.txt text

# Images (binary)
*.jpg binary
*.png binary
*.jpeg binary

# Models (binary)
*.onnx binary
*.pth binary

# Archives (binary)
*.tar.gz binary
*.zip binary
EOF

git add .gitattributes
git commit -m "Add .gitattributes"
```

## 🔄 持续集成（可选）

### GitHub Actions

创建 `.github/workflows/build.yml`:

```yaml
name: Build and Test

on:
  push:
    branches: [ main, develop ]
  pull_request:
    branches: [ main ]

jobs:
  build:
    runs-on: ubuntu-20.04
    
    steps:
    - uses: actions/checkout@v3
    
    - name: Install dependencies
      run: |
        sudo apt update
        sudo apt install -y cmake build-essential libopencv-dev
    
    - name: Build
      run: |
        mkdir build && cd build
        cmake ..
        make -j$(nproc)
    
    - name: Test
      run: |
        # 添加测试命令
        echo "Tests passed"
```

## 📢 推广

发布后，可以在以下地方分享:

1. **社交媒体**
   - Twitter/X
   - Reddit (r/computervision, r/MachineLearning)
   - LinkedIn

2. **技术社区**
   - Hacker News
   - Dev.to
   - Medium

3. **相关项目**
   - 在原始 SuperPoint 仓库提 Issue/讨论
   - ONNX Runtime 社区

## 🎓 示例 README Badges

完整 README 开头:

```markdown
# SuperPoint ONNX C++

<p align="center">
  <img src="examples/result_demo.jpg" alt="SuperPoint Demo" width="600"/>
</p>

<p align="center">
  <a href="https://github.com/yourusername/superpoint-onnx-cpp/releases">
    <img src="https://img.shields.io/github/v/release/yourusername/superpoint-onnx-cpp" alt="Release"/>
  </a>
  <a href="LICENSE">
    <img src="https://img.shields.io/badge/License-MIT-blue.svg" alt="License"/>
  </a>
  <a href="https://developer.nvidia.com/cuda-toolkit">
    <img src="https://img.shields.io/badge/CUDA-11.8-green.svg" alt="CUDA"/>
  </a>
  <a href="https://onnxruntime.ai/">
    <img src="https://img.shields.io/badge/ONNX%20Runtime-1.16.3-orange.svg" alt="ONNX"/>
  </a>
</p>

High-performance SuperPoint keypoint detection in C++ with ONNX Runtime GPU acceleration.
```

---

**准备就绪后，您的项目将会是一个完整、专业的开源项目！** 🚀
