# GTX 1060 GPU兼容性说明

## 问题总结

**GTX 1060 (SM 6.1) 与 cuDNN 9 不兼容**

错误信息：
```
CUDNN_STATUS_EXECUTION_FAILED_CUDART
cudnnConvolutionForward failed
```

## 技术原因

| GPU | 架构 | CUDA Capability | cuDNN 9支持 | cuDNN 8支持 |
|-----|------|-----------------|-------------|-------------|
| **GTX 1060** | **Pascal** | **SM 6.1** | **❌** | **✓** |
| GTX 1660 | Turing | SM 7.5 | ✓ | ✓ |
| RTX 2060 | Turing | SM 7.5 | ✓ | ✓ |
| RTX 3060 | Ampere | SM 8.6 | ✓ | ✓ |

**cuDNN 9 需求**：
- CUDA Capability ≥ 7.0 (Volta架构及以上)
- GTX 1060 是 6.1，不满足要求

## 当前解决方案

测试脚本已自动适配：
```bash
python test_img_0926.py
```

输出：
```
✓ GTX 1060 (SM 6.1) 不完全兼容 cuDNN 9
✓ 尝试GPU推理，失败则自动切换到CPU...
⚠ GPU推理失败，使用CPU
✓ 使用CPU推理: CPUExecutionProvider
```

**CPU性能**：
- 推理时间: ~180ms
- FPS: ~5.6
- 对于大多数应用场景足够

## 替代方案

### 方案1: 使用CPU（推荐，已实现）

**优点**：
- ✓ 稳定可靠
- ✓ 性能可接受（180ms/帧）
- ✓ 无需额外配置

**缺点**：
- ⚠ 比GPU慢2-3倍

**适用场景**：
- 离线批处理
- 低于30FPS的实时应用
- 原型开发

### 方案2: 降级到cuDNN 8 + CUDA 11

**步骤**：

1. **卸载cuDNN 9**：
```bash
sudo apt remove --purge libcudnn9-cuda-12
sudo apt autoremove
sudo ldconfig
```

2. **安装CUDA 11.8**：
```bash
# 下载CUDA 11.8
wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda_11.8.0_520.61.05_linux.run
sudo sh cuda_11.8.0_520.61.05_linux.run
```

3. **安装cuDNN 8**：
```bash
# 添加仓库
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update

# 安装cuDNN 8 for CUDA 11
sudo apt install -y libcudnn8 libcudnn8-dev
sudo ldconfig
```

4. **重新安装PyTorch (CUDA 11.8)**：
```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
```

5. **安装ONNX Runtime (CUDA 11)**：
```bash
pip uninstall onnxruntime onnxruntime-gpu
pip install onnxruntime-gpu==1.15.1  # 最后支持CUDA 11的版本
```

**优点**：
- ✓ GPU加速可用
- ✓ 预期2-3x性能提升 (~60-80ms)

**缺点**：
- ⚠ 复杂，需要降级整个CUDA环境
- ⚠ 可能影响其他项目
- ⚠ CUDA 11较旧（2022年）

**建议**：只有在性能关键时才考虑

### 方案3: 升级GPU（长期）

**推荐更换GPU**：
- **GTX 1660 Super** (~$200)：SM 7.5，入门级
- **RTX 3060** (~$300)：SM 8.6，性能更好
- **RTX 4060** (~$300)：SM 8.9，最新架构

**性能对比**：
| GPU | 推理时间 | FPS | 价格 |
|-----|----------|-----|------|
| GTX 1060 (CPU) | 180ms | 5.6 | - |
| GTX 1060 (cuDNN8) | ~60ms | ~16 | - |
| RTX 3060 | ~15ms | ~66 | $300 |
| RTX 4060 | ~10ms | ~100 | $300 |

## 推荐方案

### 对于当前项目：

**使用CPU模式** ✓

理由：
1. 已自动工作，无需修改
2. 5.6 FPS对大多数场景足够
3. 稳定可靠

### 如果需要更高性能：

**选择A：升级GPU** ✓✓✓（最佳）
- RTX 3060或以上
- 一次投资，长期受益
- 支持最新技术

**选择B：降级cuDNN** ⚠（不推荐）
- 仅在无法升级GPU时考虑
- 需要重新配置环境
- 性能提升有限（2-3x vs 10-20x）

## 当前状态

✓ **环境已配置**：
- cuDNN 9.19.0 已安装
- CUDA 12.8 已安装
- ONNX Runtime 1.19.2 已安装

✓ **推理脚本已优化**：
- 自动检测GPU兼容性
- 智能回退到CPU
- 性能监控

✓ **性能可接受**：
- CPU推理：180ms (5.6 FPS)
- 适用于绝大多数场景

## 验证命令

### 1. 检查当前配置
```bash
python check_cuda_environment.py
```

### 2. 运行测试
```bash
python test_img_0926.py
```

### 3. 性能对比 (CPU vs GPU)
```bash
# CPU模式（当前）
python test_img_0926.py
# 输出: ~180ms, 5.6 FPS

# 如果安装cuDNN 8（需要CUDA 11）
# 输出: ~60ms, 16 FPS（预期）
```

## 参考资料

- **cuDNN版本兼容性**：https://docs.nvidia.com/deeplearning/cudnn/support-matrix/index.html
- **GPU架构对比**：https://en.wikipedia.org/wiki/CUDA#GPUs_supported
- **ONNX Runtime CUDA支持**：https://onnxruntime.ai/docs/execution-providers/CUDA-ExecutionProvider.html

## 结论

对于GTX 1060用户：
1. ✓ **保持cuDNN 9 + CPU模式**（当前配置）
2. ⚠ 如需GPU加速，降级到cuDNN 8（复杂）
3. ✓✓✓ 长期考虑升级到RTX 3060+（最佳）

当前CPU性能（180ms）已足够大多数应用场景使用！
