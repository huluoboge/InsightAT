# SuperPoint ONNX C++ - Version History

## Version 1.0.0 (2026-02-15)

### Features
- ✅ Complete ONNX Runtime C++ implementation
- ✅ GPU acceleration with CUDA 11.8
- ✅ Dynamic resolution support
- ✅ Full NMS (Non-Maximum Suppression) implementation
- ✅ CPU/GPU automatic fallback
- ✅ Keypoint visualization

### Performance
- GTX 1060 6GB: 104.5ms @ 640×480 (9.5 FPS)
- 1.72x improvement over Python CPU baseline

### Dependencies
- CUDA 11.8.89
- cuDNN 8.9.7
- ONNX Runtime 1.16.3 GPU
- OpenCV 4.5.4+

### Documentation
- Complete API guide
- Installation instructions
- Performance benchmarks
- GPU compatibility matrix

### Known Issues
- Shared library build not yet implemented
- Python bindings pending

### Future Plans
- v1.1.0: Shared library support
- v1.2.0: Python bindings
- v1.3.0: Batch processing
- v2.0.0: TensorRT backend option

---

## Changelog Format

**Added**: New features  
**Changed**: Changes in existing functionality  
**Deprecated**: Soon-to-be removed features  
**Removed**: Removed features  
**Fixed**: Bug fixes  
**Security**: Vulnerability fixes
