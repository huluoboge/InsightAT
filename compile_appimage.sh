#!/bin/bash
set -e

# 1. 配置路径
INSIGHTAT_BUILD_DIR=build-ceres-11.8         # InsightAT 可执行文件目录
APPDIR=InsightAT.AppDir
APPNAME=InsightAT
VERSION=0.1.0
CUDA_LIBS_DIR=/usr/local/cuda-11.8/lib64  # CUDA 动态库目录（如有不同请修改）

# 2. 清理旧目录
rm -rf "$APPDIR"
mkdir -p "$APPDIR/usr/bin" "$APPDIR/usr/lib" "$APPDIR/usr/share/$APPNAME"

# 3. 拷贝可执行文件
cp $INSIGHTAT_BUILD_DIR/isat_* "$APPDIR/usr/bin/"
cp $INSIGHTAT_BUILD_DIR/InsightAT "$APPDIR/usr/bin/"
# 如有其他可执行文件，继续 cp

# 4. 拷贝资源文件
cp -r data scripts "$APPDIR/usr/share/$APPNAME/"  # 如有需要

# 5. 收集依赖库（除系统库）
for exe in "$APPDIR/usr/bin/"*; do
    ldd "$exe" | grep "=>" | awk '{print $3}' | while read lib; do
        if [[ -n "$lib" && ! "$lib" =~ ^/lib && ! "$lib" =~ ^/usr/lib$ && ! "$lib" =~ ^/usr/lib/x86_64-linux-gnu/libc.so ]]; then
            cp -n "$lib" "$APPDIR/usr/lib/" 2>/dev/null || true
        fi
    done
done

# 6. 拷贝 CUDA 运行库（只拷贝 libcudart/libcublas/libcufft 等，不拷贝 libcuda.so）
for lib in libcudart.so* libcublas.so* libcufft.so* libnvrtc.so*; do
    if [ -e "$CUDA_LIBS_DIR/$lib" ]; then
        cp -n "$CUDA_LIBS_DIR/$lib" "$APPDIR/usr/lib/"
    fi
done

# 7. 生成 AppRun 启动器
cat > "$APPDIR/AppRun" <<EOF
#!/bin/bash
HERE="\$(dirname "\$(readlink -f "\$0")")"
export LD_LIBRARY_PATH="\$HERE/usr/lib:\$LD_LIBRARY_PATH"
export INSIGHTAT_DATA_DIR="\$HERE/usr/share/$APPNAME"
exec "\$HERE/usr/bin/\$@"
EOF
chmod +x "$APPDIR/AppRun"

# 8. 生成 desktop 文件（可选，CLI 可省略）
cat > "$APPDIR/$APPNAME.desktop" <<EOF
[Desktop Entry]
Type=Application
Name=InsightAT
Exec=isat_project
Icon=app
Categories=Utility;
EOF

# 9. 准备图标（可选）
# cp path/to/icon.png "$APPDIR/app.png"

# 10. 下载 linuxdeploy 和 appimagetool
wget -N https://github.com/linuxdeploy/linuxdeploy/releases/download/continuous/linuxdeploy-x86_64.AppImage
wget -N https://github.com/AppImage/AppImageKit/releases/download/continuous/appimagetool-x86_64.AppImage
chmod +x linuxdeploy-x86_64.AppImage appimagetool-x86_64.AppImage

# 11. 生成 AppImage
./linuxdeploy-x86_64.AppImage --appdir "$APPDIR" --output appimage

echo "AppImage 打包完成！"