cd /home/group1/jym/Repos/optix7course && \
# rm -rf build && \
# mkdir -p build && \
cd build && \
# cmake .. && \
make -j$(nproc) && \
echo "✓ 编译完成！" && \
ls -la bin/