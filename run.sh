cd /home/jym/Repos/optix_collsion_check_linux && \
rm -rf build && \
mkdir -p build && \
cd build && \
cmake -DCMAKE_PREFIX_PATH=$CONDA_PREFIX .. && \
make -j$(nproc) && \
echo "✓ 编译完成！" && \
ls -la bin/