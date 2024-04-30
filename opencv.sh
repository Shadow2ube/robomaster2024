#!/bin/bash
# install opencv
set -e

ROOTDIR=${ZZROOT:-$HOME/app}
NAME1="opencv"
NAME2="opencv_contrib"
TYPE=".tar.gz"
FILE1="$NAME1$TYPE"
FILE2="$NAME2$TYPE"
DOWNLOADURL1="https://github.com/opencv/opencv/archive/4.4.0.tar.gz"
DOWNLOADURL2="https://github.com/opencv/opencv_contrib/archive/4.4.0.tar.gz"
echo $NAME1 will be installed in "$ROOTDIR"

mkdir -p "$ROOTDIR/downloads"
cd "$ROOTDIR"

if [ -f "downloads/$FILE1" ]; then
    echo "downloads/$FILE1 exist"
else
    echo "$FILE1 does not exist, downloading from $DOWNLOADURL1"
    wget $DOWNLOADURL1 -O $FILE1
    mv $FILE1 downloads/
fi

if [ -f "downloads/$FILE2" ]; then
    echo "downloads/$FILE2 exist"
else
    echo "$FILE2 does not exist, downloading from $DOWNLOADURL2"
    wget $DOWNLOADURL2 -O $FILE2
    mv $FILE2 downloads/
fi

mkdir -p src/$NAME1
mkdir -p src/$NAME2
tar xf downloads/$FILE1 -C src/$NAME1 --strip-components 1
tar xf downloads/$FILE2 -C src/$NAME2 --strip-components 1

cd src/$NAME1
mkdir -p build
cd build

export PKG_CONFIG_PATH="$ROOTDIR"/lib/pkgconfig:$PKG_CONFIG_PATH

cmake \
    -D CMAKE_BUILD_TYPE=RELEASE \
    -D BUILD_EXAMPLES=OFF \
    -D WITH_QT=OFF \
    -D CUDA_GENERATION=Auto \
    -D OpenGL_GL_PREFERENCE=GLVND \
    -D BUILD_opencv_hdf=OFF \
    -D BUILD_PERF_TESTS=OFF \
    -D BUILD_TESTS=OFF \
    -D BUILD_opencv_cnn_3dobj=OFF \
    -D BUILD_opencv_dnn=OFF \
    -D BUILD_opencv_datasets=OFF \
    -D BUILD_opencv_aruco=OFF \
    -D BUILD_opencv_tracking=OFF \
    -D BUILD_opencv_text=OFF \
    -D BUILD_opencv_stereo=OFF \
    -D BUILD_opencv_saliency=OFF \
    -D BUILD_opencv_rgbd=OFF \
    -D BUILD_opencv_reg=OFF \
    -D BUILD_opencv_ovis=OFF \
    -D BUILD_opencv_matlab=OFF \
    -D BUILD_opencv_freetype=OFF \
    -D BUILD_opencv_dpm=OFF \
    -D BUILD_opencv_face=OFF \
    -D BUILD_opencv_dnn_superres=OFF \
    -D BUILD_opencv_dnn_objdetect=OFF \
    -D BUILD_opencv_bgsegm=OFF \
    -D BUILD_opencv_cvv=OFF \
    -D BUILD_opencv_ccalib=OFF \
    -D BUILD_opencv_bioinspired=OFF \
    -D BUILD_opencv_dnn_modern=OFF \
    -D BUILD_opencv_dnns_easily_fooled=OFF \
    -D BUILD_JAVA=OFF \
    -D BUILD_opencv_python2=OFF \
    -D BUILD_NEW_PYTHON_SUPPORT=ON \
    -D BUILD_opencv_python3=ON \
    -D HAVE_opencv_python3=ON \
    -D PYTHON_DEFAULT_EXECUTABLE="$(which python3)" \
    -D WITH_OPENGL=ON \
    -D WITH_VTK=OFF \
    -D FORCE_VTK=OFF \
    -D WITH_TBB=ON \
    -D WITH_GDAL=ON \
    -D ENABLE_FAST_MATH=1 \
    -D CUDA_FAST_MATH=ON \
    -D WITH_CUBLAS=ON \
    -D WITH_MKL=ON \
    -D MKL_USE_MULTITHREAD=ON \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D WITH_CUDA=ON \
    -D NVCC_FLAGS_EXTRA="--default-stream per-thread" \
    -D WITH_NVCUVID=OFF \
    -D BUILD_opencv_cudacodec=OFF \
    -D MKL_WITH_TBB=ON \
    -D WITH_FFMPEG=ON \
    -D MKL_WITH_OPENMP=ON \
    -D WITH_XINE=ON \
    -D ENABLE_PRECOMPILED_HEADERS=OFF \
    -D CMAKE_INSTALL_PREFIX=$(python3 -c "import sys; print(sys.prefix)") \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D OPENCV_PC_FILE_NAME=opencv.pc \
    -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D INSTALL_C_EXAMPLES=ON \
    -D WITH_V4L=ON \
    -D WITH_GSTREAMER=ON \
    -D OPENCV_PYTHON3_INSTALL_PATH=$(python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") \
    -D PYTHON_EXECUTABLE=$(which python3) \
    ..

make -j"$(nproc)" && make install

echo "$NAME1" installed on "$ROOTDIR"
echo add following line to .zshrc
echo export OpenCV_DIR="$ROOTDIR"
