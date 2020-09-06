#!/bin/bash

SRC=/tmp/allwpilib

if ! [[ -e ./.git ]]; then
  echo "Run this script from the root directory of the repository"
  exit 1
fi

if ! [[ -e $SRC ]]; then
  git clone git://gitub.com/wpilibsuite/allwpilib $SRC
else
  pushd $SRC
  git switch master
  git pull
  popd
fi

# wpimath

rm -r thirdparty/wpimath

mkdir -p thirdparty/wpimath/src/main/native/{cpp,include/frc}
cp -r $SRC/wpimath/src/main/native/cpp/{controller,drake,estimator,MathShared.cpp,StateSpaceUtil.cpp} \
  thirdparty/wpimath/src/main/native/cpp
cp -r $SRC/wpimath/src/main/native/include/{drake,wpimath} \
  thirdparty/wpimath/src/main/native/include
cp -r $SRC/wpimath/src/main/native/include/frc/{controller,estimator,StateSpaceUtil.h,system} \
  thirdparty/wpimath/src/main/native/include/frc

mkdir -p thirdparty/wpimath/src/main/native/include/unsupported/Eigen/src
cp -r $SRC/wpimath/src/main/native/include/unsupported/Eigen/AutoDiff \
  thirdparty/wpimath/src/main/native/include/unsupported/Eigen
cp -r $SRC/wpimath/src/main/native/include/unsupported/Eigen/src/AutoDiff \
  thirdparty/wpimath/src/main/native/include/unsupported/Eigen/src

mkdir -p thirdparty/wpimath/src/test/native/{cpp,include}
cp -r $SRC/wpimath/src/test/native/cpp/{controller,drake,EigenTest.cpp,estimator,StateSpaceTest.cpp,StateSpaceUtilTest.cpp,system} \
  thirdparty/wpimath/src/test/native/cpp
cp -r $SRC/wpimath/src/test/native/include/drake \
  thirdparty/wpimath/src/test/native/include

# Backport state-space library to customized 2020 units library
pushd thirdparty/wpimath/src/main/native/include/frc
find . -type f \( -name \*\.cpp -o -name \*\.h \) -exec sed -i 's!"units/!"units_shim/!g' {} \;
popd
pushd thirdparty/wpimath/src/test/native/cpp
find . -type f \( -name \*\.cpp -o -name \*\.h \) -exec sed -i 's!"units/!"units_shim/!g' {} \;
popd

# Backport StateSpaceUtil.cpp to 2020 frc::Pose2d
git apply wpimath.patch

# wpiutil

rm -r thirdparty/wpiutil

mkdir -p thirdparty/wpiutil/src/main/native/include/wpi
cp $SRC/wpiutil/src/main/native/include/wpi/static_circular_buffer.h \
  thirdparty/wpiutil/src/main/native/include/wpi

mkdir -p thirdparty/wpiutil/src/test/native/cpp
cp $SRC/wpiutil/src/test/native/cpp/StaticCircularBufferTest.cpp \
  thirdparty/wpiutil/src/test/native/cpp
