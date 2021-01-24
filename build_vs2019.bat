@ECHO OFF

CALL "F:\Programme\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvarsall.bat" amd64

SET PATH=F:\Tools\cmake-3.19.2-win64-x64\bin;%PATH%;F:\Projects\libs\libs_vstudio15\utils

REM OpenNI
SET OPENNI_DIR=F:\Programme\OpenNI2
SET OPENNI2_INCLUDE64=%OPENNI_DIR%\Include
SET OPENNI2_LIB64=%OPENNI_DIR%\Lib
SET OPENNI2_REDIST64=%OPENNI_DIR%\Redist
SET PC_OPENNI_INCLUDEDIR=%OPENNI2_INCLUDE64%
SET PC_OPENNI_LIBDIR=%OPENNI2_LIB64%

REM OpenCV
SET OpenCV_DIR=F:/Projects/libs/Open3D/opencv/install

REM Eigen
SET Eigen3_DIR=F:/Projects/libs/Open3D/Eigen/install/share/eigen3/cmake

REM Boost
SET BOOST_ROOT=F:/Projects/libs/Open3D/boost/install
SET BOOST_LIBRARYDIR=%BOOST_ROOT%/lib
SET Boost_ADDITIONAL_VERSIONS="1.75"

mkdir build_2019
cd build_2019

cmake -Wno-dev -G "Visual Studio 16 2019" -A x64 -DCMAKE_CONFIGURATION_TYPES=Release;RelWithDebInfo;Debug -DOpen3D_DIR=F:/Projects/libs/Open3D/install_b/CMake -DPC_OPENNI_INCLUDEDIR=%PC_OPENNI_INCLUDEDIR% -DPC_OPENNI_LIBDIR=%PC_OPENNI_LIBDIR% -DOpenCV_DIR=%OpenCV_DIR% -DEigen3_DIR=%Eigen3_DIR% -DBOOST_ROOT=%BOOST_ROOT% -DBOOST_LIBRARYDIR=%BOOST_LIBRARYDIR% -DBoost_ADDITIONAL_VERSIONS=%Boost_ADDITIONAL_VERSIONS% ../src 2>&1 | mtee /E cmake_output.log

cd ..
pause
