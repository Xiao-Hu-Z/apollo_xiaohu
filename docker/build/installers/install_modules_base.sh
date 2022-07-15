#!/usr/bin/env bash

###############################################################################
# Copyright 2020 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

# Fail on first error.
set -e

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
. ${CURR_DIR}/installer_base.sh

# Dependency:
# openmpi <- boost <- vtk <- pcl
# bash ${CURR_DIR}/install_mpi.sh
bash ${CURR_DIR}/install_boost.sh

bash ${CURR_DIR}/install_ffmpeg.sh

# Proj was required to install VTK
bash ${CURR_DIR}/install_proj.sh
bash ${CURR_DIR}/install_vtk.sh

# PCL is required by [ Perception Localization Dreamview ]
bash ${CURR_DIR}/install_pcl.sh

# OpenCV depends on ffmpeg and vtk
bash ${CURR_DIR}/install_opencv.sh


# Clean up cache to reduce layer size.
apt-get clean && \
    rm -rf /var/lib/apt/lists/*
