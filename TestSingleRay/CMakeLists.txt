# ======================================================================== #
# Copyright 2018-2025 Ingo Wald                                            #
#                                                                          #
# Licensed under the Apache License, Version 2.0 (the "License");          #
# you may not use this file except in compliance with the License.         #
# You may obtain a copy of the License at                                  #
#                                                                          #
#     http://www.apache.org/licenses/LICENSE-2.0                           #
#                                                                          #
# Unless required by applicable law or agreed to in writing, software      #
# distributed under the License is distributed on an "AS IS" BASIS,        #
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. #
# See the License for the specific language governing permissions and      #
# limitations under the License.                                           #
# ======================================================================== #

embed_ptx(
  OUTPUT_TARGET
  TestSingleRay_devicePrograms
  PTX_LINK_LIBRARIES
  gdt
  SOURCES
  devicePrograms.cu
  EMBEDDED_SYMBOL_NAMES
  embedded_ptx_code
)

add_executable(TestSingleRay
  optix7.h
  CUDABuffer.h
  LaunchParams.h
  utils.h
  SampleRenderer.h
  SampleRenderer.cpp
  main.cpp
  )

set_target_properties(TestSingleRay PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin
  OUTPUT_NAME TestSingleRay
)

target_link_libraries(TestSingleRay
  TestSingleRay_devicePrograms
  # gdt, pulls in optix and cuda 
  gdt
  # glfw and opengl, for display
  glfWindow
  )
target_compile_definitions(TestSingleRay PRIVATE PROJECT_ROOT_DIR="${PROJECT_SOURCE_DIR}")
