// ======================================================================== //
// Copyright 2018-2019 Ingo Wald                                            //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

//#define ENABLE_DEBUG_LOG

#ifdef ENABLE_DEBUG_LOG
#define LOG(msg, ...) printf("[LOG] " msg "\n", ##__VA_ARGS__)
#else
#define LOG(msg, ...) // 什么也不做
#endif

#include <optix_device.h>
#include <cuda_runtime.h>
#include "optix7.h"
#include "LaunchParams.h"
#include <stdio.h>
using namespace osc;

namespace osc {

	/*! launch parameters in constant memory, filled in by optix upon
		optixLaunch (this gets filled in from the buffer we pass to
		optixLaunch) */
	extern "C" __constant__ LaunchParams optixLaunchParams;

	// for this simple example, we have a single ray type
	enum { SURFACE_RAY_TYPE = 0, RAY_TYPE_COUNT };

	static __forceinline__ __device__
		void* unpackPointer(uint32_t i0, uint32_t i1)
	{
		const uint64_t uptr = static_cast<uint64_t>(i0) << 32 | i1;
		void* ptr = reinterpret_cast<void*>(uptr);
		return ptr;
	}

	static __forceinline__ __device__
		void  packPointer(void* ptr, uint32_t& i0, uint32_t& i1)
	{
		const uint64_t uptr = reinterpret_cast<uint64_t>(ptr);
		i0 = uptr >> 32;
		i1 = uptr & 0x00000000ffffffff;
	}

	template<typename T>
	static __forceinline__ __device__ T* getPRD()
	{
		const uint32_t u0 = optixGetPayload_0();
		const uint32_t u1 = optixGetPayload_1();
		return reinterpret_cast<T*>(unpackPointer(u0, u1));
	}

	// 初始刀头朝上，即刀身方向朝下
	__device__ void rotateToolPoint(const vec3f& toolPoint, const vec3f& targetDir, vec3f& rotated)
	{
		const vec3f defaultUp = vec3f(0.f, 0.f, -1.f);

		// 计算旋转轴
		vec3f axis = cross(defaultUp, targetDir);
		float axisLength = length(axis);

		// 如果旋转轴长度接近零，说明 targetDir 与 defaultUp 平行或反平行
		if (axisLength < 1e-6f) {
			// 如果方向相同，不需要旋转
			if (dot(defaultUp, targetDir) > 0) {
				rotated = toolPoint;
			}
			else {
				// 如果方向相反，直接取反
				rotated = -toolPoint;
			}
			return;
		}

		// 归一化旋转轴
		axis = axis / axisLength;

		// 计算旋转角度
		float cosTheta = dot(defaultUp, targetDir);
		float theta = acosf(cosTheta);

		// 进行旋转变换
		rotated = toolPoint * cosf(theta) +
			cross(axis, toolPoint) * sinf(theta) +
			axis * dot(axis, toolPoint) * (1.f - cosf(theta));
	}

	//------------------------------------------------------------------------------
	// closest hit and anyhit programs for radiance-type rays.
	//
	// Note eventually we will have to create one pair of those for each
	// ray type and each geometry type we want to render; but this
	// simple example doesn't use any actual geometries yet, so we only
	// create a single, dummy, set of them (we do have to have at least
	// one group of them to set up the SBT)
	//------------------------------------------------------------------------------

	extern "C" __global__ void __closesthit__radiance()
	{
		//const int   primID = optixGetPrimitiveIndex();
		const float hitDistance = optixGetRayTmax();
		//vec3f& prd = *(vec3f*)getPRD<vec3f>();
		//prd = gdt::randomColor(primID);
		float& prd = *(float*)getPRD<float>();
		//prd = static_cast<int>(primID);
		prd = hitDistance;
	}

	extern "C" __global__ void __anyhit__radiance()
	{ /*! for this simple example, this will remain empty */
	}



	//------------------------------------------------------------------------------
	// miss program that gets called for any ray that did not have a
	// valid intersection
	//
	// as with the anyhit/closest hit programs, in this example we only
	// need to have _some_ dummy function to set up a valid SBT
	// ------------------------------------------------------------------------------

	extern "C" __global__ void __miss__radiance()
	{
		/*vec3f& prd = *(vec3f*)getPRD<vec3f>();*/
		// set to constant white as background color
		/*prd = vec3f(1.f);*/
		//int& prd = *(int*)getPRD<int>();
		float& prd = *(float*)getPRD<float>();
		prd = -1.0f;
	}

	//------------------------------------------------------------------------------
	// ray gen program - the actual rendering happens in here
	//------------------------------------------------------------------------------
	extern "C" __global__ void __raygen__renderFrame()
	{
		const uint3 idx = optixGetLaunchIndex();
		const int d_idx = idx.y;
		const int p_idx = idx.x + optixLaunchParams.pointOffset;
		int* pointReachable = &optixLaunchParams.urpReachable.reachable[p_idx];
		int* selectedDirectionIdx = &optixLaunchParams.selectedDirectionIdx[p_idx];
		int* selectedToolSampleIdx = &optixLaunchParams.selectedToolSampleIdx[p_idx];
		if (*pointReachable != 0) return;

		const vec3f p = optixLaunchParams.urps[p_idx];
		const vec3f currentDir = optixLaunchParams.directions[d_idx];
		int numTool = optixLaunchParams.numToolSamplePoints;
		int numCarriagePoints = optixLaunchParams.numCarriagePoints;
		vec3f* rotatedToolByDir = optixLaunchParams.rotatedToolByDir + d_idx * numTool;
		vec3f* rotatedCarriageByDir = optixLaunchParams.rotatedCarriageByDir + d_idx * numCarriagePoints;

		// One thread now handles (point, direction) and scans all tool samples.
		for (int t_idx = 0; t_idx < numTool; ++t_idx)
		{
			if (*pointReachable != 0) return;

			const vec3f toolSample = rotatedToolByDir[t_idx];
			vec3f toolOrigin = p - toolSample;

			bool isToolCarrigeHit = false;
			for (int i = 0; i < numCarriagePoints; i++)
			{
						float hitDistance = 0.0f;
						uint32_t u0, u1;
						packPointer(&hitDistance, u0, u1);

						const vec3f rayPos = toolOrigin + rotatedCarriageByDir[i];
						optixTrace(optixLaunchParams.traversable,
							rayPos,
							currentDir,
						0.f,	  // tmin
						1e20f, // tmax
						0.0f,  // rayTime
						OptixVisibilityMask(255),
						OPTIX_RAY_FLAG_DISABLE_ANYHIT | OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT,
					SURFACE_RAY_TYPE,
					RAY_TYPE_COUNT,
					SURFACE_RAY_TYPE,
					u0, u1);

				const float hitDist = hitDistance;

				if (hitDist != -1.0f)
				{
					isToolCarrigeHit = true;
					break;
				}
			}
			if (isToolCarrigeHit) continue;

					float testHandleDistance = 0.0f;
					uint32_t handle0, handle1;
					packPointer(&testHandleDistance, handle0, handle1);
					optixTrace(optixLaunchParams.traversable,
						toolOrigin,
						currentDir,
					0.f,
					1e20f,
					0.0f,
					OptixVisibilityMask(255),
					OPTIX_RAY_FLAG_DISABLE_ANYHIT | OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT,
				SURFACE_RAY_TYPE,
				RAY_TYPE_COUNT,
				SURFACE_RAY_TYPE,
				handle0, handle1);
			if (testHandleDistance != -1.0f)
			{
				continue;
			}

			bool isToolHeadHit = false;
			for (int i = 0; i < numTool; i++)
			{
				if (t_idx == i) continue;
							float hitDistance = 0.0f;
							uint32_t u0, u1;
							packPointer(&hitDistance, u0, u1);

						const vec3f rayPos = toolOrigin + rotatedToolByDir[i];
						optixTrace(optixLaunchParams.traversable,
							rayPos,
							currentDir,
						0.f,
						1e20f,
						0.0f,
						OptixVisibilityMask(255),
						OPTIX_RAY_FLAG_DISABLE_ANYHIT,
					SURFACE_RAY_TYPE,
					RAY_TYPE_COUNT,
					SURFACE_RAY_TYPE,
					u0, u1);

				const float hitDist = hitDistance;
					if (hitDist != -1.0f && hitDist > optixLaunchParams.toolHeadHitTolerance)
					{
						isToolHeadHit = true;
						break;
				}
			}
					if (isToolHeadHit) continue;

					// Found a valid tool sample for this (point, direction).
					if (atomicCAS(pointReachable, 0, 1) == 0) {
						*selectedDirectionIdx = d_idx;
						*selectedToolSampleIdx = t_idx;
					}
					return;
				}
		}

} // ::osc
