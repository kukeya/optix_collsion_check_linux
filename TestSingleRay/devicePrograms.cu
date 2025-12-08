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
		//const int p_idx = idx.x;
		const int d_idx = idx.y;
		const int t_idx = idx.z;
		const int p_idx = idx.x + optixLaunchParams.pointOffset;
		//printf("(p_idx: %d, d_idx: %d, t_idx: %d)\n", p_idx, d_idx, t_idx);
		// 读取当前空间点、方向和刀头采样点
		const vec3f p = optixLaunchParams.urps[p_idx];
		const vec3f currentDir = optixLaunchParams.directions[d_idx];
		const vec3f toolSample = optixLaunchParams.toolSample[t_idx];


		// 以当前候选刀触点（toolSample）作为对齐点，计算刀具的“原点”
		// 旋转当前刀头采样点，使其与当前方向对齐
		vec3f rotatedToolSample;
		rotateToolPoint(toolSample, currentDir, rotatedToolSample);
		// 假设我们希望将当前候选点与 p 对齐，则刀头原点为：
		vec3f toolOrigin = p - rotatedToolSample;

		// 计算carriage的碰撞，先计算全局碰撞
		//printf("jump carriage!!!!!!!!!\n");
		int numCarriagePoints = optixLaunchParams.numCarriagePoints;
		vec3f* carriagePoints = optixLaunchParams.carriage;
		vec3f rotatedCarriagePoint;
		bool isToolCarrigeHit = false;
		for (int i = 0; i < numCarriagePoints; i++)
		{
			//int hitIdPRD = 0;
			float hitDistance = 0.0f;
			uint32_t u0, u1;
			packPointer(&hitDistance, u0, u1);

			rotateToolPoint(carriagePoints[i], currentDir, rotatedCarriagePoint);
			const vec3f rayPos = toolOrigin + rotatedCarriagePoint;
			optixTrace(optixLaunchParams.traversable,
				rayPos,
				currentDir,
				0.f,	  // tmin
				1e20f, // tmax
				0.0f,  // rayTime
				OptixVisibilityMask(255),
				OPTIX_RAY_FLAG_DISABLE_ANYHIT, // OPTIX_RAY_FLAG_NONE,
				SURFACE_RAY_TYPE,			  // SBT offset
				RAY_TYPE_COUNT,				  // SBT stride
				SURFACE_RAY_TYPE,
				u0, u1);

			//const int tmpID = int(hitIdPRD);
			//const uint32_t tmp = static_cast<uint32_t>(tmpID);
			const float hitDist = hitDistance;

			//如果tmp为0xFFFFFFFF，表示没有命中任何物体
			if (hitDist != -1.0f)
			{
				const int fbIndex = t_idx + d_idx * optixLaunchParams.urpReachable.size.z +
					p_idx * optixLaunchParams.urpReachable.size.y * optixLaunchParams.urpReachable.size.z;
				/*printf("Hit at fbIndex: %d (p_idx: %d, d_idx: %d, t_idx: %d), numTool: %d\n",
					fbIndex, p_idx, d_idx, t_idx, i);*/
				atomicAnd(&optixLaunchParams.urpReachable.reachable[fbIndex], 0);
				LOG("break carriage, %d,%d,%d,%f\n", t_idx, d_idx, p_idx, hitDist);
				
				isToolCarrigeHit = true;
				break;
			}
		}
		if (isToolCarrigeHit) return;

		// 刀头原点发射一条，如果碰撞说明刀柄一定碰撞
		bool isHandleHit = false;
		//int testHandlePRD = 0;
		float testHandleDistance = 0.0f;
		uint32_t handle0, handle1;
		packPointer(&testHandleDistance, handle0, handle1);
		optixTrace(optixLaunchParams.traversable,
			toolOrigin,
			currentDir,
			0.f,	  // tmin
			1e20f, // tmax
			0.0f,  // rayTime
			OptixVisibilityMask(255),
			OPTIX_RAY_FLAG_DISABLE_ANYHIT, // OPTIX_RAY_FLAG_NONE,
			SURFACE_RAY_TYPE,			  // SBT offset
			RAY_TYPE_COUNT,				  // SBT stride
			SURFACE_RAY_TYPE,
			handle0, handle1);
		/*const int handleHitID = int(testHandlePRD);
		const uint32_t handleHit = static_cast<uint32_t>(handleHitID);*/
		const float testHandleDist = testHandleDistance;

		//如果handleHit为0xFFFFFFFF，表示没有命中任何物体
		if (testHandleDist != -1.0f)
		{
			const int fbIndex = t_idx + d_idx * optixLaunchParams.urpReachable.size.z +
				p_idx * optixLaunchParams.urpReachable.size.y * optixLaunchParams.urpReachable.size.z;
			
			LOG("Hit handle fbIndex: %d (p_idx: %d, d_idx: %d, t_idx: %d, hitdist: %f)\n",
				fbIndex, p_idx, d_idx, t_idx, testHandleDist);
			
			atomicAnd(&optixLaunchParams.urpReachable.reachable[fbIndex], 0);
			isHandleHit = true;
		}
		if (isHandleHit) return;
		
		// 局部碰撞检测，刀头，过切无考虑
		int numTool = optixLaunchParams.numToolSamplePoints;
		vec3f* toolSamplePoints = optixLaunchParams.toolSample;
		for (int i = 0; i < numTool; i++)
		{
			if (t_idx == i) continue;
			//int hitIdPRD = 0;
			float hitDistance = 0.0f;
			uint32_t u0, u1;
			packPointer(&hitDistance, u0, u1);

			rotateToolPoint(toolSamplePoints[i], currentDir, rotatedToolSample);
			const vec3f rayPos = toolOrigin + rotatedToolSample;
			optixTrace(optixLaunchParams.traversable,
				rayPos,
				currentDir,
				0.f,	  // tmin
				1e20f, // tmax
				0.0f,  // rayTime
				OptixVisibilityMask(255),
				OPTIX_RAY_FLAG_DISABLE_ANYHIT, // OPTIX_RAY_FLAG_NONE,
				SURFACE_RAY_TYPE,			  // SBT offset
				RAY_TYPE_COUNT,				  // SBT stride
				SURFACE_RAY_TYPE,
				u0, u1);

			/*const int tmpID = int(hitIdPRD);
			const uint32_t tmp = static_cast<uint32_t>(tmpID);*/
			const float hitDist = hitDistance;

			//如果tmp为0xFFFFFFFF，表示没有命中任何物体, hitDist设置ball_r以内
			if (hitDist != -1.0f && hitDist > 0.015f)
			{
				LOG("hitDist, %f\n", hitDist);
				const int fbIndex = t_idx + d_idx * optixLaunchParams.urpReachable.size.z +
					p_idx * optixLaunchParams.urpReachable.size.y * optixLaunchParams.urpReachable.size.z;
				/*printf("Hit at fbIndex: %d (p_idx: %d, d_idx: %d, t_idx: %d), numTool: %d\n",
					fbIndex, p_idx, d_idx, t_idx, i);*/
				atomicAnd(&optixLaunchParams.urpReachable.reachable[fbIndex], 0);
				//isToolHeadHit = true;
				break;
			}
		}

	}

} // ::osc
