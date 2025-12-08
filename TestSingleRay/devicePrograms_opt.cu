//#define ENABLE_DEBUG_LOG

#ifdef ENABLE_DEBUG_LOG
#define LOG(msg, ...) printf("[LOG] " msg "\n", ##__VA_ARGS__)
#else
#define LOG(msg, ...) // no-op
#endif

#include <optix_device.h>
#include <cuda_runtime.h>
#include "optix7.h"
#include "LaunchParams.h"
#include <stdio.h>
using namespace osc;

namespace osc {

	/* launch params in constant memory */
	extern "C" __constant__ LaunchParams optixLaunchParams;

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

	// ------------------------------------------------------------------------------------------------
	// Utility: compute Rodrigues rotation parameters from defaultUp -> targetDir.
	// We compute these once per thread and reuse for rotating many tool points.
	// defaultUp is assumed (0,0,-1) (same as original code).
	// ------------------------------------------------------------------------------------------------
	static __forceinline__ __device__
		void compute_rodrigues_from_default_up(const vec3f& targetDir, vec3f& axisOut, float& sinT, float& cosT, int& specialCase)
	{
		// specialCase: 0 = normal, 1 = identity (same dir), 2 = 180deg flip (opposite)
		const vec3f defaultUp = vec3f(0.f, 0.f, -1.f);
		const float cosTheta = dot(defaultUp, targetDir);
		cosT = cosTheta;

		// clamp to valid range
		float c = cosTheta;
		if (c > 1.f) c = 1.f;
		if (c < -1.f) c = -1.f;

		// if parallel (same)
		if (c > 0.999999f) {
			specialCase = 1; // identity
			axisOut = vec3f(0.f);
			sinT = 0.f;
			return;
		}
		// if anti-parallel (opposite)
		if (c < -0.999999f) {
			specialCase = 2; // 180 deg rotate
			// choose any perpendicular axis to defaultUp, e.g., (1,0,0)
			axisOut = normalize(cross(defaultUp, vec3f(1.f, 0.f, 0.f)));
			// if cross produced zero (rare), use another axis
			if (length(axisOut) < 1e-6f) axisOut = normalize(cross(defaultUp, vec3f(0.f, 1.f, 0.f)));
			sinT = 0.f; // sin(180deg) == 0
			return;
		}
		// normal case
		specialCase = 0;
		vec3f axis = cross(defaultUp, targetDir);
		axisOut = axis / length(axis);
		// sin¦È = |axis_before_norm| = length(axis) / (|defaultUp| * |targetDir|) but defaultUp and targetDir assumed normalized
		float sinTheta = length(axis);
		sinT = sinTheta;
	}

	// apply Rodrigues rotation using precomputed axis, sinT, cosT, specialCase
	static __forceinline__ __device__
		vec3f apply_rodrigues(const vec3f& v, const vec3f& axis, float sinT, float cosT, int specialCase)
	{
		if (specialCase == 1) {
			// identity
			return v;
		}
		if (specialCase == 2) {
			// 180 degree rotation: v' = -v (since rotation around any perpendicular axis flips)
			return -v;
		}
		// Rodrigues formula: v*cosT + (axis x v)*sinT + axis*(axis¡¤v)*(1-cosT)
		vec3f term1 = v * cosT;
		vec3f term2 = cross(axis, v) * sinT;
		vec3f term3 = axis * (dot(axis, v) * (1.f - cosT));
		return term1 + term2 + term3;
	}

	//------------------------------------------------------------------------------ closesthit / anyhit / miss (unchanged logic, minimal)
	extern "C" __global__ void __closesthit__radiance()
	{
		const float hitDistance = optixGetRayTmax();
		float& prd = *(float*)getPRD<float>();
		prd = hitDistance;
	}

	extern "C" __global__ void __anyhit__radiance()
	{
		// left empty (we use disable anyhit flag for performance)
	}

	extern "C" __global__ void __miss__radiance()
	{
		float& prd = *(float*)getPRD<float>();
		prd = -1.0f;
	}

	//------------------------------------------------------------------------------ raygen optimized
	extern "C" __global__ void __raygen__renderFrame()
	{
		// Each thread corresponds to one (point, direction, toolSample)
		const uint3 idx = optixGetLaunchIndex();
		const int p_idx = idx.x + optixLaunchParams.pointOffset; // as original
		const int d_idx = idx.y;
		const int t_idx = idx.z;

		// load inputs
		const vec3f p = optixLaunchParams.urps[p_idx];
		const vec3f currentDir = optixLaunchParams.directions[d_idx];
		const vec3f toolSample = optixLaunchParams.toolSample[t_idx];

		// compute rotation parameters ONCE per thread (replaces repeated acos/sin/cos)
		vec3f axis;
		float sinT = 0.f, cosT = 0.f;
		int specialCase = 0;
		compute_rodrigues_from_default_up(currentDir, axis, sinT, cosT, specialCase);

		// rotate the chosen tool sample to align with currentDir
		vec3f rotatedToolSample = apply_rodrigues(toolSample, axis, sinT, cosT, specialCase);

		// compute tool origin such that rotatedToolSample aligns to p
		vec3f toolOrigin = p - rotatedToolSample;

		// prepare index into reachable buffer (unique per thread assumed)
		const int fbIndex = t_idx + d_idx * optixLaunchParams.urpReachable.size.z +
			p_idx * optixLaunchParams.urpReachable.size.y * optixLaunchParams.urpReachable.size.z;

		// local reachable flag (avoid early return -> reduce warp divergence)
		bool isReachable = true;

		// ------------------------------
		// 1) carriage checks: loop over carriage points
		//    We still must test all carriage points (unless you make carriage a launch dim)
		//    but we avoid early return and re-use rotation parameters.
		// ------------------------------
		const int numCarriagePoints = optixLaunchParams.numCarriagePoints;
		const vec3f* __restrict__ carriagePoints = optixLaunchParams.carriage;
		// Note: hitDistance is per-trace PRD written by closesthit/miss
		for (int i = 0; i < numCarriagePoints; ++i)
		{
			// compute rotated carriage point via previously computed rodrigues params
			vec3f rotatedCarriagePoint = apply_rodrigues(carriagePoints[i], axis, sinT, cosT, specialCase);
			const vec3f rayPos = toolOrigin + rotatedCarriagePoint;

			// setup payload to receive hitDistance
			float hitDistance = 0.0f;
			uint32_t u0, u1;
			packPointer(&hitDistance, u0, u1);

			optixTrace(
				optixLaunchParams.traversable,
				rayPos,
				currentDir,
				0.0f,      // tmin
				1e20f,     // tmax
				0.0f,      // rayTime
				OptixVisibilityMask(255),
				OPTIX_RAY_FLAG_DISABLE_ANYHIT,
				SURFACE_RAY_TYPE, RAY_TYPE_COUNT, SURFACE_RAY_TYPE,
				u0, u1);

			// if hitDistance != -1.0 => hit
			if (hitDistance != -1.0f) {
				isReachable = false;
				// we can break here if we want to avoid extra traces for this section:
				// but to further reduce divergence between threads within a warp,
				// prefer to break only this small loop (no returns).
				break;
			}
		}

		// If carriage collision detected, mark unreachable and skip other tests
		if (!isReachable) {
			// write result directly (non-atomic) because fbIndex is unique per thread
			// If fbIndex is not unique across threads in your launch configuration,
			// then replace this write with an atomic operation (atomicAnd or atomicExch).
			optixLaunchParams.urpReachable.reachable[fbIndex] = 0;
			return;
		}

		// ------------------------------
		// 2) handle check: one trace from toolOrigin in currentDir
		// ------------------------------
		{
			float handleHitDist = 0.0f;
			uint32_t h0, h1;
			packPointer(&handleHitDist, h0, h1);
			optixTrace(
				optixLaunchParams.traversable,
				toolOrigin,
				currentDir,
				0.0f,
				1e20f,
				0.0f,
				OptixVisibilityMask(255),
				OPTIX_RAY_FLAG_DISABLE_ANYHIT,
				SURFACE_RAY_TYPE, RAY_TYPE_COUNT, SURFACE_RAY_TYPE,
				h0, h1);

			if (handleHitDist != -1.0f) {
				isReachable = false;
			}
		}

		if (!isReachable) {
			optixLaunchParams.urpReachable.reachable[fbIndex] = 0;
			return;
		}

		// ------------------------------
		// 3) local tool-head checks: test other tool sample points (we reuse rotation params)
		//    We intentionally skip t_idx itself (it's the aligned point)
		// ------------------------------
		const int numTool = optixLaunchParams.numToolSamplePoints;
		const vec3f* __restrict__ toolSamplePoints = optixLaunchParams.toolSample;

		for (int i = 0; i < numTool; ++i)
		{
			if (i == t_idx) continue; // don't test the same sample point
			vec3f rotated = apply_rodrigues(toolSamplePoints[i], axis, sinT, cosT, specialCase);
			const vec3f rayPos = toolOrigin + rotated;

			float hitDistance = 0.0f;
			uint32_t u0, u1;
			packPointer(&hitDistance, u0, u1);
			optixTrace(
				optixLaunchParams.traversable,
				rayPos,
				currentDir,
				0.0f,
				1e20f,
				0.0f,
				OptixVisibilityMask(255),
				OPTIX_RAY_FLAG_DISABLE_ANYHIT,
				SURFACE_RAY_TYPE, RAY_TYPE_COUNT, SURFACE_RAY_TYPE,
				u0, u1);

			// original code used threshold hitDist > 1.5; keep same heuristic
			if (hitDistance != -1.0f && hitDistance > 0.015f) {
				isReachable = false;
				break;
			}
		}

		// final write
		if (!isReachable) {
			optixLaunchParams.urpReachable.reachable[fbIndex] = 0;
		}
		// else keep existing value (assumed initialized as reachable)
	}

} // namespace osc
