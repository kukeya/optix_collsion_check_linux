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

#pragma once

#include "gdt/math/vec.h"
#include "optix7.h"

namespace osc {
	using namespace gdt;

	struct LaunchParams
	{
		struct {
			uint32_t* colorBuffer;
			vec2i size;
		} frame;

		struct {
			vec3f position;
			vec3f direction;
			vec3f horizontal;
			vec3f vertical;
		} camera;

		struct {
			uint32_t* hitID;
			vec2i size;
		} hitTool;

		struct
		{
			int* reachable;
			// x = urps.size(), y = directions.size(), z kept for compatibility.
			vec3i size;
		} urpReachable;

		OptixTraversableHandle traversable;

		vec3f pos;
		vec3f dir;
		vec3f* urps;
		int numUrps;
		vec3f* directions;
		int numDirections;
			vec3f* toolSample;
			int numToolSamplePoints;
			vec3f* carriage;
			int numCarriagePoints;
			int* selectedDirectionIdx;
			int* selectedToolSampleIdx;
			vec3f* rotatedToolByDir;
			vec3f* rotatedCarriageByDir;
			float toolHeadHitTolerance;
			int pointOffset;
		// int pointCount;
	};

} // ::osc
