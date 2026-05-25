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

#include "SampleRenderer.h"
#include "SamplingUtils.h"
#include "utils.h"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace osc {

	static float defaultScaleFactor()
	{
		return 1.0f / 35.0f;
	}

	static float defaultToolRadius()
	{
		return 1.5f;
	}

		struct CollisionRequest {
			std::string modelFile;
			std::string urpFile;
			std::string outFile;
			std::string debugToolOutFile;
			float scaleFactor;
			float toolRadius;
			float toolHeadHitToleranceOverride;
			DirectionSamplingConfig directionSampling;
			CarriageSamplingConfig carriageSampling;
			int pruneBottomZMinMode;
			int debugExportPointIndex;
			std::string requestId;

				CollisionRequest()
					: scaleFactor(defaultScaleFactor())
					, toolRadius(defaultToolRadius())
					, toolHeadHitToleranceOverride(-1.0f)
					, directionSampling()
				, carriageSampling()
				, pruneBottomZMinMode(-1)
				, debugExportPointIndex(-1)
			{
			}
		};

		struct CollisionResult {
			int unreachableCount = 0;
			double elapsedMs = 0.0;
			std::string indicesFile;
			std::string debugToolFile;
		};

	static inline bool fileExists(const std::string& path)
	{
		std::ifstream f(path.c_str());
		return f.good();
	}

	static inline std::string trim(const std::string& s)
	{
		size_t b = 0;
		while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
		size_t e = s.size();
		while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
		return s.substr(b, e - b);
	}

	static bool extractJsonString(const std::string& line, const std::string& key, std::string& out)
	{
		const std::string token = "\"" + key + "\"";
		size_t pos = line.find(token);
		if (pos == std::string::npos) return false;
		pos = line.find(':', pos + token.size());
		if (pos == std::string::npos) return false;
		++pos;
		while (pos < line.size() && std::isspace(static_cast<unsigned char>(line[pos]))) ++pos;
		if (pos >= line.size() || line[pos] != '"') return false;
		++pos;

		std::string value;
		value.reserve(64);
		while (pos < line.size()) {
			char c = line[pos++];
			if (c == '\\') {
				if (pos < line.size()) value.push_back(line[pos++]);
				continue;
			}
			if (c == '"') {
				out = value;
				return true;
			}
			value.push_back(c);
		}
		return false;
	}

	static bool extractJsonFloat(const std::string& line, const std::string& key, float& out)
	{
		const std::string token = "\"" + key + "\"";
		size_t pos = line.find(token);
		if (pos == std::string::npos) return false;
		pos = line.find(':', pos + token.size());
		if (pos == std::string::npos) return false;
		++pos;
		while (pos < line.size() && std::isspace(static_cast<unsigned char>(line[pos]))) ++pos;
		if (pos >= line.size()) return false;

		char* endPtr = nullptr;
		const char* beginPtr = line.c_str() + pos;
		const float value = std::strtof(beginPtr, &endPtr);
		if (endPtr == beginPtr) return false;
		out = value;
		return true;
	}

	static bool extractJsonInt(const std::string& line, const std::string& key, int& out)
	{
		const std::string token = "\"" + key + "\"";
		size_t pos = line.find(token);
		if (pos == std::string::npos) return false;
		pos = line.find(':', pos + token.size());
		if (pos == std::string::npos) return false;
		++pos;
		while (pos < line.size() && std::isspace(static_cast<unsigned char>(line[pos]))) ++pos;
		if (pos >= line.size()) return false;

		char* endPtr = nullptr;
		const char* beginPtr = line.c_str() + pos;
		const long value = std::strtol(beginPtr, &endPtr, 10);
		if (endPtr == beginPtr) return false;
		out = static_cast<int>(value);
		return true;
	}

	static bool extractJsonBool(const std::string& line, const std::string& key, bool& out)
	{
		const std::string token = "\"" + key + "\"";
		size_t pos = line.find(token);
		if (pos == std::string::npos) return false;
		pos = line.find(':', pos + token.size());
		if (pos == std::string::npos) return false;
		++pos;
		while (pos < line.size() && std::isspace(static_cast<unsigned char>(line[pos]))) ++pos;
		if (pos >= line.size()) return false;

		if (line.compare(pos, 4, "true") == 0) {
			out = true;
			return true;
		}
		if (line.compare(pos, 5, "false") == 0) {
			out = false;
			return true;
		}
		if (line[pos] == '1') {
			out = true;
			return true;
		}
		if (line[pos] == '0') {
			out = false;
			return true;
		}
		return false;
	}

	static bool parseBoolArg(const std::string& raw, bool& out)
	{
		std::string value = raw;
		for (size_t i = 0; i < value.size(); ++i) {
			value[i] = static_cast<char>(std::tolower(static_cast<unsigned char>(value[i])));
		}
		if (value == "1" || value == "true" || value == "yes" || value == "on") {
			out = true;
			return true;
		}
		if (value == "0" || value == "false" || value == "no" || value == "off") {
			out = false;
			return true;
		}
		return false;
	}

	static std::string jsonEscape(const std::string& in)
	{
		std::string out;
		out.reserve(in.size() + 8);
		for (size_t i = 0; i < in.size(); ++i) {
			const char c = in[i];
			if (c == '\\' || c == '"') {
				out.push_back('\\');
				out.push_back(c);
			}
			else if (c == '\n') {
				out += "\\n";
			}
			else if (c == '\r') {
				out += "\\r";
			}
			else {
				out.push_back(c);
			}
		}
		return out;
	}

	static bool parseRequestJson(const std::string& line, CollisionRequest& req, std::string& err)
	{
		if (!extractJsonString(line, "model_file", req.modelFile)) {
			err = "missing or invalid field: model_file";
			return false;
		}
		if (!extractJsonString(line, "urps_file", req.urpFile)) {
			err = "missing or invalid field: urps_file";
			return false;
		}
		if (!extractJsonString(line, "output_file", req.outFile)) {
			err = "missing or invalid field: output_file";
			return false;
		}
			req.scaleFactor = defaultScaleFactor();
			(void)extractJsonFloat(line, "factor", req.scaleFactor);
			req.toolRadius = defaultToolRadius();
			(void)extractJsonFloat(line, "tool_radius", req.toolRadius);
			(void)extractJsonFloat(line, "tool_head_hit_tolerance", req.toolHeadHitToleranceOverride);
			(void)extractJsonInt(line, "direction_count", req.directionSampling.goldenPointCount);
		(void)extractJsonFloat(line, "direction_min_z", req.directionSampling.hemisphereMinZ);
		(void)extractJsonInt(line, "carriage_sample_count", req.carriageSampling.sampleCount);
		(void)extractJsonFloat(line, "carriage_inner_radius", req.carriageSampling.innerRadius);
			(void)extractJsonFloat(line, "carriage_outer_radius", req.carriageSampling.outerRadius);
			(void)extractJsonFloat(line, "carriage_z", req.carriageSampling.z);
			(void)extractJsonInt(line, "debug_export_point_index", req.debugExportPointIndex);
			(void)extractJsonString(line, "debug_tool_output_file", req.debugToolOutFile);
			std::string samplingMode;
		if (extractJsonString(line, "sampling_mode", samplingMode)) {
			if (!parseDirectionSamplingMode(samplingMode, req.directionSampling.mode)) {
				err = "invalid field value: sampling_mode";
				return false;
			}
		}
		bool pruneBottomZMin = false;
		if (extractJsonBool(line, "prune_bottom_zmin", pruneBottomZMin)) {
			req.pruneBottomZMinMode = pruneBottomZMin ? 1 : 0;
		}
		req.requestId.clear();
		(void)extractJsonString(line, "request_id", req.requestId);
		return true;
	}

	struct SampleIntersection {
		SampleIntersection(const TriangleMesh& model,
			const std::vector<vec3f>& checkPoints,
			const std::vector<vec3f>& directions,
			const std::vector<vec3f>& toolSamplePoints,
			const std::vector<vec3f>& carriage,
			const std::vector<vec3f>& rotatedToolByDir,
			const std::vector<vec3f>& rotatedCarriageByDir,
			float toolHeadHitTolerance)
			: sample(model)
		{
			setToolConfig(directions, toolSamplePoints, carriage, rotatedToolByDir, rotatedCarriageByDir, toolHeadHitTolerance);
			setCheckPoints(checkPoints);
		}

		void setModel(const TriangleMesh& model)
		{
			sample.setModel(model);
		}

		void setToolConfig(const std::vector<vec3f>& newDirections,
			const std::vector<vec3f>& newTool,
			const std::vector<vec3f>& newCarriage,
			const std::vector<vec3f>& newRotatedToolByDir,
			const std::vector<vec3f>& newRotatedCarriageByDir,
			float newToolHeadHitTolerance)
		{
			directions = newDirections;
			toolSamplePoints = newTool;
			carriage = newCarriage;
			rotatedToolByDir = newRotatedToolByDir;
			rotatedCarriageByDir = newRotatedCarriageByDir;
			toolHeadHitTolerance = newToolHeadHitTolerance;
			resize(vec3i((int)checkPoints.size(), (int)directions.size(), 1));
		}

		void setCheckPoints(const std::vector<vec3f>& newCheckPoints)
		{
			checkPoints = newCheckPoints;
			resize(vec3i((int)checkPoints.size(), (int)directions.size(), 1));
		}

		void render()
		{
			sample.uploadArray(checkPoints, directions, toolSamplePoints, carriage, rotatedToolByDir, rotatedCarriageByDir, toolHeadHitTolerance);
			sample.render();
		}

			void resize(const vec3i& newSize)
			{
				fbSize = newSize;
				sample.resize(newSize);
				reachable.resize(newSize.x);
				selectedDirectionIdx.resize(newSize.x);
				selectedToolSampleIdx.resize(newSize.x);
			}

		vec3i                 fbSize;
		SampleRenderer        sample;
			std::vector<int>      reachable;
			std::vector<int>      selectedDirectionIdx;
			std::vector<int>      selectedToolSampleIdx;
			std::vector<vec3f>    checkPoints;
		std::vector<vec3f>    directions;
		std::vector<vec3f>    toolSamplePoints;
		std::vector<vec3f>    carriage;
		std::vector<vec3f>    rotatedToolByDir;
		std::vector<vec3f>    rotatedCarriageByDir;
		float                 toolHeadHitTolerance = 0.0f;
	};

	class CollisionEngine {
	public:
		bool runRequest(const CollisionRequest& req, CollisionResult& result, std::string& err)
		{
			auto start = std::chrono::steady_clock::now();
			if (!loadStaticAssets(err)) return false;
			if (req.scaleFactor <= 0.0f) {
				err = "scale factor must be positive";
				return false;
			}
			if (req.toolRadius <= 0.0f) {
				err = "tool radius must be positive";
				return false;
			}
			if (!fileExists(req.modelFile)) {
				err = "model file not found: " + req.modelFile;
				return false;
			}
			if (!fileExists(req.urpFile)) {
				err = "urps file not found: " + req.urpFile;
				return false;
			}

			TriangleMesh model;
			std::string modelFileCopy = req.modelFile;
			model.loadOFF(modelFileCopy);
			if (model.vertex.empty() || model.index.empty()) {
				err = "failed to load model or model is empty: " + req.modelFile;
				return false;
			}

			std::vector<vec3f> urps;
			loadPointsOFF(req.urpFile, urps);
			std::cout << "load urps size:  " << urps.size() << std::endl;

			const std::vector<vec3f> directions = generateDirectionSamples(req.directionSampling);
			if (directions.empty()) {
				err = "failed to generate directions";
				return false;
			}
			const std::vector<vec3f> baseCarriagePoints = generateCarriageSamples(req.carriageSampling);
			if (baseCarriagePoints.empty()) {
				err = "failed to generate carriage samples";
				return false;
			}
			std::cout << "sampling mode: " << directionSamplingModeName(req.directionSampling.mode)
				<< ", directions: " << directions.size()
				<< ", carriage samples: " << baseCarriagePoints.size() << std::endl;

			std::vector<vec3f> tool;
			std::vector<vec3f> carriage;
			std::vector<vec3f> rotatedToolByDir;
			std::vector<vec3f> rotatedCarriageByDir;
			float toolHeadHitTolerance = 0.0f;
				prepareScaledTooling(directions,
					baseCarriagePoints,
					req.toolRadius,
					req.scaleFactor,
					req.toolHeadHitToleranceOverride,
					tool,
					carriage,
				rotatedToolByDir,
				rotatedCarriageByDir,
				toolHeadHitTolerance);
			std::cout << "tool radius: " << req.toolRadius
				<< ", scale factor: " << req.scaleFactor
				<< ", tool-head hit tolerance: " << toolHeadHitTolerance << std::endl;

			if (!sampleInterMachine) {
				sampleInterMachine.reset(new SampleIntersection(model,
					std::vector<vec3f>(),
					directions,
					tool,
					carriage,
					rotatedToolByDir,
					rotatedCarriageByDir,
					toolHeadHitTolerance));
				loadedModelFile = req.modelFile;
			}
			else if (loadedModelFile != req.modelFile) {
				sampleInterMachine->setModel(model);
				loadedModelFile = req.modelFile;
			}
			sampleInterMachine->setToolConfig(directions,
				tool,
				carriage,
				rotatedToolByDir,
				rotatedCarriageByDir,
				toolHeadHitTolerance);

				const size_t batchSize = std::min<size_t>(urps.size(), 50000);
				std::vector<bool> urpsReachable(urps.size(), false);
				std::vector<int> selectedDirectionGlobal(urps.size(), -1);
				std::vector<int> selectedToolGlobal(urps.size(), -1);
				for (size_t batchStart = 0; batchStart < urps.size(); batchStart += batchSize) {
					size_t batchEnd = std::min(batchStart + batchSize, urps.size());
					std::vector<vec3f> urpsBatch(urps.begin() + batchStart, urps.begin() + batchEnd);

				std::cout << "处理批次: " << batchStart << " - " << batchEnd << std::endl;
					sampleInterMachine->setCheckPoints(urpsBatch);
					sampleInterMachine->render();
					sampleInterMachine->sample.downloadReachable(sampleInterMachine->reachable.data());
					sampleInterMachine->sample.downloadSelectedPose(sampleInterMachine->selectedDirectionIdx.data(),
						sampleInterMachine->selectedToolSampleIdx.data());

					for (size_t i = 0; i < urpsBatch.size(); ++i) {
						urpsReachable[batchStart + i] = (sampleInterMachine->reachable[i] != 0);
						selectedDirectionGlobal[batchStart + i] = sampleInterMachine->selectedDirectionIdx[i];
						selectedToolGlobal[batchStart + i] = sampleInterMachine->selectedToolSampleIdx[i];
					}
				}

				if (req.debugExportPointIndex >= 0) {
					const size_t debugPointIndex = static_cast<size_t>(req.debugExportPointIndex);
					if (debugPointIndex >= urps.size()) {
						err = "debug_export_point_index out of range";
						return false;
					}
					if (!urpsReachable[debugPointIndex]) {
						std::cout << "debug export skipped: point " << req.debugExportPointIndex
							<< " is unreachable under current settings" << std::endl;
					}
					else {
						const int dirIdx = selectedDirectionGlobal[debugPointIndex];
						const int toolIdx = selectedToolGlobal[debugPointIndex];
						if (dirIdx < 0 || toolIdx < 0
							|| dirIdx >= (int)directions.size()
							|| toolIdx >= (int)tool.size()) {
							err = "debug export failed: reachable point missing pose indices";
							return false;
						}
						const std::string debugToolOutFile = req.debugToolOutFile.empty()
							? (req.outFile + ".tool_debug.obj")
							: req.debugToolOutFile;
						outputToolModel(urps[debugPointIndex],
							directions[dirIdx],
							tool[toolIdx],
							tool,
							carriage,
							debugToolOutFile);
						result.debugToolFile = debugToolOutFile;
						std::cout << "已导出可达刀具姿态: point=" << req.debugExportPointIndex
							<< ", direction_idx=" << dirIdx
							<< ", tool_sample_idx=" << toolIdx
							<< ", file=" << debugToolOutFile << std::endl;
					}
				}

			std::vector<vec3f> unreachablePoints;
			std::vector<int> unreachableIndices;
			unreachablePoints.reserve(urps.size());
			unreachableIndices.reserve(urps.size());
			for (size_t i = 0; i < urpsReachable.size(); ++i) {
				if (!urpsReachable[i]) {
					unreachablePoints.push_back(urps[i]);
					unreachableIndices.push_back((int)i);
				}
			}

			const bool pruneBottomZMin = resolvePruneBottomZMin(req);
			std::vector<vec3f> filteredUnreachablePoints;
			std::vector<int> filteredUnreachableIndices;
			filterBottomZPoints(unreachablePoints,
				unreachableIndices,
				computeModelZMin(model),
				pruneBottomZMin,
				filteredUnreachablePoints,
				filteredUnreachableIndices);
			if (pruneBottomZMin) {
				std::cout << "bottom-z pruning enabled, kept "
					<< filteredUnreachablePoints.size() << " / " << unreachablePoints.size()
					<< " unreachable points" << std::endl;
			}

			writeOBJwithPoints(req.outFile, filteredUnreachablePoints);
			std::cout << "已生成不可达点输出文件: " << req.outFile << " " << filteredUnreachablePoints.size() << std::endl;

			const std::string indexOutFile = req.outFile + ".indices.txt";
			std::ofstream idxFile(indexOutFile.c_str(), std::ios::out | std::ios::trunc);
			if (!idxFile.is_open()) {
				err = "failed to create index file: " + indexOutFile;
				return false;
			}
			for (size_t i = 0; i < filteredUnreachableIndices.size(); ++i) {
				idxFile << filteredUnreachableIndices[i] << "\n";
			}
			idxFile.close();
			std::cout << "已生成不可达点索引文件: " << indexOutFile << std::endl;

			auto end = std::chrono::steady_clock::now();
			result.unreachableCount = (int)filteredUnreachablePoints.size();
			result.indicesFile = indexOutFile;
			result.elapsedMs = std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(end - start).count();
			std::cout << "耗时: " << (int)(result.elapsedMs / 1000.0) << " 秒" << std::endl;
			return true;
		}

	private:
		bool loadStaticAssets(std::string& err)
		{
			if (staticAssetsLoaded) return true;

			const std::string toolFile = std::string(PROJECT_ROOT_DIR) + "/mesh/tools/ToolHeadSample_r1.off";
			loadPointsOFF(toolFile, baseToolPoints);
			if (baseToolPoints.empty()) {
				err = "failed to load tool assets";
				return false;
			}

			staticAssetsLoaded = true;
			return true;
		}

			void prepareScaledTooling(const std::vector<vec3f>& directions,
				const std::vector<vec3f>& baseCarriagePoints,
				float toolRadius,
				float scaleFactor,
				float toolHeadHitToleranceOverride,
				std::vector<vec3f>& tool,
				std::vector<vec3f>& carriage,
			std::vector<vec3f>& rotatedToolByDir,
			std::vector<vec3f>& rotatedCarriageByDir,
			float& toolHeadHitTolerance)
		{
				tool = baseToolPoints;
				carriage = baseCarriagePoints;
				const float toolScale = scaleFactor * toolRadius;
				const float carriageScale = scaleFactor;
				toolHeadHitTolerance = toolHeadHitToleranceOverride >= 0.0f
					? toolHeadHitToleranceOverride
					: toolScale;

			for (size_t i = 0; i < tool.size(); ++i) tool[i] *= toolScale;
			for (size_t i = 0; i < carriage.size(); ++i) carriage[i] *= carriageScale;

			rotatedToolByDir.resize(directions.size() * tool.size());
			rotatedCarriageByDir.resize(directions.size() * carriage.size());
			for (size_t d = 0; d < directions.size(); ++d) {
				for (size_t t = 0; t < tool.size(); ++t) {
					rotateToolPoint(tool[t], directions[d], rotatedToolByDir[d * tool.size() + t]);
				}
				for (size_t c = 0; c < carriage.size(); ++c) {
					rotateToolPoint(carriage[c], directions[d], rotatedCarriageByDir[d * carriage.size() + c]);
				}
			}
		}

		bool resolvePruneBottomZMin(const CollisionRequest& req) const
		{
			if (req.pruneBottomZMinMode >= 0) return req.pruneBottomZMinMode != 0;
			return defaultPruneBottomZMin(req.directionSampling.mode);
		}

		float computeModelZMin(const TriangleMesh& model) const
		{
			if (model.vertex.empty()) return 0.0f;
			float zMin = model.vertex.front().z;
			for (size_t i = 1; i < model.vertex.size(); ++i) {
				zMin = std::min(zMin, model.vertex[i].z);
			}
			return zMin;
		}

		bool staticAssetsLoaded = false;
		std::vector<vec3f> baseToolPoints;
		std::unique_ptr<SampleIntersection> sampleInterMachine;
		std::string loadedModelFile;
	};

	static int runServerMode()
	{
		std::streambuf* protocolOutBuf = std::cout.rdbuf();
		std::cout.rdbuf(std::cerr.rdbuf());
		std::ostream protocolOut(protocolOutBuf);

		CollisionEngine engine;
		std::string line;
		while (std::getline(std::cin, line)) {
			line = trim(line);
			if (line.empty()) continue;

			CollisionRequest req;
			std::string parseErr;
			if (!parseRequestJson(line, req, parseErr)) {
				std::string reqId;
				(void)extractJsonString(line, "request_id", reqId);
				protocolOut << "{\"ok\":false,\"request_id\":\"" << jsonEscape(reqId)
					<< "\",\"error\":\"" << jsonEscape(parseErr) << "\"}" << std::endl;
				continue;
			}

			CollisionResult result;
			std::string err;
			const bool ok = engine.runRequest(req, result, err);
				if (ok) {
					protocolOut << "{\"ok\":true,\"request_id\":\"" << jsonEscape(req.requestId)
						<< "\",\"indices_file\":\"" << jsonEscape(result.indicesFile)
						<< "\",\"unreachable_count\":" << result.unreachableCount
						<< ",\"elapsed_ms\":" << result.elapsedMs;
					if (!result.debugToolFile.empty()) {
						protocolOut << ",\"debug_tool_file\":\"" << jsonEscape(result.debugToolFile) << "\"";
					}
					protocolOut << "}" << std::endl;
				}
			else {
				protocolOut << "{\"ok\":false,\"request_id\":\"" << jsonEscape(req.requestId)
					<< "\",\"error\":\"" << jsonEscape(err) << "\"}" << std::endl;
			}
		}
		return 0;
	}

	static int runCliOnce(int ac, char** av)
	{
		CollisionRequest req;
		req.modelFile = std::string(PROJECT_ROOT_DIR) + "/mesh/exp1/ruyi_recur1_n.off";
		req.urpFile = std::string(PROJECT_ROOT_DIR) + "/mesh/exp1/ruyi_recur1_urps_n.off";
		req.outFile = std::string(PROJECT_ROOT_DIR) + "/mesh/exp1/ruyi_recur1_output_collision.obj";
		req.scaleFactor = defaultScaleFactor();

		if (ac > 1) req.modelFile = av[1];
		if (ac > 2) req.urpFile = av[2];
		if (ac > 3) req.outFile = av[3];
		if (ac > 4) req.scaleFactor = std::stof(av[4]);
		if (ac > 5) {
			if (!parseDirectionSamplingMode(av[5], req.directionSampling.mode)) {
				throw std::runtime_error("invalid sampling_mode, expected axis_5, axis_9, golden_hemisphere or legacy_66_sphere");
			}
		}
				if (ac > 6) {
					bool pruneBottomZMin = false;
					if (!parseBoolArg(av[6], pruneBottomZMin)) {
						throw std::runtime_error("invalid prune_bottom_zmin, expected true/false/1/0");
					}
					req.pruneBottomZMinMode = pruneBottomZMin ? 1 : 0;
				}
				if (ac > 7) req.debugExportPointIndex = std::stoi(av[7]);
				if (ac > 8) req.debugToolOutFile = av[8];
				if (ac > 9) req.toolHeadHitToleranceOverride = std::stof(av[9]);

			if (ac < 7) {
				std::cout << "未检测到全部命令行参数，使用默认/补全后的配置" << std::endl;
			std::cout << "模型文件: " << req.modelFile << std::endl;
			std::cout << "URP文件: " << req.urpFile << std::endl;
			std::cout << "输出文件: " << req.outFile << std::endl;
			std::cout << "缩放比例: " << req.scaleFactor << std::endl;
					std::cout << "采样模式: " << directionSamplingModeName(req.directionSampling.mode) << std::endl;
					std::cout << "底部z_min过滤: " << (req.pruneBottomZMinMode >= 0
						? (req.pruneBottomZMinMode != 0 ? "true" : "false")
						: (defaultPruneBottomZMin(req.directionSampling.mode) ? "auto(true)" : "auto(false)")) << std::endl;
					if (req.toolHeadHitToleranceOverride >= 0.0f) {
						std::cout << "覆盖 tool-head hit tolerance: " << req.toolHeadHitToleranceOverride << std::endl;
					}
					if (req.debugExportPointIndex >= 0) {
						std::cout << "调试导出点索引: " << req.debugExportPointIndex << std::endl;
					}
			}

		CollisionEngine engine;
		CollisionResult result;
		std::string err;
		if (!engine.runRequest(req, result, err)) {
			throw std::runtime_error(err);
		}
		return 0;
	}

	extern "C" int main(int ac, char** av)
	{
		try {
			if (ac > 1 && std::string(av[1]) == "--server") {
				return runServerMode();
			}
			return runCliOnce(ac, av);
		}
		catch (std::runtime_error& e) {
			std::cerr << GDT_TERMINAL_RED << "FATAL ERROR: " << e.what()
				<< GDT_TERMINAL_DEFAULT << std::endl;
			return 1;
		}
	}

} // namespace osc
