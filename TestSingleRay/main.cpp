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

	struct Cutter {
		float ball_r = 1.5f;
		float scale_factor = 0.01f;
	};

	struct CollisionRequest {
		std::string modelFile;
		std::string urpFile;
		std::string outFile;
		float scaleFactor = 0.01f;
		std::string requestId;
	};

	struct CollisionResult {
		int unreachableCount = 0;
		double elapsedMs = 0.0;
		std::string indicesFile;
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
		req.scaleFactor = 0.01f;
		(void)extractJsonFloat(line, "factor", req.scaleFactor);
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
			const std::vector<vec3f>& rotatedCarriageByDir)
			: sample(model)
		{
			setToolConfig(directions, toolSamplePoints, carriage, rotatedToolByDir, rotatedCarriageByDir);
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
			const std::vector<vec3f>& newRotatedCarriageByDir)
		{
			directions = newDirections;
			toolSamplePoints = newTool;
			carriage = newCarriage;
			rotatedToolByDir = newRotatedToolByDir;
			rotatedCarriageByDir = newRotatedCarriageByDir;
			resize(vec3i((int)checkPoints.size(), (int)directions.size(), 1));
		}

		void setCheckPoints(const std::vector<vec3f>& newCheckPoints)
		{
			checkPoints = newCheckPoints;
			resize(vec3i((int)checkPoints.size(), (int)directions.size(), 1));
		}

		void render()
		{
			sample.uploadArray(checkPoints, directions, toolSamplePoints, carriage, rotatedToolByDir, rotatedCarriageByDir);
			sample.render();
		}

		void resize(const vec3i& newSize)
		{
			fbSize = newSize;
			sample.resize(newSize);
			reachable.resize(newSize.x);
		}

		vec3i                 fbSize;
		SampleRenderer        sample;
		std::vector<int>      reachable;
		std::vector<vec3f>    checkPoints;
		std::vector<vec3f>    directions;
		std::vector<vec3f>    toolSamplePoints;
		std::vector<vec3f>    carriage;
		std::vector<vec3f>    rotatedToolByDir;
		std::vector<vec3f>    rotatedCarriageByDir;
	};

	class CollisionEngine {
	public:
		bool runRequest(const CollisionRequest& req, CollisionResult& result, std::string& err)
		{
			auto start = std::chrono::steady_clock::now();
			if (!loadStaticAssets(err)) return false;
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

			std::vector<vec3f> tool;
			std::vector<vec3f> carriage;
			std::vector<vec3f> rotatedToolByDir;
			std::vector<vec3f> rotatedCarriageByDir;
			prepareScaledTooling(req.scaleFactor, tool, carriage, rotatedToolByDir, rotatedCarriageByDir);

			if (!sampleInterMachine) {
				sampleInterMachine.reset(new SampleIntersection(model,
					std::vector<vec3f>(),
					directions,
					tool,
					carriage,
					rotatedToolByDir,
					rotatedCarriageByDir));
				loadedModelFile = req.modelFile;
			}
			else if (loadedModelFile != req.modelFile) {
				sampleInterMachine->setModel(model);
				loadedModelFile = req.modelFile;
			}
			sampleInterMachine->setToolConfig(directions, tool, carriage, rotatedToolByDir, rotatedCarriageByDir);

			const size_t batchSize = std::min<size_t>(urps.size(), 50000);
			std::vector<bool> urpsReachable(urps.size(), false);
			for (size_t batchStart = 0; batchStart < urps.size(); batchStart += batchSize) {
				size_t batchEnd = std::min(batchStart + batchSize, urps.size());
				std::vector<vec3f> urpsBatch(urps.begin() + batchStart, urps.begin() + batchEnd);

				std::cout << "处理批次: " << batchStart << " - " << batchEnd << std::endl;
				sampleInterMachine->setCheckPoints(urpsBatch);
				sampleInterMachine->render();
				sampleInterMachine->sample.downloadReachable(sampleInterMachine->reachable.data());

				for (size_t i = 0; i < urpsBatch.size(); ++i) {
					urpsReachable[batchStart + i] = (sampleInterMachine->reachable[i] != 0);
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

			writeOBJwithPoints(req.outFile, unreachablePoints);
			std::cout << "已生成不可达点输出文件: " << req.outFile << " " << unreachablePoints.size() << std::endl;

			const std::string indexOutFile = req.outFile + ".indices.txt";
			std::ofstream idxFile(indexOutFile.c_str(), std::ios::out | std::ios::trunc);
			if (!idxFile.is_open()) {
				err = "failed to create index file: " + indexOutFile;
				return false;
			}
			for (size_t i = 0; i < unreachableIndices.size(); ++i) {
				idxFile << unreachableIndices[i] << "\n";
			}
			idxFile.close();
			std::cout << "已生成不可达点索引文件: " << indexOutFile << std::endl;

			auto end = std::chrono::steady_clock::now();
			result.unreachableCount = (int)unreachablePoints.size();
			result.indicesFile = indexOutFile;
			result.elapsedMs = std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(end - start).count();
			std::cout << "耗时: " << (int)(result.elapsedMs / 1000.0) << " 秒" << std::endl;
			return true;
		}

	private:
		bool loadStaticAssets(std::string& err)
		{
			if (staticAssetsLoaded) return true;

			const std::string directionFile = std::string(PROJECT_ROOT_DIR) + "/mesh/tools/PoS_66.off";
			std::cout << "Loading direction file from: " << directionFile << std::endl;
			loadPointsOFF(directionFile, directions);
			if (directions.empty()) {
				err = "failed to load direction file: " + directionFile;
				return false;
			}
			for (size_t i = 0; i < directions.size(); ++i) {
				directions[i] = normalize(directions[i]);
			}
			std::cout << "load directions size:  " << directions.size() << std::endl;

			const std::string toolFile = std::string(PROJECT_ROOT_DIR) + "/mesh/tools/ToolHeadSample_r1.off";
			const std::string carriageFile = std::string(PROJECT_ROOT_DIR) + "/mesh/tools/carriage_r1.5_r25_158.off";
			loadPointsOFF(toolFile, baseToolPoints);
			loadPointsOFF(carriageFile, baseCarriagePoints);
			if (baseToolPoints.empty() || baseCarriagePoints.empty()) {
				err = "failed to load tool assets";
				return false;
			}

			staticAssetsLoaded = true;
			return true;
		}

		void prepareScaledTooling(float scaleFactor,
			std::vector<vec3f>& tool,
			std::vector<vec3f>& carriage,
			std::vector<vec3f>& rotatedToolByDir,
			std::vector<vec3f>& rotatedCarriageByDir)
		{
			tool = baseToolPoints;
			carriage = baseCarriagePoints;
			const float toolScale = scaleFactor * cutter.ball_r;
			const float carriageScale = scaleFactor;

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

		Cutter cutter;
		bool staticAssetsLoaded = false;
		std::vector<vec3f> directions;
		std::vector<vec3f> baseToolPoints;
		std::vector<vec3f> baseCarriagePoints;
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
					<< ",\"elapsed_ms\":" << result.elapsedMs << "}" << std::endl;
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
		req.scaleFactor = 0.01f;

		if (ac > 1) req.modelFile = av[1];
		if (ac > 2) req.urpFile = av[2];
		if (ac > 3) req.outFile = av[3];
		if (ac > 4) req.scaleFactor = std::stof(av[4]);

		if (ac < 5) {
			std::cout << "未检测到全部命令行参数，使用默认文件路径" << std::endl;
			std::cout << "模型文件: " << req.modelFile << std::endl;
			std::cout << "URP文件: " << req.urpFile << std::endl;
			std::cout << "输出文件: " << req.outFile << std::endl;
			std::cout << "缩放比例: " << req.scaleFactor << std::endl;
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
