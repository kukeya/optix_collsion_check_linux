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

#include "glfWindow/GLFWindow.h"
#include "SampleRenderer.h"
#include "utils.h"
// our helper library for window handling
#include <GL/gl.h>
#include <chrono>
#include <fstream>

/*! \namespace osc - Optix Siggraph Course */
namespace osc {


	struct Cutter {
		float ball_r = 1.5;         // 球半径
		float carriage_r = 25;      // 滑台半径
		float carriage_height = 30; // 滑台高度
		float cylinder_height = 23; // 圆柱高度
		float scale_factor = 0.01;
	};

	struct SampleWindow : public GLFCameraWindow
	{
		SampleWindow(const std::string& title,
			const TriangleMesh& model,
			const Camera& camera,
			const float worldScale)
			: GLFCameraWindow(title, camera.from, camera.at, camera.up, worldScale),
			sample(model)
		{
		}

		virtual void render() override
		{
			if (cameraFrame.modified) {
				sample.setCamera(Camera{ cameraFrame.get_from(),
										 cameraFrame.get_at(),
										 cameraFrame.get_up() });
				cameraFrame.modified = false;
			}
			sample.render();
		}

		virtual void draw() override
		{
			return;
			////////////////////////////////////////////////////////////

			sample.downloadPixels(pixels.data());
			if (fbTexture == 0)
				glGenTextures(1, &fbTexture);

			glBindTexture(GL_TEXTURE_2D, fbTexture);
			GLenum texFormat = GL_RGBA;
			GLenum texelType = GL_UNSIGNED_BYTE;
			glTexImage2D(GL_TEXTURE_2D, 0, texFormat, fbSize.x, fbSize.y, 0, GL_RGBA,
				texelType, pixels.data());

			glDisable(GL_LIGHTING);
			glColor3f(1, 1, 1);

			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();

			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, fbTexture);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

			glDisable(GL_DEPTH_TEST);

			glViewport(0, 0, fbSize.x, fbSize.y);

			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			glOrtho(0.f, (float)fbSize.x, 0.f, (float)fbSize.y, -1.f, 1.f);

			glBegin(GL_QUADS);
			{
				glTexCoord2f(0.f, 0.f);
				glVertex3f(0.f, 0.f, 0.f);

				glTexCoord2f(0.f, 1.f);
				glVertex3f(0.f, (float)fbSize.y, 0.f);

				glTexCoord2f(1.f, 1.f);
				glVertex3f((float)fbSize.x, (float)fbSize.y, 0.f);

				glTexCoord2f(1.f, 0.f);
				glVertex3f((float)fbSize.x, 0.f, 0.f);
			}
			glEnd();
		}

		virtual void resize(const vec2i& newSize)
		{
			fbSize = newSize;
			sample.resize(newSize);
			pixels.resize(newSize.x * newSize.y);
			hitIDs.resize(newSize.x * newSize.y);
		}

		vec2i                 fbSize;
		GLuint                fbTexture{ 0 };
		SampleRenderer        sample;
		std::vector<uint32_t> pixels;
		std::vector<uint32_t> hitIDs;
		std::vector<int> reachable;
	};

	struct SampleIntersection {
		SampleIntersection(const TriangleMesh& model,
			std::vector<vec3f>& checkPoints,
			std::vector<vec3f>& directions,
			std::vector<vec3f>& toolSamplePoints,
			std::vector<vec3f> carriage) : sample(model), checkPoints(checkPoints), directions(directions), toolSamplePoints(toolSamplePoints), carriage(carriage) {
			/*auto slp = sample.getLaunchParams();
			slp->urps = checkPoints.data();
			slp->numUrps = checkPoints.size();
			slp->toolSample = toolSamplePoints.data();
			slp->numToolSamplePoints = toolSamplePoints.size();
			slp->directions = directions.data();
			slp->numDirections = directions.size();*/
			resize(vec3i(checkPoints.size(), directions.size(), toolSamplePoints.size()));
		};

		void render() {
			sample.uploadArray(checkPoints, directions, toolSamplePoints, carriage);
			sample.render();
		}

		void resize(const vec3i& newSize) {
			fbSize = newSize;
			sample.resize(newSize);
			reachable.resize(newSize.x * newSize.y * newSize.z);
		}

		vec3i                 fbSize;
		SampleRenderer        sample;
		std::vector<int>      reachable;
		std::vector<vec3f>    checkPoints;
		std::vector<vec3f>    directions;
		std::vector<vec3f>    toolSamplePoints;
		std::vector<vec3f>    carriage;
	};

	/*! main entry point to this example - initially optix, print hello
	  world, then exit */
	extern "C" int main(int ac, char** av)
	{

		std::string modelFile = std::string(PROJECT_ROOT_DIR) + "/mesh/exp1/ruyi_recur1_n.off";
		std::string urpFile = std::string(PROJECT_ROOT_DIR) + "/mesh/exp1/ruyi_recur1_urps_n.off";
		std::string outFile = std::string(PROJECT_ROOT_DIR) + "/mesh/exp1/ruyi_recur1_output_collision.obj";

		/*std::string modelFile = "D:\\Repos\\testCuda\\modelFile\\debug_normal_collison\\test_scaled.off" "D:\\Repos\\testCuda\\modelFile\\debug_normal_collison\\sample_scaled.off" "D:\\Repos\\testCuda\\modelFile\\debug_normal_collison\\output_scaled.obj"*/


		if (ac > 1) modelFile = av[1];
		if (ac > 2) urpFile = av[2];
		if (ac > 3) outFile = av[3];

		if (ac < 4) {
			std::cout << "未检测到全部命令行参数，使用默认文件路径" << std::endl;
			std::cout << "模型文件: " << modelFile << std::endl;
			std::cout << "URP文件: " << urpFile << std::endl;
			std::cout << "输出文件: " << outFile << std::endl;
		}

		try {
			auto start = std::chrono::steady_clock::now();
			 
			Cutter cutter;

			TriangleMesh model;
			model.loadOFF(modelFile);

			// load dirction from PoS file
			std::vector<vec3f> directions;
			std::string directionFile = std::string(PROJECT_ROOT_DIR) + "/mesh/tools/PoS_66.off";
			std::cout << "Loading direction file from: " << directionFile << std::endl; loadPointsOFF(directionFile, directions);
			std::cout << "load directions size:  " << directions.size() << std::endl;
			// 把 directions 归一化
			for (auto& dir : directions) {
				dir = normalize(dir);
			}

			// load urp from file
			std::vector<vec3f> urps;
			loadPointsOFF(urpFile, urps);
			std::cout << "load urps size:  " << urps.size() << std::endl;

			// load tool from ToolHeadSample_r1 file
			// 工具球半径为 1.5，且 z>=0
			std::vector<vec3f> tool;
			std::vector<vec3f> carriage;
			std::string checkPointsFile = std::string(PROJECT_ROOT_DIR) + "/mesh/tools/ToolHeadSample_r1.off";
			std::string carriagePointsFile = std::string(PROJECT_ROOT_DIR) + "/mesh/tools/carriage_r1.5_r25_158.off";
			loadPointsOFF(checkPointsFile, tool);
			loadPointsOFF(carriagePointsFile, carriage);
			
			// 按比例缩放 tool 与 carriage
			float tool_f = cutter.scale_factor * cutter.ball_r;
			for (auto& p : tool) {
				p *= tool_f;
			}

			float carriage_f = cutter.scale_factor;
			for (auto& p : carriage) {
				p *= carriage_f;
			}

			// 分批处理以避免占用过多内存
			const size_t batchSize = 10000; // 每批处理 10000 个点
			std::vector<bool> urpsRechable(urps.size());

			for (size_t batchStart = 0; batchStart < urps.size(); batchStart += batchSize) {
				size_t batchEnd = std::min(batchStart + batchSize, urps.size());
				std::vector<vec3f> urpsBatch(urps.begin() + batchStart, urps.begin() + batchEnd);

				std::cout << "处理批次: " << batchStart << " - " << batchEnd << std::endl;

				SampleIntersection* sampleInterMachine = new SampleIntersection(model, urpsBatch, directions, tool, carriage);
				sampleInterMachine->render();
				sampleInterMachine->sample.downloadReachable(sampleInterMachine->reachable.data());

				// 汇总当前批次的结果
				for (size_t i = 0; i < urpsBatch.size(); i++) {
					size_t globalIdx = batchStart + i;
					bool pointReachable = false;
					for (int d = 0; d < directions.size(); d++) {
						bool directionNoHit = false;
						for (int t = 0; t < tool.size(); t++) {
							int idx = t + d * tool.size() + i * directions.size() * tool.size();
							if (sampleInterMachine->reachable[idx] != 0) {
								directionNoHit = true;
								break;
							}
						}
						if (directionNoHit) {
							pointReachable = true;
							break;
						}
					}
					urpsRechable[globalIdx] = pointReachable;
				}

				delete sampleInterMachine;
			}

			std::vector<vec3f> unreachablePoints;
            std::vector<int> unreachableIndices; // <--- 新增：用于存储索引
			for (size_t i = 0; i < urpsRechable.size(); i++)
			{
				if (urpsRechable[i] == false) {
					/*system("pause");*/
					unreachablePoints.push_back(urps[i]);
                    unreachableIndices.push_back(i); // <--- 新增：记录原始索引
				}
			}
			// 将不可达点写出到 OBJ 文件
			writeOBJwithPoints(outFile, unreachablePoints);
			std::cout << "已生成不可达点输出文件: " << outFile << " " << unreachablePoints.size() << std::endl;

			std::string indexOutFile = outFile + ".indices.txt";
			std::ofstream idxFile(indexOutFile);
            if (idxFile.is_open()) {
                for (int idx : unreachableIndices) {
                    idxFile << idx << "\n";
                }
                idxFile.close();
                std::cout << "已生成不可达点索引文件: " << indexOutFile << std::endl;
            }
            else {
                std::cout << "无法创建索引文件: " << indexOutFile << std::endl;
            }

			auto end = std::chrono::steady_clock::now();
			auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(end - start);
			std::cout << "耗时: " << elapsed.count() << " 秒" << std::endl;

			return 0;

		}
		catch (std::runtime_error& e) {
			std::cout << GDT_TERMINAL_RED << "FATAL ERROR: " << e.what()
				<< GDT_TERMINAL_DEFAULT << std::endl;
			exit(1);
		}
		return 0;
	}

}