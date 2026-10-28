//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "lidar_common_callback.h"

#include "../lds_lidar.h"

#include <string>

// #region debug-point D:status-report
// 临时诊断：复用SDK状态回调和只读查询，不另占雷达端口，不修改设备配置。
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sstream>
#include <iomanip>
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
namespace {
void ReportLidarStatus(const std::string& text) {
  rapidjson::Document doc;
  doc.SetObject();
  auto& allocator = doc.GetAllocator();
  doc.AddMember("sessionId", "lidar-missing-cloud", allocator);
  doc.AddMember("runId", "status-probe", allocator);
  doc.AddMember("hypothesisId", "D", allocator);
  doc.AddMember("location", "sdk-status", allocator);
  doc.AddMember("msg", "[DEBUG] lidar status", allocator);
  doc.AddMember("data", rapidjson::Value(text.c_str(), allocator), allocator);
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  doc.Accept(writer);
  const std::string body(buffer.GetString(), buffer.GetSize());
  const std::string request = "POST /event HTTP/1.0\r\nHost: localhost\r\nContent-Type: application/json\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n" + body;
  int fd = socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(7777);
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (fd >= 0) {
    timeval timeout = {0, 100000};
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    if (connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0)
      send(fd, request.data(), request.size(), MSG_NOSIGNAL);
    close(fd);
  }
}
}
// #endregion

namespace livox_ros {

void LidarCommonCallback::OnLidarPointClounCb(PointFrame* frame, void* client_data) {
  if (frame == nullptr) {
    printf("LidarPointCloudCb frame is nullptr.\n");
    return;
  }
  if (client_data == nullptr) {
    printf("Lidar point cloud cb failed, client data is nullptr.\n");
    return;
  }
  if (frame->lidar_num ==0) {
    printf("LidarPointCloudCb lidar_num:%u.\n", frame->lidar_num);
    return;
  }
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);
  lds_lidar->StoragePointData(frame);
}

void LidarCommonCallback::LidarImuDataCallback(ImuData* imu_data, void *client_data) {
  if (imu_data == nullptr) {
    printf("Imu data is nullptr.\n");
    return;
  }
  if (client_data == nullptr) {
    printf("Lidar point cloud cb failed, client data is nullptr.\n");
    return;
  }
  // #region debug-point D:status-query
  // 首个IMU说明设备已连接；只读查询一次，状态推送仅报告前五次。
  static bool queried = false;
  if (!queried) {
    queried = true;
    SetLivoxLidarInfoCallback([](uint32_t handle, uint8_t type, const char* info, void*) {
      static unsigned count = 0;
      if (++count <= 5) ReportLidarStatus("push handle=" + std::to_string(handle) + " type=" + std::to_string(type) + " info=" + (info ? info : "null"));
    }, nullptr);
    const auto result = QueryLivoxLidarInternalInfo(inet_addr("192.168.1.100"), [](livox_status status, uint32_t handle, LivoxLidarDiagInternalInfoResponse* response, void*) {
      std::ostringstream out;
      out << "query status=" << status << " handle=" << handle;
      if (response) {
        out << " ret=" << unsigned(response->ret_code) << " params=" << response->param_num;
        const uint8_t* p = response->data;
        for (unsigned i = 0; i < response->param_num; ++i) {
          uint16_t key = 0, length = 0;
          memcpy(&key, p, 2);
          memcpy(&length, p + 2, 2);
          out << " key=0x" << std::hex << key << " len=" << std::dec << length << " value=";
          for (unsigned j = 0; j < length; ++j) out << std::hex << std::setw(2) << std::setfill('0') << unsigned(p[4 + j]);
          p += 4 + length;
        }
      }
      ReportLidarStatus(out.str());
    }, nullptr);
    ReportLidarStatus("query dispatch=" + std::to_string(result));
  }
  // #endregion
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);
  lds_lidar->StorageImuData(imu_data);
}

} // namespace livox_ros
