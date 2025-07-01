// Copyright 2025 coScene
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef UTILS__CURL_CLIENT_HPP_
#define UTILS__CURL_CLIENT_HPP_

#include <string>
#include <map>
#include <curl/curl.h>
#include <utils/json.hpp>

namespace colistener {

struct HttpResponse {
    int status_code;
    std::string body;
    std::map<std::string, std::string> headers;
    bool success;
    std::string error_message;
};

class CurlClient {
public:
    CurlClient();
    ~CurlClient();

    CurlClient(const CurlClient&) = delete;
    CurlClient& operator=(const CurlClient&) = delete;

    HttpResponse get(const std::string& url, 
                     const std::map<std::string, std::string>& headers = {});

    HttpResponse post(const std::string& url, 
                      const nlohmann::json& data,
                      const std::map<std::string, std::string>& headers = {});

    HttpResponse post(const std::string& url, 
                      const std::string& data,
                      const std::map<std::string, std::string>& headers = {});

    void setTimeout(int timeout_seconds);

    void setVerifySSL(bool verify);

private:
    CURL* curl_;
    struct curl_slist* header_list_;
    int timeout_seconds_;
    bool verify_ssl_;

    bool initCurl();

    HttpResponse executeRequest();

    void cleanup();

    void cleanupHeaders();
};

}  // namespace colistener

#endif  // UTILS__CURL_CLIENT_HPP_
