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

#include "utils/curl_client.hpp"

#include <iostream>

#include "utils/logger.hpp"
#include <sstream>

namespace colistener {

CurlClient::CurlClient() : curl_(nullptr), header_list_(nullptr), timeout_seconds_(30), verify_ssl_(false) {
    if (!initCurl()) {
        throw std::runtime_error("Failed to initialize CURL");
    }
}

CurlClient::~CurlClient() {
    cleanup();
}

bool CurlClient::initCurl() {
    curl_ = curl_easy_init();
    if (!curl_) {
        return false;
    }
    
    curl_easy_setopt(curl_, CURLOPT_TIMEOUT, timeout_seconds_);
    curl_easy_setopt(curl_, CURLOPT_FOLLOWLOCATION, 1L);
    
    return true;
}

void CurlClient::setTimeout(int timeout_seconds) {
    timeout_seconds_ = timeout_seconds;
    if (curl_) {
        curl_easy_setopt(curl_, CURLOPT_TIMEOUT, timeout_seconds_);
    }
}

void CurlClient::setVerifySSL(bool verify) {
    verify_ssl_ = verify;
    if (curl_) {
        curl_easy_setopt(curl_, CURLOPT_SSL_VERIFYPEER, verify ? 1L : 0L);
        curl_easy_setopt(curl_, CURLOPT_SSL_VERIFYHOST, verify ? 2L : 0L);
    }
}

void CurlClient::cleanupHeaders() {
    if (header_list_) {
        curl_slist_free_all(header_list_);
        header_list_ = nullptr;
    }
}

HttpResponse CurlClient::get(const std::string& url, 
                            const std::map<std::string, std::string>& headers) {
    if (!curl_) {
        return HttpResponse{0, "", {}, false, "CURL not initialized"};
    }

    cleanupHeaders();
    for (const auto& header : headers) {
        std::string header_line = header.first + ": " + header.second;
        header_list_ = curl_slist_append(header_list_, header_line.c_str());
    }


    curl_easy_setopt(curl_, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl_, CURLOPT_HTTPHEADER, header_list_);
    curl_easy_setopt(curl_, CURLOPT_TIMEOUT, timeout_seconds_);
    curl_easy_setopt(curl_, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl_, CURLOPT_SSL_VERIFYPEER, verify_ssl_ ? 1L : 0L);
    curl_easy_setopt(curl_, CURLOPT_SSL_VERIFYHOST, verify_ssl_ ? 2L : 0L);
    curl_easy_setopt(curl_, CURLOPT_HTTPGET, 1L);
    
    return executeRequest();
}

HttpResponse CurlClient::post(const std::string& url, 
                             const nlohmann::json& data,
                             const std::map<std::string, std::string>& headers) {
    return post(url, data.dump(), headers);
}

HttpResponse CurlClient::post(const std::string& url, 
                             const std::string& data,
                             const std::map<std::string, std::string>& headers) {
    if (!curl_) {
        return HttpResponse{0, "", {}, false, "CURL not initialized"};
    }
    
    curl_easy_reset(curl_);
    
    curl_easy_setopt(curl_, CURLOPT_TIMEOUT, timeout_seconds_);
    curl_easy_setopt(curl_, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl_, CURLOPT_SSL_VERIFYPEER, verify_ssl_ ? 1L : 0L);
    curl_easy_setopt(curl_, CURLOPT_SSL_VERIFYHOST, verify_ssl_ ? 2L : 0L);
    
    curl_easy_setopt(curl_, CURLOPT_POST, 1L);
    curl_easy_setopt(curl_, CURLOPT_POSTFIELDS, data.c_str());
    curl_easy_setopt(curl_, CURLOPT_POSTFIELDSIZE, data.length());

    curl_easy_setopt(curl_, CURLOPT_URL, url.c_str());

    cleanupHeaders();
    for (const auto& header : headers) {
        std::string header_line = header.first + ": " + header.second;
        header_list_ = curl_slist_append(header_list_, header_line.c_str());
    }
    curl_easy_setopt(curl_, CURLOPT_HTTPHEADER, header_list_);
    
    return executeRequest();
}

HttpResponse CurlClient::executeRequest() {
    std::string response_body;
    std::map<std::string, std::string> response_headers;

    auto write_callback = [](void* contents, size_t size, size_t nmemb, std::string* userp) -> size_t {
        size_t totalSize = size * nmemb;
        userp->append(reinterpret_cast<char*>(contents), totalSize);
        return totalSize;
    };

    auto header_callback = [](char* buffer, size_t size, size_t nitems, std::map<std::string, std::string>* headers) -> size_t {
        std::string header_line(buffer, size * nitems);

        if (!header_line.empty() && header_line.back() == '\n') {
            header_line.pop_back();
        }
        if (!header_line.empty() && header_line.back() == '\r') {
            header_line.pop_back();
        }

        size_t colon_pos = header_line.find(':');
        if (colon_pos != std::string::npos) {
            std::string key = header_line.substr(0, colon_pos);
            std::string value = header_line.substr(colon_pos + 1);

            key.erase(0, key.find_first_not_of(" \t"));
            key.erase(key.find_last_not_of(" \t") + 1);
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);

            (*headers)[key] = value;
        }

        return size * nitems;
    };

    curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, +write_callback);
    curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &response_body);
    curl_easy_setopt(curl_, CURLOPT_HEADERFUNCTION, +header_callback);
    curl_easy_setopt(curl_, CURLOPT_HEADERDATA, &response_headers);

    try {
        CURLcode res = curl_easy_perform(curl_);
        HttpResponse response;
        response.body = response_body;
        response.headers = response_headers;

        if (res != CURLE_OK) {
            response.success = false;
            response.error_message = curl_easy_strerror(res);
            response.status_code = 0;
            COLOG_ERROR("CURL request failed: %s", response.error_message.c_str());
        } else {
            long http_code = 0;
            curl_easy_getinfo(curl_, CURLINFO_RESPONSE_CODE, &http_code);
            response.status_code = static_cast<int>(http_code);
            response.success = (http_code >= 200 && http_code < 300);

            if (!response.success) {
                response.error_message = "HTTP " + std::to_string(http_code);
                COLOG_ERROR("HTTP request from [%s] failed with status code: %d", curl_, response.status_code);
                COLOG_DEBUG("Response body: %s", response_body.c_str());
                COLOG_DEBUG("Response headers:");
                for (const auto& header : response_headers) {
                    COLOG_DEBUG("  %s: %s", header.first.c_str(), header.second.c_str());
                }
            }
        }

        return response;
    }
    catch (const std::exception& e) {
        std::cerr << "CurlClient::executeRequest() exception: " << e.what() << std::endl;
        return HttpResponse();
    }
}

void CurlClient::cleanup() {
    cleanupHeaders();
    if (curl_) {
        curl_easy_cleanup(curl_);
        curl_ = nullptr;
    }
}

}  // namespace colistener
