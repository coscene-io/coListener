#include "actions/common_action.hpp"

namespace colistener {
CommonAction::CommonAction() : curl_(nullptr), headers_(nullptr) {
    curl_ = curl_easy_init();
    if (!curl_) {
        throw std::runtime_error("Failed to initialize CURL");
    }
    headers_ = curl_slist_append(nullptr, "Content-Type: application/json");
    endpoint_ = std::string(DEFAULT_URL) +
        ":" +
        std::string(DEFAULT_PORT) +
        std::string(DEFAULT_ROUTE);
}

CommonAction::~CommonAction() {
    if (headers_) {
        curl_slist_free_all(headers_);
    }
    if (curl_) {
        curl_easy_cleanup(curl_);
    }
}

bool CommonAction::execute(const std::vector<MessageCache>& messages) {
    if (messages.empty()) return true;

    // if msg need be processed, do it here.
    nlohmann::json json_array = nlohmann::json::array();
    for (const auto& cache_item : messages) {
        nlohmann::json item;
        item["topic"] = cache_item.topic;
        item["msg"] = cache_item.msg;
        item["msgType"] = cache_item.msgType;
        item["ts"] = cache_item.ts;
        json_array.push_back(item);
    }

    try {
        const std::string json_str = json_array.dump();

        curl_easy_setopt(curl_, CURLOPT_URL, endpoint_.c_str());
        curl_easy_setopt(curl_, CURLOPT_HTTPHEADER, headers_);
        curl_easy_setopt(curl_, CURLOPT_POSTFIELDS, json_str.c_str());
        curl_easy_setopt(curl_, CURLOPT_POSTFIELDSIZE, json_str.length());

        const CURLcode res = curl_easy_perform(curl_);
        if (res != CURLE_OK) {
            return false;
        }

        long http_code = 0;
        curl_easy_getinfo(curl_, CURLINFO_RESPONSE_CODE, &http_code);
        if (http_code >= 200 && http_code < 300) {
            return true;
        } else {
            return false;
        }
    }
    catch (const std::exception& e) {
        return false;
    }
}
} // namespace colistener
