#include "actions/common_action.hpp"
#include "utils/logger.hpp"

namespace colistener {
CommonAction::CommonAction() : curl_(nullptr), headers_(nullptr), dev_null_(nullptr) {
    curl_ = curl_easy_init();
    if (!curl_) {
        throw std::runtime_error("Failed to initialize CURL");
    }
    
    dev_null_ = fopen("/dev/null", "w");
    if (!dev_null_) {
        curl_easy_cleanup(curl_);
        throw std::runtime_error("Failed to open /dev/null");
    }
    
    headers_ = curl_slist_append(nullptr, "Content-Type: application/json");
    endpoint_ = std::string(DEFAULT_URL) +
        ":" +
        std::string(DEFAULT_PORT) +
        std::string(DEFAULT_ROUTE);
    
    curl_easy_setopt(curl_, CURLOPT_WRITEDATA, dev_null_);
}

CommonAction::~CommonAction() {
    if (headers_) {
        curl_slist_free_all(headers_);
    }
    if (curl_) {
        curl_easy_cleanup(curl_);
    }
    if (dev_null_) {
        fclose(dev_null_);
    }
}

bool CommonAction::execute(const std::vector<MessageCache>& messages) {
    if (messages.empty()) return true;

    // Log the number of messages to be processed
    COLOG_INFO("Processing %zu messages for sending", messages.size());

    // Create the root JSON object with "messages" array
    nlohmann::json root;
    root["messages"] = nlohmann::json::array();

    for (const auto& cache_item : messages) {
        nlohmann::json item;
        item["topic"] = cache_item.topic;
        // Parse msg string into JSON object
        try {
            item["msg"] = nlohmann::json::parse(cache_item.msg);
        } catch (const nlohmann::json::parse_error& e) {
            COLOG_ERROR("Failed to parse msg as JSON: %s", e.what());
            // Fallback to empty object if parsing fails
            item["msg"] = nlohmann::json::object();
        }
        item["msgType"] = cache_item.msgType;
        item["ts"] = cache_item.ts;
        root["messages"].push_back(item);
    }

    try {
        const std::string json_str = root.dump();
        
        // Log request details
        COLOG_DEBUG("Sending request to endpoint: %s", endpoint_.c_str());
        COLOG_DEBUG("Request payload size: %zu bytes", json_str.length());
        // Log detailed request payload for debugging
        COLOG_DEBUG("Request payload: %s", json_str.c_str());

        curl_easy_setopt(curl_, CURLOPT_URL, endpoint_.c_str());
        curl_easy_setopt(curl_, CURLOPT_HTTPHEADER, headers_);
        curl_easy_setopt(curl_, CURLOPT_POSTFIELDS, json_str.c_str());
        curl_easy_setopt(curl_, CURLOPT_POSTFIELDSIZE, json_str.length());

        const CURLcode res = curl_easy_perform(curl_);
        if (res != CURLE_OK) {
            // Log CURL errors
            COLOG_ERROR("CURL request failed: %s", curl_easy_strerror(res));
            return false;
        }

        long http_code = 0;
        curl_easy_getinfo(curl_, CURLINFO_RESPONSE_CODE, &http_code);
        if (http_code >= 200 && http_code < 300) {
            // Log successful response
            COLOG_INFO("Request successful, HTTP code: %ld", http_code);
            return true;
        } else {
            // Log failed response
            COLOG_ERROR("Request failed with HTTP code: %ld", http_code);
            return false;
        }
    }
    catch (const std::exception& e) {
        // Log any exceptions that occur during request
        COLOG_ERROR("Exception occurred while sending request: %s", e.what());
        return false;
    }
}
} // namespace colistener
