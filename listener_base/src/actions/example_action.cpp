#include <iostream>
#include "actions/example_action.hpp"

namespace colistener {

ExampleAction::ExampleAction() {
    // construct action
}

ExampleAction::~ExampleAction() {
    // deconstruct action here
}

bool ExampleAction::execute(const std::vector<MessageCache>& messages) {
    // You can process the data here by either sending it via HTTP, storing it on the local disk, or doing anything else you want.
    // Of course, you can also perform secondary processing on the data here.
    // The data in the parameter has already been persisted.
    // If this function returns true, all the messages in the parameters will be removed from the persistent database.
    // If it returns false, the data will remain in the persistent database,
    // and the existing data will appear again in the parameter when the function is called next time.

    if (messages.empty()) return true;

    nlohmann::json json_array = nlohmann::json::array();
    for (const auto& cache_item : messages) {
        nlohmann::json item;
        item["topic"] = cache_item.topic;
        item["msg"] = nlohmann::json::parse(cache_item.msg);
        item["msgType"] = cache_item.msgType;
        item["ts"] = cache_item.ts;
        json_array.push_back(item);
    }

    std::cout << json_array.dump() << std::endl;
    return true;
}
}