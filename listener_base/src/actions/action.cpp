#include "actions/action.hpp"
#include "actions/common_action.hpp"
#include "actions/example_action.hpp"

namespace colistener {
std::shared_ptr<Action> Action::create(const std::string& type) {
    if (type == "common" || type.empty()) {
        return std::make_shared<CommonAction>();
    } else if (type == "example") {
        return std::make_shared<ExampleAction>();
    }
    // more action types here.

    throw std::runtime_error("Unknown action type: " + type);
}
} // namespace colistener
