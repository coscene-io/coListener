from sys import argv
import json
from ..base_actions import noop
from .config_validator import validate_config

if __name__ == "__main__":
    config = json.loads(argv[1])
    result, _ = validate_config(config, noop)

    print(json.dumps(result))
    exit(0 if result["success"] else 1)
