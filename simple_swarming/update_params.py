import json
with open("params.json", "w") as f:
    json.dump({"W_ALIGNMENT": 0.1, "W_COHESION": 0.07, "W_SEPARATION": 0.25}, f)