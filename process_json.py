import json

# Define the skill mapping
skill_mapping = {
    range(1, 10): "PickAndPlace",
    range(10, 12): "PickAndPlaceWithSupport",
    range(12, 13): "PickHandoverAndPlace"
}

def get_skill(brick_id):
    for key in skill_mapping:
        if brick_id in key:
            return skill_mapping[key]
    return None

# Read the input JSON file
input_file = '/home/patricia/Desktop/Learn/projects/ros1_aidf/src/AIDF/cliff_meta_skills.json'
with open(input_file, 'r') as file:
    data = json.load(file)

# Process the data
processed_data = {}
for key, value in data.items():
    brick_id = int(key)
    skill = get_skill(brick_id)
    num_robot = 1 if value['sup_robot_id'] == 0 else 2
    processed_data[key] = {
        "skill": skill,
        "object": f"b{value['brick_id']}_{value['brick_seq']}",
        "target-x": value["x"],
        "target-y": value["y"],
        "target-z": value["z"],
        "ori": value["ori"],
        "robot-id": value["robot_id"],
        "NumRobot": num_robot
    }

# Write the output JSON file
output_file = '/home/patricia/Desktop/Learn/projects/ros1_aidf/src/AIDF/processed_cliff_meta_skills.json'
with open(output_file, 'w') as file:
    json.dump(processed_data, file, indent=4)

print(f"Processed data has been written to {output_file}")