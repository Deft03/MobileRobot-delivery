import yaml

with open('list_goal.yaml', 'w') as file1:
    out = {'receive':(0,6),
           'goal_list':[(5,5),(4,4)]
           }
    yaml.safe_dump(out,file1)