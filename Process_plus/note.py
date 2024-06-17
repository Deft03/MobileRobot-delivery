# import path4server

# agent_list = []
# agent = {'name':"R1",'goal':[5,5],'start':[2,2],'even':None}
# agent_list.append(agent)

# print(path4server.main(agent_list))

a = "2D,3L,4R"
a = a.split(',')
for item in a:
    print(int(item[:-1]))
    print(item[-1])
