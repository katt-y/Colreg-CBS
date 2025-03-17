import numpy as np
import heapq
import copy
import pickle
from functions import *
from classes_refinement import *
import random
import time

value_of_arranging = 0
vertex_conflict = False
wait = False
with open("problems.pkl", "rb") as file:
    set_of_nodes = pickle.load(file)
with open("myobject_2.pkl", "rb") as file:
    set_of_nodes = pickle.load(file)
obstacles_nodes = []
reference_dict = set_of_nodes[3]
nodes_to_load = set_of_nodes[1]
for i in obstacles_nodes:
    nodes_to_load[reference_dict[i]].obstacle_node = True

open_list = []

value_of_arranging = 0



obstacles_nodes = []
reference_dict = set_of_nodes[3]
nodes_to_load = set_of_nodes[1]
for i in obstacles_nodes:
    nodes_to_load[reference_dict[i]].obstacle_node = True
Random = False

open_list = []

locations = [(0.5, 0.5),
 (0.5, 2.23),
 (0.5, 3.96),
 (0.5, 5.7),
 (0.5, 7.43),
 (0.5, 9.16),
 (0.5, 10.89),
 (0.5, 12.62),
 (2.0, 11.76),
 (2.0, 10.03),
 (2.0, 8.29),
 (2.0, 6.56),
 (2.0, 4.83),
 (2.0, 3.1),
 (2.0, 1.37),
 (3.5, 0.5),
 (3.5, 2.23),
 (3.5, 3.96),
 (3.5, 5.7),
 (3.5, 7.43),
 (3.5, 9.16),
 (3.5, 10.89),
 (3.5, 12.62),
 (5.0, 11.76),
 (5.0, 10.03),
 (5.0, 8.29),
 (5.0, 6.56),
 (5.0, 4.83),
 (5.0, 3.1),
 (5.0, 1.37),
 (6.5, 0.5),
 (6.5, 2.23),
 (6.5, 3.96),
 (6.5, 5.7),
 (6.5, 7.43),
 (6.5, 9.16),
 (6.5, 10.89),
 (6.5, 12.62),
 (8.0, 11.76),
 (8.0, 10.03),
 (8.0, 8.29),
 (8.0, 6.56),
 (8.0, 4.83),
 (8.0, 3.1),
 (8.0, 1.37),
 (9.5, 0.5),
 (9.5, 2.23),
 (9.5, 3.96),
 (9.5, 5.7),
 (9.5, 7.43),
 (9.5, 9.16),
 (9.5, 10.89),
 (9.5, 12.62),
 (11.0, 11.76),
 (11.0, 10.03),
 (11.0, 8.29),
 (11.0, 6.56),
 (11.0, 4.83),
 (11.0, 3.1),
 (11.0, 1.37),
 (12.5, 0.5),
 (12.5, 2.23),
 (12.5, 3.96),
 (12.5, 5.7),
 (12.5, 7.43),
 (12.5, 9.16),
 (12.5, 10.89),
 (12.5, 12.62)]

def get_random_locations(locations, n):
    if n > len(locations):
        raise ValueError("n cannot be greater than the number of available locations")
    return random.sample(locations, n)

# Example usage

Random_2 = input(' Would you like to main provide the indices for the starting and ending positions for each agent ? y/n :  ') # Change this value to select a different number of locations
if Random_2.lower() == 'y':
    locations_2 = []
    n = int(input('Please select the number of agents that you would like to test the CBS for :  '))
    for _ in range(n):
        locations_2.append(locations[int(input('Please enter the indice for the start coordinate of '+ str(_+1)+ '-th agent:'))]) # Change this value to select a different number of locations
        locations_2.append(locations[int(input('Please enter the indice for the end coordinate of '+ str(_+1)+ '-th agent:'))]) # Change this value to select a different number of locations
    
    locations_2.reverse()
    locations = locations_2
else:
    n = int(input('Please select the number of agents that you would like to test the CBS for , note that n should be even :  ')) # Change this value to select a different number of locations
 # Change this value to select a different number of locations
    random_locations = get_random_locations(locations, n)
    print(random_locations)
    random_locations_2 = copy.deepcopy(random_locations)
    random_locations_2.reverse()
    random_locations.extend(random_locations_2)
    locations = random_locations

# Example usage
# locations = [(0.5 ,0.5, 0.5),(2.5 ,2.5, 2.5),(2.5 ,2.5, 2.5),(0.5 ,0.5, 0.5),(0.5, 2.5, 0.5), (1.5, 2.5, 1.5)]
agent_list = [0]
open_list = []
high_level_nodes = 0
low_level_nodes = 0
locations_2 = copy.deepcopy(locations)
#locations = locations_2 = [(5.0, 6.56),(5.0, 1.37),(5.0, 1.37),(5.0, 6.56)]
initial_time = time.time()
agent_list = [0]
while locations != []:
    agent_list.append(
        agents(
            set_of_nodes[1][reference_dict[locations.pop()]],
            set_of_nodes[1][reference_dict[locations.pop()]],
        )
    )

high_level = high_level_node(
    nodes_to_load,
    agents=agent_list,
    reference_dict=reference_dict,
    reference_nodes=nodes_to_load,
    timestep_start_dict={
        i: (0, agent_list[i].start_node.location) for i in range(1, len(agent_list))
    },
)

k = []
t = 0
for i in range(1, len(high_level.agents)):
    print('agent = ' + str(i))
    a, b, b_3, b_4, b_2 = solution = high_level.find_path_colregs(
        agent_num=i,
        start_node=high_level.agents[i].start_node,
        dest_node=high_level.agents[i].goal_node,
    )
    if b_2 > low_level_nodes:
        low_level_nodes = b_2
    k.append(a)
    high_level.node_cost.append(b)
    high_level.imaginary_agent_timesteps[i] = [_[1] for _ in b_3]
    high_level.successor_agent_timesteps[i] = [_[1] for _ in b_4]
    for _ in b_3:
        # try:
        #     high_level.agent_positions[_[1]][i] = []
        #     high_level.agent_positions[_[1]][i].append(_[0])
        # except KeyError:
        #     high_level.agent_positions[_[1]] = {}
        #     high_level.agent_positions[_[1]][i] = []
        #     high_level.agent_positions[_[1]][i].append(_[0])
        if _[1] not in high_level.agent_positions:
            high_level.agent_positions[_[1]] = {}
        
        if i not in high_level.agent_positions[_[1]]:
            high_level.agent_positions[_[1]][i] = []
        
        high_level.agent_positions[_[1]][i].append(_[0])

    for _ in b_4:
        try:
            high_level.future_positions[_[1]][i] = _[0]
        except KeyError:
            high_level.future_positions[_[1]] = {}
            high_level.future_positions[_[1]][i] = _[0]

    if b > t:
        t = b

k_2 = []
for i in k:
    _ = np.pad(i, (0, t + 1 - len(i)), mode="constant", constant_values=(0, i[-1]))
    k_2.append(_)

k_2 = np.array(k_2).T
high_level.node_grid = k_2
high_level.path_cost = np.sum(high_level.node_cost)
initial_cost = high_level.path_cost
high_level.timestep = 0
heapq.heappush(open_list, (high_level.path_cost, id(high_level), high_level))
j = 0

while open_list != []:
    # if high_level_nodes > 1000:
    #      break

    path_cost, i_d, present_node = heapq.heappop(open_list)
    # print('length of open list = ' +str(len(open_list)))
    final_time = time.time()

    j += 1
    # print('chec before checking conflicts = ' + str(present_node.nodes[28].explored))
    print(present_node.agent_positions)
    final_array = [[x.location for x in i] for i in present_node.node_grid]
    print("final_array of the node to be presented = " + str(final_array))
    

    # final_array = np.array(final_array)
    # final_array = final_array.T
    # # print(final_array)
    # truth_values = [
    #     list(zip(final_array[0][i], final_array[1][i]))
    #     for i in range(len(final_array[0]))
    # ]
    # print(truth_values)
    
    a = present_node.check_conflicts()

    # print('chec after checking conflicts = ' + str(present_node.nodes[28].explored))
    if a == None:

        print("The solution was reached")
        print("total time taken = " + str(final_time - initial_time))
        time.sleep(3)
        break
    else:

            if type(a) == tuple:
                a, b, c = a
                if type(a) != tuple:
                    constraints = [(b[0], c, a), (b[1], c, a)]
                    print('constraints = ' + str([(b[0], c, a.location), (b[1], c, a.location)]))


                else:
                    constraints = [(a, b[0], c), ((a[1], a[0]), b[1], c)]
                    tuple_constraints = True
                    print('constraints = ' + str([((a[0].location,a[1].location), b[0], c), ((a[1].location, a[0].location), b[1], c)]))


                # print(str([((x[0].location,x[1].location),y,z) for (x,y,z) in constraints]) + 'edge')
                # single_child = True
            # print(
            #     "present_node grid = "
            #     + str([[x.location for x in i] for i in present_node.node_grid])
            # )

            elif "wait" in a:

                if 'waiting_agent' not in a:
                    print('secondary edge constraints = ' + str(a))
                    constraints = []
                    list_of_keys = [_ for _ in a.keys()]
                    constraints.append(
                        (
                            list_of_keys[0],
                            a["timestep"],
                            present_node.node_grid[a["timestep"] - 1][list_of_keys[0] - 1].location,
                            a[list_of_keys[0]][1],
                            present_node.node_grid[a["timestep"]][list_of_keys[0] - 1].osf_dict[
                                a[list_of_keys[0]][1]
                            ],
                        )
                    )
                    constraints.append(
                        (
                            list_of_keys[1],
                            a["timestep"],
                            present_node.node_grid[a["timestep"] - 1][list_of_keys[1] - 1].location,
                            a[list_of_keys[1]][1],
                            present_node.node_grid[a["timestep"]][list_of_keys[1] - 1].osf_dict[
                                a[list_of_keys[1]][1]
                            ],
                        )
                    )

                    constraints.append( (
                        (
                            present_node.node_grid[a["timestep"] - 1][list_of_keys[0] - 1],
                            present_node.node_grid[a["timestep"]][list_of_keys[0] - 1],
                        ),
                        list_of_keys[0],
                        a['timestep']
                    ))

                    constraints.append (  (
                        (
                            present_node.node_grid[a["timestep"] - 1][list_of_keys[1] - 1],
                            present_node.node_grid[a["timestep"]][list_of_keys[1] - 1],
                        ),
                        list_of_keys[1],
                        a['timestep']
                    ))

                else:
                    wait = True
                    constraints = []
                    list_of_keys =  list(a.keys())[:2]
                    waiting_agent = a['waiting_agent']
                    the_other_agent = [e for e in list_of_keys if e != waiting_agent][0]
                    constraints.append( (
                            the_other_agent,
                            a["timestep"],
                            present_node.node_grid[a["timestep"] - 1][the_other_agent - 1].location,
                            a[the_other_agent][1],
                            present_node.node_grid[a["timestep"]][the_other_agent - 1].osf_dict[
                                a[the_other_agent][1]
                            ],
                        ))
                
            #  constraints.append((waiting_agent,a["timestep"] + 1,present_node.node_grid[a["timestep"] + 1][waiting_agent - 1]))
                    constraints.append(((present_node.node_grid[a["timestep"]][waiting_agent - 1],present_node.node_grid[a["timestep"] + 1][waiting_agent - 1]),waiting_agent,a["timestep"] + 1))
        
        # agent_dict = {_ : [],the_other_agent : [],'wait':False}
            else:
                
                if not new:

                    with open('array.txt','a') as the:

                        the.write("final_array of the node to be presented = " + str(final_array) + "\n")

                vertex_conflict = True
                the_time = a["timestep"]
                constraints = []
            

                constraints.append(
                    (
                        (
                            present_node.node_grid[the_time - 2][a["imaginary_agent"] - 1],
                            present_node.node_grid[the_time - 1][a["imaginary_agent"] - 1],
                        ),
                        a["imaginary_agent"],
                        a['timestep'] - 1
                    )
                )
            
                print('secondary vertex constraints = ' + str( ( (
                        present_node.node_grid[the_time - 2][a["imaginary_agent"] - 1].location,
                        present_node.node_grid[the_time - 1][a["imaginary_agent"] - 1].location,
                    ),
                    a["imaginary_agent"], 
                    a['timestep'] - 1)))
                
                
                # with open('array.txt','a') as the:

                #     the.write('secondary vertex constraints = ' + str( ( (
                #     present_node.node_grid[the_time - 2][a["imaginary_agent"] - 1].location,
                #     present_node.node_grid[the_time - 1][a["imaginary_agent"] - 1].location,
                # ),
                # a["imaginary_agent"],
                # a['timestep'] - 1)) + "\n")
            
                
                if present_node.node_grid[the_time - 1][a["imaginary_agent"] - 1].location  != present_node.node_grid[the_time][a["imaginary_agent"] - 1].location:
                        constraints.append((a['imaginary_agent'],a['timestep'] - 1, present_node.node_grid[the_time - 2][a["imaginary_agent"] - 1].location, [e for e in range(1,7) if present_node.node_grid[the_time - 1][a["imaginary_agent"] - 1].osf_dict[e] == present_node.node_grid[the_time][a["imaginary_agent"] - 1]][0],present_node.node_grid[the_time][a["imaginary_agent"] - 1]))
                    
                else: 
                    recurrent_location = present_node.node_grid[the_time - 1][a["imaginary_agent"] - 1].location
                   
                    
                    for _ in range(1,100):

                        if present_node.node_grid[the_time + _][a["imaginary_agent"] - 1].location != recurrent_location:
                            break
                        
                    constraints.append((a['imaginary_agent'],a['timestep'] - 1, present_node.node_grid[the_time - 2][a["imaginary_agent"] - 1].location, [e for e in range(1,7) if present_node.node_grid[the_time - 1][a["imaginary_agent"] - 1].osf_dict[e] == present_node.node_grid[the_time + _][a["imaginary_agent"] - 1]][0],present_node.node_grid[the_time + _][a["imaginary_agent"] - 1]))

                    print(' Secondary Vertex Constraints = ' + str(constraints[1][:4]) + ',' + str(constraints[1][4].location))
                    
                    # with open('array.txt','a') as the:

                    #     the.write(' Secondary Vertex Constraints = ' + str(constraints[0][:4]) + ',' + str(constraints[0][4].location) + '\n')

            #         constraints.append(
            #                 (
            #                     (
            #                         present_node.node_grid[the_time - 1][a["real_agent"] - 1],
            #                         present_node.node_grid[the_time][a["real_agent"] - 1],
            #                     ),
            #                     a["real_agent"],
            #                     a['timestep']
            #                 )
            #             )

            #     constraints.append((a['imaginary_agent'],a['timestep'] - 1, present_node.node_grid[the_time - 2][a["imaginary_agent"] - 1].location, [e for e in range(1,7) if present_node.node_grid[the_time - 1][a["imaginary_agent"] - 1].osf_dict[e] == present_node.node_grid[the_time][a["real_agent"] - 1]][0],present_node.node_grid[the_time][a["real_agent"] - 1]))
            #    # print(' Secondary Vertex Constraints = ' + str(constraints[0][:4]) + ',' + str(constraints[0][4].location))
                
                # with open('array.txt','a') as the:

                #     the.write(' Secondary Vertex Constraints = ' + str(constraints[0][:4]) + ',' + str(constraints[0][4].location) + '\n')

                constraints.append(
                        (
                            (
                                present_node.node_grid[the_time - 1][a["real_agent"] - 1],
                                present_node.node_grid[the_time][a["real_agent"] - 1],
                            ),
                            a["real_agent"],
                            a['timestep']
                        )
                    )
                
            

                with open('array.txt','a') as the:

                    the.write(' Secondary Vertex Constraints = ' + str(  (
                    (
                        present_node.node_grid[the_time - 1][a["real_agent"] - 1].location,
                        present_node.node_grid[the_time][a["real_agent"] - 1].location,
                    ),
                    a["real_agent"],
                    a['timestep']
                )) + '\n')

                # constraints.append(
                #     (a["real_agent"], a['timestep'], present_node.node_grid[the_time][a["real_agent"] - 1])   ## This will have to be changed to an edge conflict for the sake of completeness.
                # )
            
                
                print('secondary vertex constraints = ' + str(constraints[2]))
                vertex_conflict = True

            
    node_grid = present_node.node_grid

    array = [[] for _ in range(len(constraints))]
    for i in node_grid:
        for _ in range(len(array)):
            array[_].append(copy.copy(i))

    for _ in range(len(array)):
        array[_] = np.array(array[_])
    nodes_for_children = [copy.deepcopy(present_node.nodes) for i in range(len(constraints))]
    # print('in nodes for children = '+ str([x[28].explored for x in nodes_for_children]))
    # print('The constraints when two nodes are created = ' + str(constraints))
    for i in range(len(constraints)):

        next_node = high_level_node(
            nodes=nodes_for_children.pop(),
            agents=agent_list,
            conflicts=constraints[i],
            node_grid=array[i],
            reference_dict=reference_dict,
            reference_nodes=nodes_to_load,
            timestep=0,
        )  # high_level_node(nodes_for_children.pop(),agents = agent_list,conflicts = constraints[i], node_grid = array[i],reference_dict = reference_dict,reference_nodes = nodes_to_load)
        next_node.predecessor = present_node
        next_node.path_cost = present_node.path_cost
        next_node.node_cost = copy.copy(present_node.node_cost)
        next_node.path_complete_dict = copy.copy(present_node.path_complete_dict)
        next_node.timestep_start_dict = copy.copy(present_node.timestep_start_dict)
        next_node.timestep_dict = copy.copy(present_node.timestep_dict)
        next_node.secondary_conflict = copy.deepcopy(present_node.secondary_conflict)
        next_node.vertex_conflicts = copy.deepcopy(present_node.vertex_conflicts)
        next_node.agent_positions = copy.deepcopy(present_node.agent_positions)
        next_node.future_positions = copy.deepcopy(present_node.future_positions)
        next_node.imaginary_agent_timesteps = copy.deepcopy(present_node.imaginary_agent_timesteps)
        next_node.successor_agent_timesteps = copy.deepcopy(present_node.successor_agent_timesteps)

        if len(next_node.conflict) == 3:
            if type(next_node.conflict[0]) == tuple:
                a, agent, c = next_node.conflict
                if not wait:
                    next_node.nodes[
                        next_node.reference_dict[a[1].location]
                    ].constraints.append((agent, a[0].location, c))
                else:
                    next_node.nodes[
                        next_node.reference_dict[a[1].location]
                    ].waiting_constraints.append((agent, a[0].location, c))
                    wait = False
            else:

                
                agent, c, a = next_node.conflict
            
                next_node.nodes[
                    next_node.reference_dict[a.location]
                ].constraints.append((agent, c))

                # else:

                #     next_node.nodes[
                #         next_node.reference_dict[a.location]
                #     ].waiting_constraints.append((agent, c))

                #     wait = False

        else:

            agent, c, b, key, a = next_node.conflict
            next_node.nodes[
                next_node.reference_dict[a.location]
            ].turning_constraints.append((agent, c, b, key))

        if value_of_arranging == 1:
            print("This was the breaking point")
            print("Found the error")
            break

        the_solution = next_node.find_path_colregs(
            agent_num=agent,
            start_node=next_node.reference_nodes[
                next_node.reference_dict[next_node.timestep_start_dict[agent][1]]
            ],
            dest_node=next_node.agents[agent].goal_node,
            timestep=next_node.timestep_start_dict[agent][0],
        )
        if the_solution == None:
            print("None")
            time.sleep(2)

            continue
        b_2 = the_solution[4]
        if b_2 > low_level_nodes:
            low_level_nodes = b_2
        the_solution = the_solution[:4]
        # print('present_node.timestep and c = ' + str(present_node.timestep) + ' ' + str(c))
        # print('The solution[0] = '+ str([e.location for e in the_solution[0]]))
        for _ in next_node.imaginary_agent_timesteps[agent]:
            next_node.agent_positions[_][agent] = []
        next_node.imaginary_agent_timesteps[agent] = [_[1] for _ in the_solution[2]]
        for _ in the_solution[2]:
            
            if _[1] not in next_node.agent_positions:
                next_node.agent_positions[_[1]] = {}
        
            if agent not in next_node.agent_positions[_[1]]:
                next_node.agent_positions[_[1]][agent] = []
        
            next_node.agent_positions[_[1]][agent].append(_[0])
        
            # try:
            #     next_node.agent_positions[_[1]][agent] = _[0]
            # except KeyError:
            #     next_node.agent_positions[_[1]] = {}
            #     next_node.agent_positions[_[1]][agent] = _[0]

        
        for _ in next_node.successor_agent_timesteps[agent]:
            next_node.future_positions[_][agent] = 0
        
        next_node.successor_agent_timesteps[agent] = [_[1] for _ in the_solution[3]]
        
        for _ in the_solution[3]:
            try:
                next_node.future_positions[_[1]][agent] = _[0]
            except KeyError:
                next_node.future_positions[_[1]] = {}
                next_node.future_positions[_[1]][agent] = _[0]

        

        value_of_arranging = next_node.arrange_node_grid(
            returned_path=the_solution[0], agent_num=agent
        )
        
        final_array = [[x.location for x in i] for i in next_node.node_grid]
        print("array after putting thr constraints = " + str(final_array))
        print(next_node.agent_positions)
        if vertex_conflict:

            with open('array.txt','a') as the:

                the.write("array after putting thr constraints = " + str(final_array) +'\n')
                the.write(str(next_node.agent_positions) + "\n")

                
        if value_of_arranging == 1:
            print("The value of arranging = " + str(value_of_arranging))
        # print([[f.location for f in f_2] for f_2 in next_node.node_grid])
        if value_of_arranging == 1:

            print("This was the breaking point")
            break
        # print(
        #     "next_node_grid = "
        #     + str([[x.location for x in _] for _ in next_node.node_grid])
        # )
        next_node.node_cost[agent] = the_solution[1]
        next_node.path_cost = np.sum(next_node.node_cost)
        next_node.path_complete_dict[agent] = False
        # if vertex_conflict:
        #     heapq.heappush(new_list, (next_node.path_cost, id(next_node), next_node))
        if high_level.path_cost > next_node.path_cost:
            print('high level = '+ str([[x.location for x in i] for i in present_node.node_grid]))
            print([[x.location for x in i] for i in next_node.node_grid])
            time.sleep(5)
        
        heapq.heappush(open_list, (next_node.path_cost, id(next_node), next_node))
        
        if len(open_list) > high_level_nodes:
            high_level_nodes = len(open_list)
        
    
        new = False
        vertex_conflict = False
        tuple_constraints = False
        secondary = False
        if value_of_arranging == 1:
            print("Found the error")
            break

if value_of_arranging == 1:
    print(locations_2)

if a == None:

    final_array = [[x.location for x in i] for i in present_node.node_grid]
    print("final_array = " + str(final_array))
    final_array = np.array(final_array)
    final_array = final_array.T
    # print(final_array)
    truth_values = [
        list(zip(final_array[0][i], final_array[1][i]))
        for i in range(len(final_array[0]))
    ]
    print(truth_values)
    truth_values_2 = [
        [
            (
                True
                if x == 0
                or (
                    nodes_to_load[reference_dict[i[x]]]
                    in nodes_to_load[reference_dict[i[x - 1]]].osf_dict.values()
                    or i[x - 1] == i[x]
                )
                else False
            )
            for x in range(len(i))
        ]
        for i in truth_values
    ]
    print(truth_values_2)

print("locations = " + str(locations_2))
print("secondary conflicts = " + str(present_node.secondary_conflict))
print("vertex conflicts = " + str(present_node.vertex_conflicts))






