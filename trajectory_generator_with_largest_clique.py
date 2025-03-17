import numpy as np
import heapq
import copy
import pickle
from functions import *
from classes_refinement import *
import random
import time
import os
import imageio
import argparse
import json


parser = argparse.ArgumentParser(description='PyTorch MNIST WAE-MMD')
parser.add_argument('--num_agents', type=int, default=18, metavar='N',
                    help='input agent num')
parser.add_argument('--starting_index', type=int, default=0, help='the index from which you would like to start')
parser.add_argument('--ending_index', type=int, default=100, help='the index from which you would like to end')
args = parser.parse_args()
info_dict = {'high_level_nodes': {},'unsolved_instances_index':[],'low_level_nodes':{},'intial_cost':{},'final_cost':{},'time_taken':{},'additional_cost':{},'number_of_agents':{}}

vertex_conflict = False
new = False
wait = False
time_limit = 0
locations_dict = {10: [], 16: [], 20: []}
single_child = False
high_level_nodes = 0
value_of_arranging = 0
child_list = []
secondary = tuple_constraints = False
with open("myobject_2.pkl", "rb") as file:
    set_of_nodes = pickle.load(file)
with open("agent_locations.pkl", "rb") as file:
    lick = pickle.load(file)
with open("problems.pkl", "rb") as file:
     problems_to_load = pickle.load(file)

location_list = problems_to_load[args.num_agents]['problem_sets'][args.starting_index:]
obstacles_nodes = []
reference_dict = set_of_nodes[3]
open_list = []
new_list = []
location_coordinates = [
    [
        (0.5, 0.5),
        (0.5, 2.23),
        (0.5, 3.96),
        (0.5, 5.7),
        (0.5, 7.43),
        (0.5, 9.16),
        (0.5, 10.89),
        (0.5, 12.62),
    ],
    [
        (2.0, 1.37),
        (3.5, 0.5),
        (5.0, 1.37),
        (6.5, 0.5),
        (8.0, 1.37),
        (9.5, 0.5),
        (11.0, 1.37),
        (12.5, 0.5),
    ],
    [
        (12.5, 2.23),
        (12.5, 3.96),
        (12.5, 5.7),
        (12.5, 7.43),
        (12.5, 9.16),
        (12.5, 10.89),
        (12.5, 12.62),
    ],
    [
        (2.0, 11.76),
        (3.5, 12.62),
        (5.0, 11.76),
        (6.5, 12.62),
        (8.0, 11.76),
        (9.5, 12.62),
        (11.0, 11.76),
    ],
]
# location_coordinates_2 = copy.deepcopy(location_coordinates)
# locations = [(6.5, 12.62), (0.5, 7.43), (0.5, 9.16), (9.5, 12.62), (3.5, 12.62), (2.0, 1.37), (0.5, 3.96), (12.5, 2.23), (0.5, 0.5), (11.0, 1.37), (2.0, 11.76), (12.5, 0.5), (12.5, 5.7), (8.0, 1.37), (0.5, 5.7), (12.5, 10.89), (0.5, 2.23), (6.5, 0.5)]
# locations = []
# number_of_agents = 18
# while len(locations) < number_of_agents:
#     coordinate_array = [0, 1, 2, 3]
#     agent_locations = []
#     while len(agent_locations) != 2:
#         coordinates = random.choice(coordinate_array)
#         coordinate_array.remove(coordinates)
#         try:
#             agent_locations.extend(random.sample(location_coordinates[coordinates], 1))
#         except ValueError:
#             continue
#         location_coordinates[coordinates].remove(agent_locations[-1])
#     locations.extend(agent_locations) 
#locations = [(3.5, 12.62), (0.5, 2.23), (6.5, 12.62), (0.5, 10.89), (11.0, 1.37), (11.0, 11.76), (8.0, 11.76), (0.5, 5.7), (12.5, 12.62), (2.0, 11.76), (8.0, 1.37), (0.5, 12.62), (0.5, 3.96), (6.5, 0.5), (0.5, 0.5), (5.0, 1.37), (3.5, 0.5), (12.5, 9.16)]
#locations = [(11.0, 1.37), (2.0, 11.76), (0.5, 2.23), (2.0, 1.37), (0.5, 7.43), (6.5, 12.62), (0.5, 3.96), (12.5, 10.89), (12.5, 0.5), (9.5, 12.62)]
problem_counter = 0
solution_dict = {}
solution_dict['clique'] = copy.deepcopy(info_dict)
solution_dict['remaining'] = copy.deepcopy(info_dict)
for the_problem in location_list[:int(args.ending_index)]:
    problem_counter += 1
    open_list = []
    high_level_nodes = 0
    low_level_nodes = 0
    the_problem = the_problem[:int(args.num_agents)]
    locations = copy.deepcopy(the_problem)
    locations_2 = copy.deepcopy(locations)
    # n = number_of_agents
    solved_clique = False
    solved = False
    print(locations)
    while not solved: 
        if not solved_clique:
            clique_graph = graph(location_list=[],final_list=locations)
            clique_graph.load_vertices()
            clique_graph.create_connections()
            largest_clique,largest_clique_complement = clique_graph.find_largest_clique()
            print('reached here')
            if len(largest_clique) > args.num_agents//4:
                new_largest_clique = []
                while len(new_largest_clique) < args.num_agents//4:
                    distances = [squared_distance(largest_clique[i][0],largest_clique[i][1]) for i in range(len(largest_clique))]
                    k = np.argmin(distances)
                    new_largest_clique.append(largest_clique[k])
                    largest_clique.remove(largest_clique[k])

                largest_clique_complement.extend(largest_clique)

            largest_clique_2 = []
            for i in new_largest_clique:
                for _ in i:  
                    largest_clique_2.append(_)

            largest_clique =  copy.deepcopy(largest_clique_2)
            largest_clique_2 = []
            for i in largest_clique_complement:
                for _ in i:  
                    largest_clique_2.append(_)
                
            largest_clique_complement = copy.deepcopy(largest_clique_2)

        if not solved_clique: 
            nodes_to_load = copy.deepcopy(set_of_nodes[1])
            for i in largest_clique_complement:
                nodes_to_load[reference_dict[i]].obstacle_node = True

            else:
                nodes_to_load = copy.deepcopy(set_of_nodes[1])

        if solved_clique:
            locations = copy.deepcopy(largest_clique_complement)
        else: 
            locations = copy.deepcopy(largest_clique)

        locations_2 = copy.copy(locations)
        locations_2.reverse()
        locations.extend(locations_2)
        #locations = [(12.5, 5.7), (0.5, 2.23), (5.0, 11.76), (0.5, 3.96), (0.5, 7.43), (12.5, 12.62), (11.0, 11.76), (12.5, 9.16), (0.5, 12.62), (9.5, 0.5), (9.5, 0.5), (0.5, 12.62), (12.5, 9.16), (11.0, 11.76), (12.5, 12.62), (0.5, 7.43), (0.5, 3.96), (5.0, 11.76), (0.5, 2.23), (12.5, 5.7)]
        #locations = [(9.5, 0.5), (12.5, 5.7), (0.5, 5.7), (12.5, 12.62), (8.0, 1.37), (0.5, 7.43), (0.5, 10.89), (12.5, 9.16), (0.5, 3.96), (12.5, 10.89), (5.0, 1.37), (12.5, 3.96), (12.5, 3.96), (5.0, 1.37), (12.5, 10.89), (0.5, 3.96), (12.5, 9.16), (0.5, 10.89), (0.5, 7.43), (8.0, 1.37), (12.5, 12.62), (0.5, 5.7), (12.5, 5.7), (9.5, 0.5)]

        # locations = [(12.5, 2.23), (0.5, 5.7), (3.5, 0.5), (6.5, 12.62), (2.0, 1.37), (0.5, 3.96), (8.0, 1.37), (12.5, 7.43), (3.5, 12.62), (0.5, 7.43), (0.5, 7.43), (3.5, 12.62), (12.5, 7.43), (8.0, 1.37), (0.5, 3.96), (2.0, 1.37), (6.5, 12.62), (3.5, 0.5), (0.5, 5.7), (12.5, 2.23)]
        # locations_2 = copy.copy(locations)
        # print(locations)
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
        high_level.timestep = 0
        initial_cost = high_level.path_cost
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

            if new:  
                with open('array.txt',"a") as the:

                    the.write("final_array of the node to be presented = " + str(final_array) + "\n")
        
            for _ in present_node.agent_positions:

                for __ in present_node.agent_positions[_]:

                    if present_node.node_grid[_][__ - 1].location in present_node.agent_positions[_][__]:
                        print(_,__)
                        print('Found the error')
                        break
        
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
                
                if not solved_clique:
                    go_to = 'clique'
                    solution_dict[go_to]['additional_cost'] = float(np.max(present_node.node_cost))
                else:
                    go_to = 'remaining'
                    solution_dict[go_to]['number_of_agents'][args.starting_index + problem_counter] = len(present_node.node_grid[0])
                present_node.output = present_node.node_grid
                print("The solution was reached")
                print("total time taken = " + str(final_time - initial_time))
                time.sleep(3)
                solution_dict[go_to]['time_taken'][args.starting_index + problem_counter] = float(final_time - initial_time)
                solution_dict[go_to]["high_level_nodes"][args.starting_index + problem_counter] = float(high_level_nodes)
                solution_dict[go_to]["intial_cost"][args.starting_index + problem_counter] = float(initial_cost)
                solution_dict[go_to]['final_cost'][args.starting_index + problem_counter] = float(present_node.path_cost)
                solution_dict[go_to]['low_level_nodes'][args.starting_index + problem_counter] = float(low_level_nodes)

                with open('results_clique' + str(args.num_agents)+ ':'+ str(args.starting_index) +'.txt', 'w') as f:
                    json.dump(solution_dict, f, indent=4) 
                    time.sleep(3)
                    print("secondary conflicts = " + str(present_node.secondary_conflict))
                if not solved_clique:
                    solved_clique = True
                    # solution_dict['clique'] = present_node.node_grid
                else:
                    solved = True
                    # solution_dict['remaining'] = present_node.node_grid
                break
            elif final_time - initial_time > 300:
                  
                if not solved_clique:
                    go_to = 'clique'
                    solved_clique = True
                else:
                    go_to = 'remaining'
                    solved = True
                
                solution_dict[go_to]['time_taken'][args.starting_index + problem_counter] = float(final_time - initial_time)
                solution_dict[go_to]["high_level_nodes"][args.starting_index + problem_counter] = float(high_level_nodes)
                solution_dict[go_to]["intial_cost"][args.starting_index + problem_counter] = float(initial_cost)
                solution_dict[go_to]['final_cost'][args.starting_index + problem_counter] = float(present_node.path_cost)
                solution_dict[go_to]['low_level_nodes'][args.starting_index + problem_counter] = float(low_level_nodes)
                solution_dict[go_to]["unsolved_instances_index"].append(args.starting_index + problem_counter - 1)
                with open('results_clique' + str(args.num_agents)+ ':'+ str(args.starting_index) +'.txt', 'w') as f:
                    json.dump(solution_dict, f, indent=4) 
                    time.sleep(3)
                    print("secondary conflicts = " + str(present_node.secondary_conflict))
                
                break

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
                    print('secondary edge constraints = ' + str(a))
                
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
                print(a)
            

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
            
                
                print(' Secondary Vertex Constraints = ' + str(  (
                    (
                        present_node.node_grid[the_time - 1][a["real_agent"] - 1].location,
                        present_node.node_grid[the_time][a["real_agent"] - 1].location,
                    ),
                    a["real_agent"],
                    a['timestep']
                )) + '\n')
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

                heapq.heappush(open_list, (next_node.path_cost, id(next_node), next_node))
                if len(open_list) > high_level_nodes:
                    high_level_nodes = len(open_list)
                
                
                # if id(present_node) in child_list:
                #     child_list.append(id(next_node))
                #     print('length of child list = ' + str(len(child_list)))

            
            new = False
            vertex_conflict = False
            tuple_constraints = False
            secondary = False
            if value_of_arranging == 1:
                print("Found the error")
                break

    if value_of_arranging == 1:
        print(locations_2)

    # if solved:
        
    #     for _ in range(2):
    #         if _  == 0:
    #             final_array = [[x.location for x in i] for i in solution_dict["clique"]]
    #             print("final_array for clique = " + str(final_array))
    #         else:
    #             final_array = [[x.location for x in i] for i in solution_dict["remaining"]]
    #             print("final_array for remaining = " + str(final_array))
    #         final_array = np.array(final_array)
    #         final_array = final_array.T
    #         # print(final_array)
    #         truth_values = [
    #             list(zip(final_array[0][i], final_array[1][i]))
    #             for i in range(len(final_array[0]))
    #         ]
    #         print(truth_values)
    #         truth_values_2 = [
    #             [
    #                 (
    #                     True
    #                     if x == 0
    #                     or (
    #                         nodes_to_load[reference_dict[i[x]]]
    #                         in nodes_to_load[reference_dict[i[x - 1]]].osf_dict.values()
    #                         or i[x - 1] == i[x]
    #                     )
    #                     else False
    #                 )
    #                 for x in range(len(i))
    #             ]
    #             for i in truth_values
    #         ]
    #         print(truth_values_2)

    # print("locations = " + str(locations_2))
    # print("secondary conflicts = " + str(present_node.secondary_conflict))
    # # print("vertex conflicts = " + str(present_node.vertex_conflicts))
