from constants import *

agent_name_list = [
    "franka_0",
    "aliengo_0",
    "quadrotor_0"
]

room_name_list = [
    "bedroom_0",
    "childs_room_0",
    "living_room_0",
    "kitchen_0",
]



locations = {

    # birthpoint
    "birthpoint-franka": [-1.4, 7.6913, 0.78], 
    "birthpoint-aliengo": [1.7745543718338013, 7.816161632537842, aliengo_defaults["height"]], 
    "birthpoint-quadrotor": [2.7745543718338013, 0.7, quadrotor_defaults["land_height"]], 


    # targetpoint  (movetowads room_name)
    "targetpoint-bedroom_0-p1": [1.7745543718338013, 1.204, 0],
    "targetpoint-living_room_0-p1": [1.7745543718338013, 6.0485, 0],
    "targetpoint-living_room_0-p2": [0.67271,2.86984, 0],
    "targetpoint-kitchen_0-p1": [-1.23657, 2.92, 0],
    "targetpoint-kitchen_0-p2": [-0.43989, 2.86984, 0],
    "targetpoint-childs_room_0-p1": [0.67271, 1.90567, 0],

    # landpoint (land_on position_name)
    "landpoint-bedroom_0_floor-p1": [2.7745543718338013, 0.7, 0.4],
    "landpoint-living_room_0_floor-p1": [1.7745543718338013, 7.816161632537842, 0],
    "landpoint-breakfast_table_skczfi_0-p1": [-0.7, 7.45, 0.85], 

    # putpoint  (puton position_name)
    "putpoint-living_room_0_floor-p1": [3.6, 6.8, 0],
    "putpoint-coffee_table_fqluyq_0-p1": [3.5979461669921875, 7.8, 0.43435],
    "putpoint-breakfast_table_skczfi_0-p1": [-1.0, 7.6, 0.7374815106391907], 
    "putpoint-breakfast_table_wicker_basket_dgkhyn_0-p1": [-0.55, 7.78176, 0.81639],
    
    # on  (defined object initial position)
    "on-coffee_table_fqluyq_0-apple_agveuv_0": [3.5979461669921875, 7.3694, 0.43435],  # init apple
    "on-coffee_table_fqluyq_0-orange_cwcefi_0": [3.5979461669921875, 7.5694, 0.43435],  # init orange
    "on-breakfast_table_skczfi_0-bottle_of_milk_czblzn_0": [-0.7, 7.9304, 0.98],  # init milkbox
    "on-breakfast_table_skczfi_0-microwave_abzvij_0": [-1.0, 7.35, 0.88],  # init microwave

    "nearby-co ffee_table_fqluyq_0-p1": [2.94625, 7.3694, 0],  
    "nearby-coffee_table_fqluyq_0-p2": [2.94625, 7.5694, 0],  
    "nearby-coffee_table_fqluyq_0-p3": [2.94625, 7.8, 0],  
    "nearby-coffee_table_fqluyq_0-p4": [2.94625, 8.04, 0], 
    "nearby-coffee_table_fqluyq_0-p5": [2.94625, 8.24, 0],
    "nearby-landpoint-living_room_0_floor-p1": [2.35, 7.816161632537842, 0],
    "nearby-landpoint-living_room_0_floor-p2": [1.2, 7.816161632537842, 0],
    "nearby-landpoint-living_room_0_floor-p3": [1.7745543718338013, 7.24, 0],
    "nearby-landpoint-living_room_0_floor-p4": [1.7745543718338013, 8.4, 0],
    "nearby-putpoint-living_room_0_floor-p4": [1.7745543718338013, 8.4, 0],
}   


nearbypose = {
    "coffee_table_fqluyq_0": [[2.94625, 7.3694, 0, 0, 0, 0, 1],  # [(pos)x, y, z, (quat), b, c, w]
                              [2.94625, 7.5694, 0, 0, 0, 0, 1],
                              [2.94625, 7.8, 0, 0, 0, 0, 1],
                              [2.94625, 8.04, 0, 0, 0, 0, 1],
                              [2.94625, 8.24, 0, 0, 0, 0, 1]],
    "breakfast_table_skczfi_0": [[0.8, 7.6913, 0, 0, 0, 1, 0]],
    "door_duymuw_0": [[1.7745543718338013, 1.204, 0, 0.        ,  0.        , 0.70710678,  0.70710678],
                      [1.7745543718338013, 3.0, 0, 0.        ,  0.        , -0.70710678,  0.70710678]],
    "landpoint-living_room_0_floor": [[2.35, 7, 0, 0, 0, 1, 0],
                                      [1.2, 7, 0, 0, 0, 0, 1],
                                      [1.7745543718338013, 6.44, 0, 0, 0, 0.70710678, 0.70710678]],
    "putpoint-living_room_0_floor": [[1.61565, 7.60495, 0.4, 0, 0, -0.6211, 0.78373]],
}


opendoorpose = {
    "door_duymuw_0": [
                        [1.6, 2.7, 0.82, 0.0, 0.0, -0.71051, 0.70369],
                        [1.7, 2.2, 0.85, 0.0, 0.0, -0.71051, 0.70369],
                        [2.0, 2.2, 0.85, 0.0, 0.0, -0.71051, 0.70369]
                    ]
}


# roompose  (movetowads room_name)
roompose = {
    "bedroom_0-p1": [[1.7745543718338013, 1.204, 0, 0.        ,  0.        , -0.70710678,  0.70710678]],
    "living_room_0": [[1.7745543718338013, 6.0485, 0, 0.        ,  0.        , 0.70710678,  0.70710678],
                      [0.67271,2.86984, 0, 0.        ,  0.        , 0.70710678,  0.70710678]],
    "kitchen_0": [[-1.23657, 2.92, 0, 0.        ,  0.        , -0.70710678,  0.70710678],
                  [-0.43989, 2.86984, 0, 0, 0, 1, 0]],
    "childs_room_0": [[0.67271, 1.90567, 0, 0.        ,  0.        , -0.70710678,  0.70710678]]
}


putpose = {
    "living_room_0_floor-p1": [3.6, 6.8, 0, 0, 0, 0, 1],
    "coffee_table_fqluyq_0": [3.5979461669921875, 7.8, 0.43435, 0, 0, 0, 1],
    "breakfast_table_skczfi_0": [-1.0, 7.6, 0.7374815106391907, 0, 0, 0, 1],
    "breakfast_table_wicker_basket_dgkhyn_0-p1": [-0.55, 7.78176, 0.85] + [0, 0, 0, 1]
}



birthpose = {
    "franka_0": [-1.15, 7.553, 0.78, 0, 0, 0, 1],
    "aliengo_0": [1.7745543718338013, 7.816161632537842, aliengo_defaults["height"], 0, 0, 0, 1],
    "quadrotor_0": [2.7745543718338013, 0.7, quadrotor_defaults["land_height"], 0, 0, 0, 1],
    "apple_agveuv_0": locations["on-coffee_table_fqluyq_0-apple_agveuv_0"] + [0, 0, 0, 1],
    "orange_cwcefi_0": locations["on-coffee_table_fqluyq_0-orange_cwcefi_0"] + [0, 0, 0, 1],
    "bottle_of_milk_czblzn_0": locations["on-breakfast_table_skczfi_0-bottle_of_milk_czblzn_0"] + [0, 0, 0, 1],
    "wicker_basket_dgkhyn_0": locations["putpoint-breakfast_table_wicker_basket_dgkhyn_0-p1"] + [0, 0, 0, 1],
    "microwave_abzvij_0": locations["on-breakfast_table_skczfi_0-microwave_abzvij_0"] + R.from_euler('z', 45, degrees=True).as_quat().tolist(),
    "door_duymuw_0": [1.7745543718338013, 2.3250746726989746, 1.1510995626449585, 0.0, 0.0, 0.70710678, 0.70710678],
    "door_duymuw_1": [0.6894515156745911, 2.3250746726989746, 1.1510995626449585, 0.0, 0.0, 0.70710678, 0.70710678]
}


landpose = {
    "bedroom_0_floor": [2.7745543718338013, 0.7, quadrotor_defaults["land_height"], 0, 0, 0, 1],
    "living_room_0_floor": [1.7745543718338013, 7, quadrotor_defaults["land_height"], 0.        ,  0.        , 0.70710678,  0.70710678],
    "breakfast_table_skczfi_0": [-0.65, 7.42, 0.83, 0, 0, 1, 0]
}

scale = {
    "apple_agveuv_0": 0.75
}

bbox = {
    "apple_agveuv_0": [0.089999996, 0.08999999, 0.10609152],

}


articulations = {
    "door_duymuw_1": {
        "category": "door",
        "prim_path": "/World/door_duymuw_1",
        "in_rooms": ["bedroom_0", "living_room_0"],
        "indoor_handle": [[1.61391, 2.22964, 0.85], [0.70126, 0.0, 0.0, 0.71291]],
        "outdoor_handle": [[1.61391, 2.46, 0.85], [0.70369, 0.0, 0.0, -0.71051]],
        "root_link": {
            "pos": [
                1.7745543718338013,
                2.3250746726989746,
                1.1510995626449585
            ],
            "ori": [
                0.0,
                0.0,
                0.70710678,
                0.70710678
            ],
            "lin_vel": [
                0.0,
                0.0,
                0.0
            ],
            "ang_vel": [
                0.0,
                0.0,
                0.0
            ]
        },
        "joints": {
            
        }
    },
    "door_duymuw_2": {
        "category": "door",
        "prim_path": "/World/door_duymuw_2",
        "in_rooms": ["childs_room_0", "living_room_0"],
        "indoor_handle": [[0.42834, 2.22964, 0.83], [0.70126, 0.0, 0.0, 0.71291]],
        "outdoor_handle": [[0.53849, 2.22, 0.855], [0.70369, 0.0, 0.0, -0.71051]],
        "root_link": {
            "pos": [
                0.6894515156745911,
                2.3250746726989746,
                1.1510995626449585
            ],
            "ori": [
                0.0,
                0.0,
                0.70710678,
                0.70710678
            ],
            "lin_vel": [
                0.0,
                0.0,
                0.0
            ],
            "ang_vel": [
                0.0,
                0.0,
                0.0
            ]
        },
        "joints": {
            "j_link_2": {
                "pos": [
                    6.042600773047135e-13
                ],
                "vel": [
                    -1.2434497875801753e-14
                ],
                "effort": [
                    1.2190484994789585e-05
                ],
                "target_pos": [
                    0.0
                ],
                "target_vel": [
                    0.0
                ]
            }
        }
    },
    "fridge_xyejdx_0": {
        "category": "fridge",
        "prim_path": "/World/fridge_xyejdx_0",
        "in_rooms": ["living_room_0"],
        "handle": [[-0.6778, 4.1995, 1.0947], [0.70415, -0.71004, 0.0, 0.0]],
        "root_link": {
            "pos": [
                -0.1851631999015808,
                3.9132142066955566,
                0.827250599861145
            ],
            "ori": [
                2.6193447411060333e-10,
                -5.820766091346741e-11,
                0.7088045477867126,
                -0.705405056476593
            ],
            "lin_vel": [
                0.0,
                0.0,
                0.0
            ],
            "ang_vel": [
                0.0,
                0.0,
                0.0
            ]
        },
        "joints": {
            "j_link_0": {
                "pos": [
                    4.132069353346424e-09
                ],
                "vel": [
                    1.652785073247287e-07
                ],
                "effort": [
                    -6.5449607973278034e-06
                ],
                "target_pos": [
                    0.0
                ],
                "target_vel": [
                    0.0
                ]
            },
            "j_link_1": {
                "pos": [
                    -2.7319622895082335e-18
                ],
                "vel": [
                    -2.391068130980483e-16
                ],
                "effort": [
                    1.0756496521935333e-05
                ],
                "target_pos": [
                    0.0
                ],
                "target_vel": [
                    0.0
                ]
            }
        }
    }
}

