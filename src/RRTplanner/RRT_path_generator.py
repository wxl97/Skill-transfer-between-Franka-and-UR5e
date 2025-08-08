from rrt_with_sobol_sampler import RRTSobol

def main():
    start_pose = [-0.5945910223071946, 0.4301502852490936, 0.136105098321691, -1.6266413795849624, -0.01258050409088198, 2.06070846985867, 0.3387359455803625]
    target_pose = [-0.382178, -0.643689, 0.278861, -2.401447, 0.176929, 1.800268, 0.599072]

    # Define obstacles
    obstacle_list = [
        [0.45, -0.15, 0.45, -0.92, 0.39, -0.01, 0.01, 0.02],
        [0.55, -0.25, 0.40, -0.92, 0.39, -0.01, 0.02, 0.02],
    ]

    # Initialize the RRT with Sobol sampler
    rrt = RRTSobol(
        start=start_pose,
        goal=target_pose,
        rand_area=[-3.14, 3.14],  # Adjusted sampling area
        obstacle_list=obstacle_list,
        robot_radius=0.05,  # Robot radius
        goal_sample_rate=20,  # Increased goal sampling rate
        path_resolution=0.05  # Smaller resolution for denser path
    )


    # Generate the path using RRT
    path = rrt.planning()

    # Print the generated path
    print("Generated Path:", path)

if __name__ == "__main__":
    main()