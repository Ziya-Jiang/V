# Curobo imports
from curobo.types.base import TensorDeviceType
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig

from curobo_config import CuroboConfig

def create_ik_solver(config: CuroboConfig, **kwargs):
    """
    Initialize the WorkspaceAnalyzer with robot configuration.
    
    Args:
        config (CuroboConfig): Configuration instance containing robot and world setup.
        **kwargs: Additional parameters including:
            num_seeds (int): Number of seeds for IK solver optimization (default: 20).
    """
    
    # Extract kwargs with defaults
    num_seeds = kwargs.get("num_seeds", 20)
    rotation_threshold = kwargs.get("rotation_threshold", 0.1)
    position_threshold = kwargs.get("position_threshold", 0.01)
    self_collision_check = kwargs.get("self_collision_check", True)
    self_collision_opt = kwargs.get("self_collision_opt", True)
    use_particle_opt = kwargs.get("use_particle_opt", True)
    use_gradient_descent = kwargs.get("use_gradient_descent", False)
    use_cuda_graph = kwargs.get("use_cuda_graph", True)
    
    n_obstacle_cuboids = 30
    n_obstacle_mesh = 10
    
    # Configure IK solver with collision checking
    ik_config = IKSolverConfig.load_from_robot_config(
        kwargs.get("robot_config", config.robot_cfg),
        config.world_cfg,
        rotation_threshold=rotation_threshold,
        position_threshold=position_threshold,
        self_collision_check=self_collision_check,
        self_collision_opt=self_collision_opt,
        use_particle_opt = use_particle_opt,
        num_seeds = num_seeds,
        use_cuda_graph = use_cuda_graph,
        use_gradient_descent = use_gradient_descent,
        tensor_args=TensorDeviceType(),
        collision_checker_type=CollisionCheckerType.MESH,
        collision_cache={"obb": n_obstacle_cuboids, "mesh": n_obstacle_mesh},
        # use_fixed_samples=True,
    )
    return IKSolver(ik_config)
    
