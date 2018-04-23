#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time
import argparse
from multi_agent_deployment.unification_util import *
from multi_agent_deployment.util import *


most_recent_state_estimate = {
    # turtlename : (x, y)
}

def coords(s):
    try:
        x, y = map(float, s.split(','))
        return x, y
    except:
        raise argparse.ArgumentTypeError("coordinates must be x,y")


def make_listener((topic_name, turtle_name)):
    def callback(data):
        most_recent_state_estimate[turtle_name] = (data.pose.pose.position.x, data.pose.pose.position.y)
        #rospy.loginfo( data.pose.pose )
    print topic_name, callback
    sub = rospy.Subscriber(topic_name, Odometry, callback)
    return sub, callback 

if __name__ == "__main__":
    rospy.init_node('pytest', anonymous=True)

    turtle_names = ['tb3_0', 'tb3_1', 'tb3_2']
    sub_topics = map('/{0}/odom'.format, turtle_names)
    pub_topics = map('/{0}/waypoint'.format, turtle_names)
    listener_callback_list = map(make_listener, zip(sub_topics, turtle_names))
    publishers = [rospy.Publisher(t, Twist, queue_size=10) for t in pub_topics]
    indexes={}
    for i, t in zip(range(len(turtle_names)), turtle_names):
        indexes[t] = i  # keep a map of indexes for converting from maps to arrays n stuff
        most_recent_state_estimate[t] = (0,0)
    rate = rospy.Rate(2000)

    parser = argparse.ArgumentParser(description="unified geometric, probablistic, and potential field deployment")
    parser.add_argument("--time_steps", type=int, default=100, help='# of timesteps to sim')
    parser.add_argument("--num_agents", type=int, default=3, help='# agents to sim')
    parser.add_argument("--resolution", type=int, default=50, help='integration resolution')
    parser.add_argument("--alpha", type=float, default=-1.0, help='alpha value to parameterize mixing function')
    parser.add_argument("--step_size", type=float, default=0.05)
    parser.add_argument("--random_seed", type=int, default=42, help="random seed")
    parser.add_argument("--agent_center", type=coords, default=(0,0))    
    parser.add_argument("--agent_variance", type=float, default=1.0)
    parser.add_argument("--importance_centers", type=coords, default=[(0,0)], nargs='+')
    parser.add_argument("--boundaries", type=coords, default=[(-10, 10), (-10, 10)], nargs=2)
    parser.add_argument("--plot_vor", type=bool, default=False)
    parser.add_argument("--run_id", default=time.strftime("%Y_%m_%d-%H_%M_%S"))
    parser.add_argument("--noisiness", type=float, default=0.0)

    args = parser.parse_args()
     
    run_id = args.run_id
    TIME_STEPS = args.time_steps
    num_agents = args.num_agents
    x_min, x_max = args.boundaries[0]
    y_min, y_max = args.boundaries[1]
    res = args.resolution
    importance_centers = np.asarray(args.importance_centers)

    step_size = args.step_size
    noisiness = args.noisiness

    agent_center=args.agent_center
    agent_variance=args.agent_variance
    agents = make_random_points(num_agents, agent_variance, agent_center) 

    random_seed = args.random_seed
    np.random.seed(random_seed)

    dx = float(x_max - x_min) / float(res)
    dy = float(y_max - y_min) / float(res)
    
    x = np.linspace(x_min, x_max, res)
    y = np.linspace(y_min, y_max, res)
    X, Y = np.meshgrid(x, y)
    mesh = X,Y
    
    alpha = args.alpha
    f = quadratic_f
    grad_f = grad_quadratic_f
    phi = multivariate_gaussian_importance_func(importance_centers)        
    eval_H, grad_H = make_grad_H(alpha, f, grad_f, phi)
   
    while True:

        x_t = np.array([ np.array([most_recent_state_estimate[t][0],most_recent_state_estimate[t][1]]) for t in turtle_names ], dtype=np.float64)
        H = grad_H(x_t, mesh, dx, dy)

        # noise = np.random.randn(num_agents, 2)
        # noise_normals = np.array(map(np.linalg.norm, noise)).reshape(num_agents, 1)
        # noise = np.divide(noise, noise_normals)
        # noise = (noisiness) * noise
            
        H_normals = np.array(map(np.linalg.norm, H)).reshape(num_agents, 1)
        # H += np.multiply(H_normals, noise)
        H = np.divide(H, H_normals)

        x_t -= (step_size) * H
        print x_t

        for p, t in zip(publishers, turtle_names):
            x,y = x_t[indexes[t]]
            t = Twist()
            t.linear.x = x
            t.linear.y = y
            p.publish(t)
        time.sleep(2)
        rate.sleep()