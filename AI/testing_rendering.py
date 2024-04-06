import gymnasium as gym
import time
env = gym.make('CartPole-v1', render_mode="rgb_array")
env.reset()

for i in range(1000):
    env.step(env.action_space.sample())
    # env.render()
    time.sleep(0.5)
