import numpy as np
import tensorflow as tf
from keras import layers, models, optimizers
import gymnasium as gym
import random
import matplotlib.pyplot as plt
import time

class AgentCartPole:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        self.memory = []
        self.gamma = 0.75  # discount rate
        self.epsilon = 1.0  # exploration rate
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.9
        self.model = self.build_model()
        self.target_model = self.build_model()
        self.target_model.set_weights(self.model.get_weights())
        self.optimizer = optimizers.Adam(learning_rate=0.05)
        self.loss_function = tf.keras.losses.Huber()

    def build_model(self):
        model = models.Sequential()
        model.add(layers.Dense(64, input_dim=self.state_size, activation='relu'))
        model.add(layers.Dense(128, activation='relu'))
        model.add(layers.Dense(self.action_size, activation='linear'))
        model.compile(loss='mse', optimizer=optimizers.Adam(learning_rate=0.05))
        return model

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))
        if len(self.memory) > 500:
            del self.memory[0]

    def act(self, state):
        if np.random.rand() <= self.epsilon:
            return np.random.choice(self.action_size)
        act_values = self.model.predict(state)
        return np.argmax(act_values[0])

    def replay(self, batch_size):
        if len(self.memory) < batch_size:
            return
        minibatch_experience = random.sample(self.memory, batch_size)
        for state, action, reward, next_state, done in minibatch_experience:
            target = reward
            if not done:
                target = reward + self.gamma * np.amax(self.target_model.predict(next_state)[0])
            target_f = self.target_model.predict(state)
            target_f[0][action] = target
            with tf.GradientTape() as tape:
                q_values = self.model(state)
                loss = self.loss_function(target_f, q_values)
            grads = tape.gradient(loss, self.model.trainable_variables)
            self.optimizer.apply_gradients(zip(grads, self.model.trainable_variables))

        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def update_target_model(self):
        self.target_model.set_weights(self.model.get_weights())

env = gym.make('CartPole-v1')
state_size = env.observation_space.shape[0]
action_size = env.action_space.n
agent = AgentCartPole(state_size, action_size)
batch_size = 100
num_episodes = 100
max_steps = 250
update_target_frequency = 5

# Train the model
for episode in range(num_episodes):
    state = env.reset()[0]
    state = np.array(state).reshape(1, -1).astype(np.float32)
    total_reward = 0

    for step in range(max_steps):
        action = agent.act(state)
        next_state, reward, done, _, _ = env.step(action)
        reward -= 0.01  # Apply the living penalty
        next_state = np.array(next_state).reshape(1, -1).astype(np.float32)
        agent.remember(state, action, reward, next_state, done)
        state = next_state
        total_reward += reward

        if done:
            break

    if len(agent.memory) > batch_size:
        agent.replay(batch_size)

    if episode % update_target_frequency == 0:
        agent.update_target_model()

    print("Episode:", episode, "Total Reward:", total_reward)

# Save the trained model
agent.model.save('cartpole_model.h5')

# Verify the trained model
saved_model = models.load_model('cartpole_model.h5')
num_test_episodes = 5
render_frequency = 1

for episode in range(num_test_episodes):
    state = env.reset()[0]
    state = np.array(state).reshape(1, -1).astype(np.float32)
    total_reward = 0

    for step in range(max_steps):
        if episode % render_frequency == 0:
            env.render()
            time.sleep(0.01)

        action = np.argmax(saved_model.predict(state)[0])
        next_state, reward, done, _ = env.step(action)
        state = np.array(next_state).reshape(1, -1).astype(np.float32)
        total_reward += reward

        if done:
            break

    print("Test Episode:", episode, "Total Reward:", total_reward)

env.close()
