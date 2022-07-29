from src.gym_envs.Kicker import KickerEnv


env = KickerEnv()

for _ in " " * 10000:
    print(env.step(env.action_space.sample()))
