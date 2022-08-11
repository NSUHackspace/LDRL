import gym
from gym import spaces
from gym import ActionWrapper


class FlattenAction(ActionWrapper):
    """Action space wrapper that flattens the actions."""

    def __init__(self, env: gym.Env):
        """Flattens the observations of an environment.

        Args:
            env: The environment to apply the wrapper
        """
        super().__init__(env)
        self.action_space = spaces.flatten_space(env.action_space)

    def action(self, action):
        """Flattens an action.

        Args:
            action: The action to flatten

        Returns:
            The flattened action
        """
        return spaces.unflatten(self.env.action_space, action)
