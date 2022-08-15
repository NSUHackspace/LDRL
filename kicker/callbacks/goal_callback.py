from stable_baselines3.common.callbacks import BaseCallback


class GoalCallback(BaseCallback):
    def __init__(self, verbose=0):
        super(GoalCallback, self).__init__(verbose)
        self.win_goals = 0
        self.lose_goals = 0

    def _on_step(self) -> bool:
        res = self.training_env.get_attr("ball_return")
        if res == [1]:
            self.win_goals += 1
        elif res == [-1]:
            self.lose_goals += 1
        self.logger.record("win_goals", self.win_goals)
        self.logger.record("lose_goals", self.lose_goals)
        return True
